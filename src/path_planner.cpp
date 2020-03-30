#include "path_planner.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Dense"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// debug
using std::cout;
using std::endl;;
using nlohmann::json;

Map::Map(string map_file_){

    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x_f;
    double y_f;
    float s_f;
    float d_x;
    float d_y;
    iss >> x_f;
    iss >> y_f;
    iss >> s_f;
    iss >> d_x;
    iss >> d_y;
    x.push_back(x_f);
    y.push_back(y_f);
    s.push_back(s_f);
    dx.push_back(d_x);
    dy.push_back(d_y);
    }
}

void Car::readTelemetry(json &json_data){

    // Main car's localization Data
    x = json_data[1]["x"];
    y = json_data[1]["y"];
    s = json_data[1]["s"];
    d = json_data[1]["d"];
    yaw = json_data[1]["yaw"];
    speed = json_data[1]["speed"];

    // Previous path data given to the Planner
    previous_path_x = json_data[1]["previous_path_x"];
    previous_path_y = json_data[1]["previous_path_y"];

    // Previous path's end s and d values
    end_path_s = json_data[1]["end_path_s"];
    end_path_d = json_data[1]["end_path_d"];

    // Previous path size
    previous_path_size = previous_path_x.size();

    // Sensor Fusion Data, a list of all other cars on the same side
    //   of the road.
    sensor_fusion = json_data[1]["sensor_fusion"];
}

PathPlanner::PathPlanner(Map& map_in){
    map = map_in;
}

checkCar PathPlanner::check_cars_in_lane(Car &car)
{
    // Default values
    checkCar check_car_info;
    check_car_info.flag = false;
    check_car_info.distance = 1000;
    check_car_info.speed = 49.5;
    check_car_info.too_close = false;

    // Maximum distance from other car to react
    double safety_distance = 30;

    for(int i = 0; i < car.sensor_fusion.size(); ++i)
    {
        float d = car.sensor_fusion[i][6];
        double check_s = car.sensor_fusion[i][5];

        // If car is in the same lane and it is preceding us
        if ((floor(d/4) == ref.lane) && (check_s > car.s))
        {
            double vx = car.sensor_fusion[i][3];
            double vy = car.sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy) * 2.24;
            double check_distance = check_s - car.s;

            check_s += ((double)car.previous_path_size*0.02*check_speed/2.24);

            if((check_s > car.s) && (check_distance < safety_distance))
            {
                check_car_info.distance = check_distance;
                check_car_info.too_close = (check_distance < safety_distance);
                check_car_info.speed = check_speed;
                check_car_info.distance = check_distance;
            }

        }
    }

    return check_car_info;
}

double PathPlanner::cost_close_vehicle(const int c_lane, const double dist)
{
    // Set distance for which it could not change lane
    double fw_dist = 20;
    double bck_dist = 10;

    double cost;
    if((dist > -bck_dist) && (dist < fw_dist))
    {
        cost = 10000;
    } else
    {
        cost = 0;
    }

    return cost;
}

double PathPlanner::cost_side_vehicle(const int c_lane, const double c_speed, const double car_speed, const double dist)
{
    double range_dist = 50;
    double speed_diff = (c_speed - car_speed)/2.24;

    double cost;
    if((dist < 0) && (dist > -range_dist) && (speed_diff > dist))
    {
        cost = 25;
    }
    else if (dist < 0)
    {
        cost = dist / 10;
    }

    return cost;
}

double PathPlanner::cost_next_vehicle(const int c_lane, const double c_speed, const double car_speed, const double dist, const checkCar& l_car)
{
    double range_dist = l_car.distance + 15;
    double cost;
    if((dist > 0) && (dist < range_dist) && (c_speed < l_car.speed))
    {
        cost = 5;
    }
    else if (dist > 0)
    {
        cost = - dist/20;
    }
}

void PathPlanner::ask_lane_change(const Car &car, const checkCar &inLaneCar)
{

    // Compute lanes
    vector<int> insp_lanes;
    switch(ref.lane)
    {
        case 0: insp_lanes = {0,1}; break;
        case 1: insp_lanes = {0,1,2}; break;
        case 2: insp_lanes = {1,2}; break;
    }

    // Compute cost
    vector<double> cost(insp_lanes.size(),0);
    vector<double> no_car(insp_lanes.size(), true);

    for(int i = 0; i < car.sensor_fusion.size(); ++i)
    {
        float d = car.sensor_fusion[i][6];
        int check_lane = floor(d/4);
        auto it = std::find(insp_lanes.begin(), insp_lanes.end(), check_lane);
        if(it != insp_lanes.end() && (check_lane != ref.lane))
        {
            int idx = it - insp_lanes.begin();
            double vx = car.sensor_fusion[i][3];
            double vy = car.sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy) * 2.24;
            double check_car_s = car.sensor_fusion[i][5];
            double check_distance = check_car_s-car.s;

            // Count cars in lane
            no_car[idx] = false;

            // Penalize lane change if there any cars close to the vehicle on near lane(s)
            cost[idx] += cost_close_vehicle(check_lane, check_distance);

            // Penalize lane change if cars in the side mirrors are coming too fast
            cost[idx] += cost_side_vehicle(check_lane, check_speed, car.speed, check_distance);

            // Penalize lane change if cars in the side mirrors are coming too fast
            cost[idx] += cost_next_vehicle(check_lane, check_speed, car.speed, check_distance, inLaneCar);
        }
    }

    // Reward lane if there are no cars
    for(int k = 0; k < no_car.size(); ++k)
    {
        if((insp_lanes[k] != ref.lane) && no_car[k])\
        {
            cost[k] -= 50;
        }
    }

    // Stop from returning to previous lane before 3 s from last lane change
    int previous_lane_idx = std::find(insp_lanes.begin(), insp_lanes.end(), previous_lane) - insp_lanes.begin();
    if(!lane_change_timer.is_ready()) { cost[previous_lane_idx] += 1000; }

    // Select lanes with smallest cost
    int best_cost_idx = std::min_element(cost.begin(), cost.end()) - cost.begin();
    int best_lane = insp_lanes[best_cost_idx];

    previous_lane = ref.lane;
    ref.lane = best_lane;
    if(best_lane != previous_lane) {lane_change_timer.set_timer(3.0); }
}

vector<vector<double>> PathPlanner::generate_trajectory(Car &car)
{

    // Check if there are slower cars in ego vehicle's lane
    checkCar check_info = check_cars_in_lane(car);

    // If car ahead is too slow, check if lane change is both feasible and convenient
    lane_change_timer.update_timer(0.02);
    if(check_info.too_close)
    {
        ask_lane_change(car, check_info);
    }

    // Vectors of x, y trajectory coordinates
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // Widely spaced points
    vector<double> ptsx;
    vector<double> ptsy;

    // Car reference state
    ref.x = car.x;
    ref.y = car.y;
    ref.yaw = deg2rad(car.yaw);

    if (car.previous_path_size < 2)
    {
        double prev_car_x = car.x - cos(car.yaw);
        double prev_car_y = car.y - sin(car.yaw);

        ptsx.push_back(prev_car_x);
        ptsy.push_back(prev_car_y);

        ptsx.push_back(car.x);
        ptsy.push_back(car.y);

        // Initial values
        ref.speed = 0;
        ref.lane = 1;
    }
    else
    {
        ref.x = car.previous_path_x[car.previous_path_size-1];
        ref.y = car.previous_path_y[car.previous_path_size-1];

        double ref_x_prev = car.previous_path_x[car.previous_path_size-2];
        double ref_y_prev = car.previous_path_y[car.previous_path_size-2];
        ref.yaw = atan2(ref.y - ref_y_prev, ref.x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsy.push_back(ref_y_prev);

        ptsx.push_back(ref.x);
        ptsy.push_back(ref.y);
    }

    vector<double> ref_frenet = getFrenet(ref.x, ref.y, ref.yaw, map.x, map.y);
    double ref_s = ref_frenet[0];

    // Add 3 evenly-spaced points in Frenet reference frame
    vector<double> next_p0 = getXY(ref_s+30, (2+4*ref.lane), map.s, map.x, map.y);
    vector<double> next_p1 = getXY(ref_s+60, (2+4*ref.lane), map.s, map.x, map.y);
    vector<double> next_p2 = getXY(ref_s+90, (2+4*ref.lane), map.s, map.x, map.y);
    // vector<double> next_p3 = getXY(ref_s+80, (2+4*ref.lane), map.s, map.x, map.y);

    ptsx.push_back(next_p0[0]);
    ptsx.push_back(next_p1[0]);
    ptsx.push_back(next_p2[0]);
    // ptsx.push_back(next_p3[0]);

    ptsy.push_back(next_p0[1]);
    ptsy.push_back(next_p1[1]);
    ptsy.push_back(next_p2[1]);
    // ptsy.push_back(next_p3[1]);

    for(int i = 0; i < ptsx.size(); ++i)
    {
        // shift car reference angle to 0 degrees
        double shift_x = ptsx[i] - ref.x;
        double shift_y = ptsy[i] - ref.y;

        ptsx[i] = (shift_x * cos(0 - ref.yaw) - shift_y*sin(0-ref.yaw));
        ptsy[i] = (shift_x * sin(0 - ref.yaw) + shift_y*cos(0-ref.yaw));
    }

    double max_acc = .25;

    // Set trajectory speed
    if(check_info.too_close && (ref.speed > check_info.speed))
    {
        ref.speed -= std::min(max_acc,(ref.speed - check_info.speed));
    }
    else if (ref.speed < check_info.speed)
    {
        ref.speed += std::min(max_acc, (check_info.speed - ref.speed));
    }

    tk::spline s;

    s.set_points(ptsx, ptsy);

    for(int i = 0; i < car.previous_path_size; ++i)
    {
        next_x_vals.push_back(car.previous_path_x[i]);
        next_y_vals.push_back(car.previous_path_y[i]);
    }

    // Calculate how to break up spline points
    double target_x = 20;
    double target_y = s(target_x);
    double target_dist = sqrt(pow(target_x,2) + pow(target_y, 2));
    double x_add_on = 0;

    for(int i = 1; i <= 25-car.previous_path_size; ++i)
    {
        double N = (target_dist / (.02*ref.speed/2.24));
        double x_point = x_add_on + (target_x)/N;
        double y_point = s(x_point);
        x_add_on = x_point;

        // From local to global coordinates
        double x_ref = x_point;
        double y_ref = y_point;
        x_point = (x_ref * cos(ref.yaw) - y_ref * sin(ref.yaw));
        y_point = (x_ref * sin(ref.yaw) + y_ref * cos(ref.yaw));
        x_point += ref.x;
        y_point += ref.y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    vector<vector<double>> next_vals {next_x_vals, next_y_vals};

    return next_vals;
}

vector<double> PathPlanner::JMT(vector<double> &start, vector<double> &end, double T) {

    MatrixXd A = MatrixXd(3, 3);
    A << T*T*T, T*T*T*T, T*T*T*T*T,
        3*T*T, 4*T*T*T,5*T*T*T*T,
        6*T, 12*T*T, 20*T*T*T;

    MatrixXd B = MatrixXd(3,1);
    B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
        end[1]-(start[1]+start[2]*T),
        end[2]-start[2];

    MatrixXd Ai = A.inverse();

    MatrixXd C = Ai*B;

    vector <double> result = {start[0], start[1], .5*start[2]};

    for(int i = 0; i < C.size(); ++i) {
    result.push_back(C.data()[i]);
    }

    return result;
}