#include "path_planner.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
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

vector<vector<double>> PathPlanner::generate_trajectory(Car &car){

    // Vectors of x, y trajectory coordinates
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // lane where to start
    int lane = 1;

    // reference velocity
    double ref_vel = 49.5;

    // Widely spaced points
    vector<double> ptsx;
    vector<double> ptsy;

    // Car reference state
    double ref_x = car.x;
    double ref_y = car.y;
    double ref_yaw = deg2rad(car.yaw);

    if (car.previous_path_size < 2){

        double prev_car_x = car.x - cos(car.yaw);
        double prev_car_y = car.y - sin(car.yaw);

        ptsx.push_back(prev_car_x);
        ptsy.push_back(prev_car_y);

        ptsx.push_back(car.x);
        ptsy.push_back(car.y);

    } else {

        ref_x = car.previous_path_x[car.previous_path_size-1];
        ref_y = car.previous_path_y[car.previous_path_size-1];

        double ref_x_prev = car.previous_path_x[car.previous_path_size-2];
        double ref_y_prev = car.previous_path_y[car.previous_path_size-2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsy.push_back(ref_y_prev);

        ptsx.push_back(ref_x);
        ptsy.push_back(ref_y);

    }

    vector<double> ref_frenet = getFrenet(ref_x, ref_y, ref_yaw, map.x, map.y);
    double ref_s = ref_frenet[0];

    // Add 4     evenly-spaced points in Frenet reference frame
    vector<double> next_p0 = getXY(ref_s+20, (2+4*lane), map.s, map.x, map.y);
    vector<double> next_p1 = getXY(ref_s+40, (2+4*lane), map.s, map.x, map.y);
    vector<double> next_p2 = getXY(ref_s+60, (2+4*lane), map.s, map.x, map.y);
    vector<double> next_p3 = getXY(ref_s+80, (2+4*lane), map.s, map.x, map.y);

    ptsx.push_back(next_p0[0]);
    ptsx.push_back(next_p1[0]);
    ptsx.push_back(next_p2[0]);
    ptsx.push_back(next_p3[0]);

    ptsy.push_back(next_p0[1]);
    ptsy.push_back(next_p1[1]);
    ptsy.push_back(next_p2[1]);
    ptsy.push_back(next_p3[1]);

    for(int i = 0; i < ptsx.size(); ++i)
    {
        // shift car reference angle to 0 degrees
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y*sin(0-ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y*cos(0-ref_yaw));

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

    for(int i = 1; i <= 50-car.previous_path_size; ++i)
    {
        double N = (target_dist / (.02*ref_vel/2.24));
        double x_point = x_add_on + (target_x)/N;
        double y_point = s(x_point);
        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // Rotate back
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    cout << endl;

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