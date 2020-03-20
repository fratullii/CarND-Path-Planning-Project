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

vector<vector<double>> PathPlanner::generate_trajectory(){

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // lane where to start
    int lane = 1;

    // reference velocity
    double vel_ref = 49.5;

    // Widely spaced points
    vector<double> ptsx;
    vector<double> ptsy;

    // int lane = 1;
    // double dist_inc = 0.4;
    // int traj_size = 50;
    // for(int i = 0; i < traj_size; ++i){
    //     double next_s = car_s + (i+1)*dist_inc;
    //     double next_d = 2 + lane*4; // fixed lane
    //     vector<double> xy = getXY(next_s, next_d, map_s, map_x, map_y);
    //     next_x_vals.push_back(xy[0]);
    //     next_y_vals.push_back(xy[1]);
    // }

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