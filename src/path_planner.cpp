#include "path_planner.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

using std::vector;

vector<vector<double>> PathPlanner::generate_trajectory(
        int lane, double dist_inc, int traj_size,
        double car_s,
        std::vector<double> map_s, std::vector<double> map_x,
        std::vector<double> map_y){

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    for(int i = 0; i < traj_size; ++i){
        double next_s = car_s + (i+1)*dist_inc;
        double next_d = 2 + lane*4; // fixed lane
        vector<double> xy = getXY(next_s, next_d, map_s, map_x, map_y);

        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }

    vector<vector<double>> next_vals {next_x_vals, next_y_vals};

    return next_vals;
}