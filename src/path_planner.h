#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <fstream>
#include <iostream>
#include <string>
#include <vector>


class PathPlanner{
    public:

    PathPlanner() {};

    virtual ~PathPlanner() {};

    std::vector<std::vector<double>> generate_trajectory(
        int lane, double dist_inc, int traj_size,
        double car_s,
        std::vector<double> map_s, std::vector<double> map_x,
        std::vector<double> map_y
    );

    private:
};

#endif //  PATHPLANNER_H_

