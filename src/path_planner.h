#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "json.hpp"

struct Map {

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> s;
    std::vector<double> dx;
    std::vector<double> dy;

    /**
     * @brief Default constructor
     */
    Map(){};

    /**
     * @brief Construct a new Map object
     * 
     * @param map_file_ string where to read a map file from
     */
    Map(std::string map_file_);

};

struct Car {

    // Constructor
    Car() {};

    // Main car's localization Data
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;

    // Previous path data
    nlohmann::json previous_path_x;
    nlohmann::json previous_path_y;

    // End points
    double end_path_s;
    double end_path_d;

    // Previous point size
    int previous_path_size;

    // Sensor fusion
    nlohmann::json sensor_fusion;

    /**
     * @brief Set all the new variables from telemetry
     *
     * @param json_data json object containing telemetry variables
     */
    void readTelemetry(nlohmann::json &json_data);
};

class PathPlanner{
    public:

    PathPlanner(Map&);

    virtual ~PathPlanner() {};

    std::vector<std::vector<double>> generate_trajectory(Car &car);

    private:
    /**
     * @brief Reference state
     *
     * State of the last point in the trajectory
     */
    struct Reference {
        double x;
        double y;
        double yaw;
        double speed;
        double acc;
        double s;
        double d;
        int lane;
    } ref;

    struct checkCar {
        bool too_close;
        double speed;
    };

    Map map;

    checkCar check_cars_in_lane(Car &car);

    int ask_lane_change(Car &car, checkCar &inlaneCar);

    /**
     * Calculate the Jerk Minimizing Trajectory that connects the initial state
     * to the final state in time T.
     *
     * @param start - the vehicles start location given as a length three array
     *   corresponding to initial values of [s, s_dot, s_double_dot]
     * @param end - the desired end state for vehicle. Like "start" this is a
     *   length three array.
     * @param T - The duration, in seconds, over which this maneuver should occur.
     *
     * @return an array of length 6, each value corresponding to a coefficent in 
     *   the polynomial:
     *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
     *
     * EXAMPLE
     *   > JMT([0, 10, 0], [10, 10, 0], 1)
     *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
     */
    std::vector<double> JMT(std::vector<double> &start, std::vector<double> &end, double T);
};

#endif //  PATHPLANNER_H_

