#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "json.hpp"
#include "spline.h"

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

struct CheckCar {
    bool flag;
    bool too_close;
    double speed;
    double distance;
};

class Timer {
    private:

    double time = 0;

    public:

    inline void set_timer(double t) {time = t; };
    inline double get_time() {return time; };

    inline void update_timer(double time_step) {
        time = std::max(0., time-time_step);
    };
    inline bool is_ready() {return (time == 0);};
};

class PathPlanner{

    public:

    PathPlanner(Map&);

    virtual ~PathPlanner() {};

    /**
     * @brief Generate trajectory for highway driving
     * 
     * Writes on two vectors containing the trajectory coordinates the car
     * will have to follow
     * 
     * @param next_x_vals vector<double> with trajectory x points
     * @param next_y_vals vector<double> with trajectory x points
     * @param car Car object with ego vehicle's telemetry information
     */
    void generate_trajectory( std::vector<double>& next_x_vals,
                              std::vector<double>& next_y_vals,
                              Car &car );

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

    Map map;
    CheckCar check_cars_in_lane(Car &car);
    Timer lane_change_timer;
    int previous_lane;
    double acc;
    double max_speed;

    void ask_lane_change(const Car &car, const CheckCar &inLaneCar);

    void compute_trajectory( std::vector<double>& next_x_vals,
                             std::vector<double>& next_y_vals, 
                             Car& car );

    tk::spline compute_spline(Car &car);

    void select_speed(const CheckCar& check_info);

    
    /**
     * @brief Cost of close vehicle in other lane
     * 
     * Check if the vehicle in the other lane is close to the ego vehicle and
     * computes the cost
     * 
     * @param c_lane other vehicle lane
     * @param dist relative distance between other vehicle and ego vehicle
     * @return double cost
     */
    double cost_close_vehicle( const int c_lane,
                               const double dist );

    /**
     * @brief Cost of vehicle behind
     * 
     * Check if the vehicle in the other lane and behind the ego vehicle is
     * going too fast and computes the cost
     * 
     * @param c_lane other vehicle's lane
     * @param c_speed other vehicle's speed
     * @param car_speed ego vehicle's speed
     * @param dist relative distance between other vehicle and ego vehicle
     * @return double cost
     */
    double cost_incoming_vehicle( const int c_lane,
                                  const double c_speed,
                                  const double car_speed,
                                  const double dist );

    /**
     * @brief Cost of vehicle ahead
     * 
     * Check if the vehicle in the other lane and ahead of the ego vehicle
     * are fast going fast enough to make the lane change maneuver convenient
     * 
     * @param c_lane other vehicle's lane
     * @param c_speed other vehicle's speed
     * @param car_speed ego vehicle's speed
     * @param dist relative distance between other vehicle and ego vehicle
     * @param l_car next vehicle in the ego vehicle's lane
     * @return double cost
     */
    double cost_next_vehicle( const int c_lane,
                              const double c_speed,
                              const double car_speed,
                              const double dist,
                              const CheckCar& l_car );

};

#endif //  PATHPLANNER_H_

