# Path Planning

This documents aims at explaining how the path planner implemented in this project works.

From a functional and logic point of view, the planner is composed of three parts:

- A method that checks for slower vehicles ahead in the same lane
- A method that evaluate lane change, only when there is another slower vehicle in the same lane
- A part of the code dedicated to implement the desired behavior into a trajectory that can be followed

### Check for slower vehicles in the same lane

This operation is implemented in the `check_cars_in_lane` lane method.  It simply iterates through all the car detected by the sensor fusion part and checks the ones in the same lane, identifying the closest one and reporting the closest one. It raises a flag if this vehicle is too close (i.e. if the distance is below a certain threshold).

### Lane change

This is the behavioral planning part. When `check_cars_in_lane` reports a slower vehicle in the same lane and below a certain threshold, the planner uses the method `ask_lane_change` whether the vehicle should keep the lane or should perform a lane change and, in case of multiple lanes, which one it should pick.

The logic behind of the lane change is to perform a lane change whenever it is safe and convenienty. Safety means not only avoid collisions but also behing far away enough from other vehicles. 

Regarding convenience, the lane change is a greedy optimizer. It performs the locally optimal choice, so in some particular scenarios the choice of changing may end up being non convenient. Nevertheless, it has been observed as capable of optimally solving all the simpler situations (where the local optimum corresponds to the global optimum) and even some of the more complex ones.

`ask_lane_change` operates by computing the cost of each lane and indicating the lane with the smallest cost. Only feasible lanes are evaluated: the vehicle cannot switch to two lanes and it has to wait at least 3 s before returning to lane it was previously following. Now let's dive into the code:

First off, a lane is immediately rewarded with a negative cost when it is empty.

Then three cases are considered, implemented by three methods used iterating again through all the sensor fusion vehilce:

- `cost_close_vehicle` punishes with a high cost a lane if a car is very close to the vehicle. We want to avoid collisions, so the cost increase is very high.
- `cost_incoming_vehicle` checks if the sensed vehicle is behind our vehicle and in the other lane: if it fast enough to risk a collision, it punishes with a high cost for the same collision avoidance reason. Otherwise, it rewards with a negative cost proportional to the distance from the other vehicle
- `cost_next_vehicle` checks if the sensed vehicle is ahead of us and its speed. If it is going too slow, the cost increases. This is not a safety-related measure but rather a performance-related measure. It would be reasonable to change lane if we cannot pass the vehicle which is currently slowing us down in our lane. In fact, if the vehicle ahead of us is fast enough, the cost decreases proportionally to the distance from it.

The cost reward is higher in `cost_incoming_vehicle` than in `cost_next_vehicle`, since if two lanes are safe, we would rather go in the one with no vehicles ahead.

### Compute trajectory

Once the high-level behavior has been defined, it is possible to compute the trajectory.

The spatial path is computed by means of a spline passing through 3 points in the lane of interest, ahead respectively of 30, 60 and 90 m. The trajectory is computed by linearing the next 20 m, assuming constant speed and sampling the points from the spline, so that we end up with the final trajectory. This operation is implemented in the `compute_trajectory` method.

At every cycle, the planner receives the old trajectory and check how many points the car followed since the last cycle. Then, after computing the new path, it appends to the trajectory only as many point as the amount needed to fill the trajectory, which has a limited size. The size is a parameter that can be tuned to optimize perfomance (in this project is 25).