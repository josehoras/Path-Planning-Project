# Writeup for Path Planning Project

In this project I implemented the C++ algorithms for a car to drive safely on a highway simulation with other cars driving a different speeds. My car manages to adapt to the highway traffic, change lanes when safe in order to drive faster, while keeping below the 50 MPH speed limit and within its lane.

The goals for the project are described in the [Project's Rubric](https://review.udacity.com/#!/rubrics/1971/view):

- The code compiles correctly.
- The car is able to drive at least 4.32 miles without incident.
- The car drives according to the speed limit.
- Max Acceleration and Jerk are not exceeded.
- Car does not have collisions.
- The car stays in its lane, except for the time between changing lanes.
- The car is able to change lanes

All the goals were fulfilled in the final implementation of the code.

The main part of the code was implemented in `main.cpp`, while some functions where placed in `helpers.h` to improve readability and modularity to the code.

## main.cpp

The architecture to communicate with the simulator is already given here. The central part of the program begins when the simulator delivers data from line 97: `if (event == "telemetry") {`

After this the data coming from the simulator is stored in several variables and the vectors used to deliver the next path locations to our car are declared:
```
vector<double> next_x_vals;
vector<double> next_y_vals;
```

From here begins my own implementation of the code. The structure of my code can be broken down in:

- **Prediction** of the future position of my car and other cars on the road
- **Behavior Planning** of my car using costs functions to evaluate which lane to use
- **Trajectory Generation** to accelerate, keep in lane speed, and change lanes, keeping low levels of jerk and acceleration

## Prediction

On each cycle the code will feed a series of 50 path points to the simulator for our car to follow. Not all of this positions will be realized at the end of the cycle, and the simulator will feed back  the previous path points that were left over. I want to add path points from the last of the previous path position. This is a position in the future of my car that I calculate and it is from this predicted position that I refer the rest of the operations to decide the future planning and trajectory.

On the first cycle there will be of course no path leftover and the initial position will be taken as the predicted position.

The quantities of interest are the last position on Frenet coordinates, the last and the one before last positions of the previous path in map x, y coordinates. From these last two positions I calculate the car's angle and its velocity. Having these two points will also be necessary to generate the next trajectory using the spline curve.

```
pred_x = previous_path_x[path_size-1];
pred_y = previous_path_y[path_size-1];
pred_x2 = previous_path_x[path_size-2];
pred_y2 = previous_path_y[path_size-2];
pred_phi = atan2(pred_y-pred_y2,pred_x-pred_x2);
double last_dist = sqrt(pow(pred_y-pred_y2,2) + pow(pred_x-pred_x2, 2));
car.pred_vel = last_dist/0.02;
pred_s = end_path_s;
pred_d = end_path_d;
```

Finally I also calculate predicted car lane and a boolean flag that indicates whether the car is currently changing lanes.

```
car.pred_lane = pred_d / 4;
bool changing_lanes = (car.pred_lane != car.goal_lane);
```








