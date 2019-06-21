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

- Prediction of the future position of my car and other cars on the road
- Behavior Planning of my car using costs functions to evaluate which lane to use
- Trajectory Generation to accelerate, keep in lane speed, and change lanes, keeping low levels of jerk and acceleration

## Prediction