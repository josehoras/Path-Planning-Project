# Path Planning Project

[Self-Driving Car Engineer Nanodegree Program](https://eu.udacity.com/course/self-driving-car-engineer-nanodegree--nd013)

In this project I implemented the C++ algorithms for a car to drive safely on a highway simulation with other cars driving a different speeds. My car manages to adapt to the highway traffic speed, changing lanes when safe in order to drive faster, and keeping below the 50 MPH speed limit and within its lane.

The main elements implemented are:

- Prediction of the future position of my car and other cars on the road
- Behavior Planning of my car using costs functions to evaluate which lane to use
- Trajectory Generation to accelerate, keep in lane speed, and change lanes, keeping low levels of jerk and acceleration

The deliverables for the project are:

- [C++ code](./Traffic_Sign_Classifier.ipynb) contained in helpers.cpp and main.cpp
- [A writeup report](./writeup.md) (markdown)

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Installation and Build

This project involves the Term3 Simulator which can be downloaded [here](ttps://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

Once you clone this repo, it includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

Once the install for uWebSocketIO is complete, the main program can be built and run by:
- Clone this repo.
- Make a build directory: `mkdir build && cd build`
- Compile: `cmake .. && make`
- Run it: `./path_planning`.

Refer to the [Udacity project repository](https://github.com/udacity/CarND-Path-Planning-Project) for more detail installation instructions.
   
## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```







