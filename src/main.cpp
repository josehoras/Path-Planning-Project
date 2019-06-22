#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;

double MPH2MPS = 0.447;
double MPS2MPH = 2.237;

struct Car{
  int pred_lane;
  int goal_lane;
  double pred_vel;
  double goal_vel;
  string state;
};

Car car;

int main() {
  // Initialize car variables
  car.pred_lane = 1;
  car.goal_lane = 1;
  car.pred_vel = 0;
  car.goal_vel = 0;
  car.state = "KL";

  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          double speed_limit = 49.5 * MPH2MPS;

          double pred_x;
          double pred_y;
          double pred_x2;
          double pred_y2;
          double pred_s;
          double pred_d;
          double pred_phi;

      /** PREDICTION - Next the future position and relevant magnitudes of my car
       * at the end of the previous path are calculated, and stored in prediction variables
       */
          int path_size = previous_path_x.size();
          if (path_size == 0) {
            pred_x = car_x;
            pred_y = car_y;
            pred_x2 = car_x - cos(car_yaw);
            pred_y2 = car_y - sin(car_yaw);
            pred_phi = deg2rad(car_yaw);
            pred_s = car_s;
            pred_d = car_d;
          }
          else {
            pred_x = previous_path_x[path_size-1];
            pred_y = previous_path_y[path_size-1];
            pred_x2 = previous_path_x[path_size-2];
            pred_y2 = previous_path_y[path_size-2];
            pred_phi = atan2(pred_y-pred_y2,pred_x-pred_x2);
            double last_dist = sqrt(pow(pred_y-pred_y2,2) + pow(pred_x-pred_x2, 2));
            car.pred_vel = last_dist/0.02;
            pred_s = end_path_s;
            pred_d = end_path_d;
          }
          cout << "Speed: " << car.pred_vel  * MPS2MPH << endl;
          cout << "Phi: " << pred_phi << endl;

          car.pred_lane = pred_d / 4;
          bool changing_lanes = (car.pred_lane != car.goal_lane);

          /** Calculate speed and distance to the next front and back car
           * on each lane */
          vector<double> front_car_dist(3, 500);
          vector<double> back_car_dist(3, 500);
          vector<double> front_car_vel(3, -1);
          vector<double> back_car_vel(3, -1);

          for(int check_lane = 0; check_lane<3; ++check_lane){
            // First get a collection of cars in the lane not far from us
            vector<int> cars_in_lane;
            for(int i=0; i<sensor_fusion.size(); ++i) {
              double other_s = sensor_fusion[i][5];
              double other_d = sensor_fusion[i][6];
              // If car is in our lane and is up to 50m in front or behind us
              if(int(other_d / 4) == check_lane && abs(other_s - car_s) < 50){
                cars_in_lane.push_back(i);
              }
            }
            // Get the closest car in front or the back, its speed and position
            // when we will be at car_s after path_size
            for(int i=0;i<cars_in_lane.size();++i){
              double vx = sensor_fusion[cars_in_lane[i]][3];
              double vy = sensor_fusion[cars_in_lane[i]][4];
              double other_v = sqrt(vx*vx+vy*vy);
              double other_s = sensor_fusion[cars_in_lane[i]][5];
              double other_pred_s = other_s + other_v * path_size * 0.02;

              if(other_pred_s > pred_s && (other_pred_s - pred_s) < front_car_dist[check_lane]){
                front_car_dist[check_lane] = other_pred_s - pred_s;
                front_car_vel[check_lane] = other_v;
              }
              if(other_pred_s <= pred_s && (pred_s - other_pred_s) < back_car_dist[check_lane]){
                back_car_dist[check_lane] = pred_s - other_pred_s;
                back_car_vel[check_lane] = other_v;
              }
            }
          }
          /** Calculate maximal speed on each lane based on the front car. **/
          vector<double> max_speed(3, speed_limit);
          for(int check_lane = 0; check_lane<3; ++check_lane){
            if(front_car_dist[check_lane] < 30){
              double A = (speed_limit - front_car_vel[check_lane]) / 15;
              double B = 2*front_car_vel[check_lane] - speed_limit;
              max_speed[check_lane] = A * front_car_dist[check_lane] + B;
            }
            else{
              max_speed[check_lane] = speed_limit;
            }
          }
          /** Printing output for debugging **/
          for(int i=0; i<max_speed.size(); ++i){
            cout << i << ": " << max_speed[i] * MPS2MPH<< "  /  ";
          }
          cout << endl;
          for(int i=0; i<max_speed.size(); ++i){
            cout << "s: " << front_car_dist[i] << "  /  ";
          }
          cout << endl;
          for(int i=0; i<max_speed.size(); ++i){
            cout << "b: " << back_car_dist[i] << "  /  ";
          }
          cout << endl;

          /** BEHAVIOR PLANNING - Based on the car's current state, I get a list of the possible states
           * available to the car */
          vector<string> states;
          states.push_back("KL");
          if(car.state == "KL") {
            if (car.pred_lane != 0) { states.push_back("LCL"); }
            if (car.pred_lane != 2) { states.push_back("LCR"); }
          }
          else if (car.state == "LCL") { states.push_back("LCL");}
          else if (car.state == "LCR") { states.push_back("LCR");}

          /** Next I calculate cost for each possible state, and the state with lower
           * cost is selected */
          vector<double> state_cost;
          for(int i=0; i < states.size(); ++i){
            double cost = calculate_cost(states[i], car.pred_lane, changing_lanes,
                                          speed_limit, max_speed, front_car_dist, back_car_dist);
            state_cost.push_back(cost);
          }
          // Choose lower cost
          double min_cost = 100.0;
          string pref_state;
          for(int i=0; i<states.size(); ++i){
            if(state_cost[i] < min_cost){
              min_cost = state_cost[i];
              pref_state = states[i];
            }
          }
          car.state = pref_state;

          /** Printing output for debugging **/
          for(int i=0; i<states.size(); ++i){
            cout << states[i] << ": " << state_cost[i] << "  /  ";
          }
          cout << "   ---   " << car.state << endl;

          /** TRAJECTORY GENERATION - Generate trajectory parameters */
          vector<double> goals;
          goals = set_goals(car.state, car.pred_lane, max_speed);
          car.goal_lane = goals[0];
          car.goal_vel = goals[1];

          /** Printing output for debugging **/
          cout<<"Goal lane: "<<car.goal_lane<<", Goal vel: "<<car.goal_vel * MPS2MPH<<endl;
          cout << endl;

          /** Adjust car velocity to the goal speed */
          if(car.goal_vel > car.pred_vel){
            if((speed_limit - car.pred_vel) < 1)  // Be cautious close to the speed limit!
              car.pred_vel += 0.1;
            else if(car.pred_vel < 1)             // We can hit the gas more when we are almost stopped
              car.pred_vel += 2;
            else
              car.pred_vel += 0.35;
          }
          else if(car.goal_vel < car.pred_vel){
            car.pred_vel -= 0.35;
          }

          /** Based on Set XY points bassed on current goal speed and goal lane,
           * the trajectory is generated using the spline library,
           * that will generate a smooth curve and keep the jerk and acceleration
           * from exceding the upper limits*/

          vector<double> X, Y;
          vector<double> coord;
          X.push_back(pred_x2);
          Y.push_back(pred_y2);
          X.push_back(pred_x);
          Y.push_back(pred_y);
          for (int i = 1; i < 4; ++i){
            double next_s = pred_s + i * 30;
            double next_d = (2 + 4 * car.goal_lane);
            coord = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            X.push_back(coord[0]);
            Y.push_back(coord[1]);
          }

          // Transform spline anchor points to the car's coordinates
          for (int i=0; i<X.size(); ++i){
            double shift_x = X[i] - pred_x;
            double shift_y = Y[i] - pred_y;
            X[i] = (shift_x * cos(0-pred_phi) - shift_y*sin(0-pred_phi));
            Y[i] = (shift_x * sin(0-pred_phi) + shift_y*cos(0-pred_phi));
          }
          // Create spline
          tk::spline s;
          s.set_points(X,Y);
          // Calculate the path points along the spline for the car to follow each 0.02 seconds
          double step_dist =  car.pred_vel * 0.02;
          double new_car_x = 0;
          double new_car_y = 0;
          for (int i = 0; i < 50; ++i) {
            if (i < path_size) {    // Store and do not change the previous path in the simulator
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
            else {                  // Set rest of the points along the spline set apart by step_dist
              new_car_x += step_dist;
              new_car_y = s(new_car_x);
              // Change back to map coordinates and append to buffer
              double new_x = pred_x + new_car_x * cos(pred_phi) - new_car_y * sin(pred_phi);
              double new_y = pred_y + new_car_x * sin(pred_phi) + new_car_y * cos(pred_phi);
              next_x_vals.push_back(new_x);
              next_y_vals.push_back(new_y);
            }
          }

          /** Next the path and relevant data is sent to the simulator */

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
