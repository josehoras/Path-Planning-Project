#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
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

string car_state = "KL";

int main() {
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
          //double lane = 1;
          double speed_limit = 49.5;

          double pos_x;
          double pos_y;
          double angle;
          double pos_s;
          double pos_d;
          double car_v = car_speed;
          int path_size = previous_path_x.size();
          // cout << car_d << endl;


          if (path_size == 0) {
            pos_x = car_x;
            pos_y = car_y;
            angle = deg2rad(car_yaw);
            pos_s = car_s;
            pos_d = car_d;
          }
          else {
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];
            double pos_x2 = previous_path_x[path_size-2];
            double pos_y2 = previous_path_y[path_size-2];
            angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
            pos_s = end_path_s;
            pos_d = end_path_d;
          }

          int goal_lane = pos_d / 4;
          int current_lane = car_d / 4;
          //cout << car_state << endl;


          // Calculate max speed, next front and next back car on each lane
          vector<double> max_speed(3);
          vector<double> front_car_dist(3);
          vector<double> back_car_dist(3);
          for(int check_lane = 0; check_lane<3; ++check_lane){
            vector<int> cars_in_lane;
            for(int i=0; i<sensor_fusion.size(); ++i) {
              double check_car_s = sensor_fusion[i][5];
              double check_car_d = sensor_fusion[i][6];
              // If car is in our lane and is up to 100m in front or behind us
              if(int(check_car_d / 4) == check_lane &&
                 abs(check_car_s - pos_s) < 50){
                //cout << check_car_s << ", " << car_s << endl;
                cars_in_lane.push_back(i);
              }
            }
            // For cars in the lane in front of us, get the closest, its speed and position
            // when we will be at car_s after path_size
            front_car_dist[check_lane] = 500.0;
            back_car_dist[check_lane] = 500.0;
            max_speed[check_lane] = speed_limit;
            for(int i=0;i<cars_in_lane.size();++i){
              double vx = sensor_fusion[cars_in_lane[i]][3];
              double vy = sensor_fusion[cars_in_lane[i]][4];
              double other_car_v = sqrt(vx*vx+vy*vy);
              double other_car_current_s = sensor_fusion[cars_in_lane[i]][5];
              double other_car_s = other_car_current_s + other_car_v * path_size * 0.02;

              if(other_car_s > pos_s && (other_car_s - pos_s) < front_car_dist[check_lane]){
                front_car_dist[check_lane] = other_car_s - pos_s;
                // 1 m/s = 3600 m/h = 3.6 km/h = 2.237 m/h
                max_speed[check_lane] = other_car_v * 2.237;
              }
              if(other_car_s < pos_s && (pos_s - other_car_s) < back_car_dist[check_lane]){
                back_car_dist[check_lane] = pos_s - other_car_s;
              }
            }
          }
          for(int i=0; i<max_speed.size(); ++i){
            cout << i << ": " << max_speed[i] << "  /  ";
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
          // Adjust car velocity to our car lane
          if(max_speed[current_lane] == speed_limit){
            car_v += (max_speed[current_lane]  - car_v) / 10;
          }
          else{
            if(front_car_dist[current_lane] > 25){
              car_v += (max_speed[current_lane] - car_v) / 8;
            }
            else{
              car_v += (max_speed[current_lane] - 5 - car_v) / 5;
            }
          }

          // Get successor states
          vector<string> states;
          states.push_back("KL");
          if(car_state == "KL") {
            if (current_lane != 0) { states.push_back("PLCL"); }
            if (current_lane != 2) { states.push_back("PLCR"); }
          } else if (car_state == "PLCL" && current_lane != 0) {
              states.push_back("PLCL");
              states.push_back("LCL");
          } else if (car_state == "PLCR" && current_lane != 2) {
              states.push_back("PLCR");
              states.push_back("LCR");
          }
          else if (car_state == "LCL") { states.push_back("LCL");}
          else if (car_state == "LCR") { states.push_back("LCR");}

          for(int i=0; i<states.size(); ++i){
            cout << i << ": " << states[i] << "  /  ";
          }
          cout << endl;

          // Calculate cost for each lane option
          vector<double> state_cost;
          for(int i=0; i < states.size(); ++i){
            state_cost.push_back(calculate_cost(current_lane, goal_lane, states[i], speed_limit,
                                  max_speed, front_car_dist, back_car_dist));
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

          // Choose trajectory for preferred state
          int pref_lane = current_lane;
          string dir = pref_state.substr(pref_state.size() - 2);
          if(dir == "CL") {  pref_lane = current_lane-1; }
          if(dir == "CR") {  pref_lane = current_lane+1; }

          // Print lane costs and preferred lane
          for(int i=0; i<max_speed.size(); ++i){
            cout << "c: " << state_cost[i] <<"  /  ";
          }
          cout << "   ---   " << pref_lane << endl;
          cout << endl;

          // Change to preferred lane!
          goal_lane = pref_lane;

          // Set XY points bassed on some s-d points as basis for the spline
          vector<double> X, Y;
          vector<double> coord;
          X.push_back(pos_x);
          Y.push_back(pos_y);
          for (int i = 1; i < 4; ++i){
            double next_s = pos_s + i * 20;
            double next_d = (2 + 4 * goal_lane);
            coord = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            X.push_back(coord[0]);
            Y.push_back(coord[1]);
          }

          // Transform ref point to car's coordinates XY
          for (int i=0; i<X.size(); ++i){
            double shift_x = X[i] - pos_x;
            double shift_y = Y[i] - pos_y;
            X[i] = (shift_x * cos(0-angle) - shift_y*sin(0-angle));
            Y[i] = (shift_x * sin(0-angle) + shift_y*cos(0-angle));
          }
          // Create spline
          tk::spline s;
          s.set_points(X,Y);

          // 1mph = 1.609kmph = 1609/3600 mps = 0.4469 mps = 0.008938 meters in 0.02sec
          double step_dist = 0.008938 * car_v;

          for (int i = 0; i < 50; ++i) {
            if (i < path_size) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
            else {        // Set rest of the points along the spline set apart by step_dist
              double new_car_x = step_dist * (i - path_size + 1);
              double new_car_y = s(new_car_x);

              double new_x = pos_x + new_car_x * cos(angle) - new_car_y * sin(angle);
              double new_y = pos_y + new_car_x * sin(angle) + new_car_y * cos(angle);

              next_x_vals.push_back(new_x);
              next_y_vals.push_back(new_y);
            }
          }

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
