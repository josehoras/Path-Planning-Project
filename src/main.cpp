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
          double pos_x;
          double pos_y;
          double angle;
          double pos_s;
          double pos_d;
          double car_v = car_speed;
          int path_size = previous_path_x.size();
          // cout << car_d << endl;
          for (int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          if (path_size == 0) {
            pos_x = car_x;
            pos_y = car_y;
            angle = deg2rad(car_yaw);
            pos_s = car_s;
            pos_d = car_d;

          } else {
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];
            double pos_x2 = previous_path_x[path_size-2];
            double pos_y2 = previous_path_y[path_size-2];
            angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
            // car_v = sqrt(pow(pos_x2-pos_x, 2) + pow(pos_y2-pos_y, 2)) / 0.02;
            vector<double> pos_sd = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
            pos_s = pos_sd[0];
            pos_d = pos_sd[1];
          }
          // 1mph = 1.609kmph = 1609/3600 mps = 0.4469 mps = 0.008938 meters in 0.02sec
          double lane = 1;
          double speed_limit = 49.5;
          if (car_v < speed_limit/2) {
            car_v += 4;
          }
          else if (car_v < speed_limit-2) {
            car_v += 1.5;
          }
          else{
            car_v = speed_limit;
          }
          //cout << car_v << endl;
          double step_dist = 0.008938 * car_v;

          // Set XY points bassed on some s-d points as basis for the spline
          vector<double> X, Y;
          vector<double> coord;
          X.push_back(pos_x);
          Y.push_back(pos_y);
          for (int i = 1; i < 4; ++i){
            double next_s = pos_s + i * 20;
            double next_d = (2 + 4 * lane);
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
          tk::spline s;
          s.set_points(X,Y);    // currently it is required that X is already sorted

          for (int i = 0; i < 40-path_size; ++i) {

            double new_car_x = step_dist * (i + 1);
            double new_car_y = s(x_point);

            double x_t = x_point;
            double y_t = y_point;

            double new_x = pos_x + new_car_x * cos(angle) - new_car_y * sin(angle);
            double new_y = pos_y + new_car_x * sin(angle) + new_car_y * cos(angle);

//            x_point += pos_x;
//            y_point += pos_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
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
