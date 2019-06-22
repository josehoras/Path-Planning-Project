#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

int get_lane(int current_lane, string state){
  int intended_lane = current_lane;
  string dir = state.substr(state.size() - 2);
  if(dir == "CL" && current_lane != 0) { intended_lane = current_lane-1; }
  if(dir == "CR" && current_lane != 2) { intended_lane = current_lane+1; }
  return intended_lane;
}

double calculate_cost (string next_state, int current_lane, bool changing_lanes, double speed_limit,
                        vector<double> max_speed, vector<double> front_car_dist,
                        vector<double> back_car_dist)
{
  int next_lane = get_lane(current_lane, next_state);
  // Efficiency cost based on lane max speed
  double eff_cost = (speed_limit - max_speed[next_lane]) / speed_limit;
  // Penalization for a lane with car on it, even if they still don't limit our speed
  if(front_car_dist[next_lane] < 100)
    eff_cost += 0.04;
  // Bonus to change to middle lane if the next one is empty
  if (next_state!="KL" && next_lane==1 && max_speed[1] < (speed_limit -5)){
    if(  (next_state=="LCL" && max_speed[0] == speed_limit && back_car_dist[0] > 5)
      || (next_state=="LCR" && max_speed[2] == speed_limit && back_car_dist[2] > 5))
      eff_cost -= 0.15;
  }
  // Security cost to not get near other cars when changing lanes
  double sec_cost = 0;
  int sec_dist_front = 5;
  int sec_dist_back = 5;
  if(front_car_dist[next_lane] < sec_dist_front){
    sec_cost += (sec_dist_front - front_car_dist[next_lane]) / (sec_dist_front*2);
  }
  if(back_car_dist[next_lane] < sec_dist_back && next_lane != current_lane){
    sec_cost += (sec_dist_back - back_car_dist[next_lane]) / (sec_dist_back*2);
  }
  // Small cost to discourage the car to change lanes with no reason
  double lazy_cost = 0;
  if(next_lane != current_lane) { lazy_cost = 0.05; }
  // Cost to discourage the car to abort a change lane action while still changing lanes
  double keep_action_cost = 0;
  if(next_state=="KL" && changing_lanes)
    keep_action_cost = 0.9;

  return eff_cost + 10 * sec_cost + lazy_cost + keep_action_cost;
}

vector<double> set_goals(string next_state, int current_lane, vector<double> max_speed)
{
  double goal_lane;

  if(next_state == "KL")
    goal_lane = current_lane;
  if(next_state == "LCL")
    goal_lane = current_lane - 1;
  if(next_state == "LCR")
    goal_lane = current_lane + 1;

  double goal_vel = max_speed[goal_lane];

  return {goal_lane, goal_vel};
}

#endif  // HELPERS_H
