#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "../spline/src/spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

enum {
  AHEAD = 0,
  LEFT = 1,
  RIGHT = 2
};

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_(__FILE__);
  map_file_ = map_file_.substr(0, map_file_.find_last_of("\\/"));
  map_file_ += "/../data/highway_map.csv";
  std::cout << map_file_ << std::endl;

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

  // Ego vehicle will stay on starting lane when possible by default.
  const unsigned int DEFAULT_CAR_LANE = 1;

  // Current ego vehicle lane.
  unsigned int car_lane = DEFAULT_CAR_LANE;

  // Current ego vehicle target velocity. Ego vehicle is starting at 0 km/s.
  double ref_vel = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &car_lane, &ref_vel]
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

          /**
           * 0. Set up constant values and initiate variables.
           */

          // 80 km/h = ~49.7 mp/h - we use metric system in Europe
          const double MAX_SPEED = 80;
          // 5 m/s for frequency 50Hz.
          const double MAX_ACCELERATION = ms_to_kmh_50hz(5);
          // 30 m distance.
          const double DISTANCE = 30;

          unsigned int prev_size = previous_path_x.size();

          // Prevent collisions.
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          /**
           * 1. Prediction.
           */

          // Check current and future positions of detected nearby vehicles.
          vector<bool> vehicle = {false, false, false};
          // Loop through all the detected vehicles
          for (int i = 0; i < sensor_fusion.size(); i++) {
            // Get vehicle current lane.
            unsigned int vehicle_lane = (unsigned int)sensor_fusion[i][6] / 4;
            // Get vehicle current speed.
            double vehicle_speed = distance(0, 0, sensor_fusion[i][3],
                                          sensor_fusion[i][4]);
            //double check_car_s = sensor_fusion[i][5];
            double vehicle_s = sensor_fusion[i][5];

            // If vehicle is on the same lane as ego vehicle
            if (vehicle_lane == car_lane) {
              vehicle[AHEAD] = vehicle[AHEAD] || (vehicle_s > car_s &&
                  vehicle_s - car_s < DISTANCE);
            // If vehicle is on the left lane from the ego vehicle.
            } else if (vehicle_lane - car_lane == -1) {
              // Mark the vehicle current position.
              vehicle[LEFT] = vehicle[LEFT] || (car_s - DISTANCE < vehicle_s &&
                  car_s + DISTANCE > vehicle_s);
              // Mark the vehicle future position.
              vehicle[LEFT] = vehicle[LEFT] || (car_s +
                  prev_size*ms_50hz(car_speed) - DISTANCE < vehicle_s +
                  prev_size*ms_50hz(vehicle_speed) && car_s +
                  prev_size*ms_50hz(car_speed) + DISTANCE > vehicle_s +
                  prev_size*ms_50hz(vehicle_speed));
            // If vehicle is on the right lane from the ego vehicle.
            } else if (vehicle_lane - car_lane == 1) {
              // Mark the vehicle current position.
              vehicle[RIGHT] = vehicle[RIGHT] || (car_s - DISTANCE <
                  vehicle_s && car_s + DISTANCE > vehicle_s);
              // Mark the vehicle future position.
              vehicle[RIGHT] = vehicle[RIGHT] || (car_s +
                  prev_size*ms_50hz(car_speed) - DISTANCE < vehicle_s +
                  prev_size*ms_50hz(vehicle_speed) && car_s +
                  prev_size*ms_50hz(car_speed) + DISTANCE > vehicle_s +
                  prev_size*ms_50hz(vehicle_speed));
            }
          }

          /**
           * 2. Behavior Planning.
           */

          double speed_change = 0;

          // If there is vehicle ahead ego vehicle.
          if (vehicle[AHEAD]) {
            // If there is no vehicle on the left lane and there is a left lane.
            if (!vehicle[LEFT] && car_lane > 0) {
              // Change ego vehicle lane to left line.
              car_lane--;
            // If there is no vehicle on the right lane and there is a right
            // lane.
            } else if (!vehicle[RIGHT] && car_lane < 2) {
              // Change ego vehicle lane to right line.
              car_lane++;
            // If there is no space at neighborhood lane.
            } else {
              // Decrease speed.
              speed_change -= MAX_ACCELERATION;
            }
          // If there is no vehicle ahead ego vehicle.
          } else {
            // If ego vehicle is on a side lane.
            if ((car_lane == 0 && !vehicle[RIGHT]) ||
                (car_lane == 2 && !vehicle[LEFT])) {
              // Use default lane.
              car_lane = DEFAULT_CAR_LANE;
            }
            // If ego vehicle speed is lower then maximum speed.
            if (ref_vel < MAX_SPEED) {
              // Increase speed.
              speed_change += MAX_ACCELERATION;
            }
          }

          /**
           * 3. Trajectory generation.
           */

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if (prev_size < 2) {
            // There are no previous points. Use the car current position.
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            // Use previous two points.
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // Setting up 3 target points in the future as minimally required
          // by spline.
          for (unsigned int i = 1; i <= 3; i ++) {
            vector<double> next_pos_e = getXY(car_s + DISTANCE*i,
                                              2 + 4*car_lane, map_waypoints_s,
                                              map_waypoints_x, map_waypoints_y);
            ptsx.push_back(next_pos_e[0]);
            ptsy.push_back(next_pos_e[1]);
          }


          // Change points coordinates to ego vehicle coordinates system.
          for (int i = 0; i < ptsx.size(); i++) {

            vector<double> pos_e =
                move_and_rotate_point_relative_to_orgin({ref_x, ref_y}, ref_yaw,
                                                        {ptsx[i], ptsy[i]});
            ptsx[i] = pos_e[0];
            ptsy[i] = pos_e[1];
          }

          // Create spline.
          tk::spline s;
          s.set_points(ptsx, ptsy);

          // Add path points from previous path.
          for (int i = 0; i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate distance y position 30 m ahead.
          double target_x = DISTANCE;
          double target_y = s(target_x);
          double target_dist = distance(0, 0, target_x, target_y);

          double add_on_x = 0;

          for(int i = 1; i < 50 - prev_size; i++) {
            ref_vel += speed_change;
            if (ref_vel > MAX_SPEED) {
              ref_vel = MAX_SPEED;
            } else if (ref_vel < MAX_ACCELERATION) {
              ref_vel = MAX_ACCELERATION;
            }
            vector<double> pos_e(2);
            pos_e[0] = add_on_x +
                target_x/(target_dist/(kmh_to_ms_50hz(ref_vel)));
            pos_e[1] = s(pos_e[0]);
            add_on_x = pos_e[0];
            // Change point coordinates to word coordinate system.
            pos_e = rotate_and_move_back_point_relative_to_orgin({ref_x, ref_y},
                                                                ref_yaw, pos_e);
            next_x_vals.push_back(pos_e[0]);
            next_y_vals.push_back(pos_e[1]);
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
