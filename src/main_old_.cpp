#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helper_functions.h"

using namespace std;

// for convenience
using json = nlohmann::json;


int main(int argc, char* argv[]) {
  uWS::Hub h;

  // open a file
  ofstream outfile;
  // read argument
  if (argc >= 2){
    outfile.open(argv[1],ofstream::out);
    outfile << "Results" << endl;
  }

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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&outfile](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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
          	double car_yaw = deg2rad(j[1]["yaw"]);    // convert to RAD !!
          	double car_speed = j[1]["speed"];  // convert to m/s
            car_speed *= 0.44704;
          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

            // PARAMETERS
            int lane;                     // 0 = left, 1 = middle, 2 = right
            double max_vel = 49.5*0.44704;  // optimal speed (m/s)
            double ref_vel;               // reference speed for path planning (m/s)
            double dist_inc = 20;         // step of distance between points for the reference trajectory (m)
            int n_samp = 50;              // nb of samples of trajectory
            int n_prev_keep = 5;          // nb of points of previous list (beginning) to keep for the new list
            double lane_width = 4;        // lane width (m)
            double lane_center = lane_width/2;   // position of lane center (m)
            double delta_T = 0.02;        // time increment (s)
            double a_max = 30;            // max acceleration (m/s2)
            double inc_vel = a_max*delta_T; // maximum velocity change between two points (m/s)
            double collision_dist = 30;   // distance for collision checking (m)

            // Local variables
            double pos_s;
            double pos_d;
            double ref_d;
            double pos_x;
            double pos_y;
            double angle;
            vector<double> next_x_vals;
          	vector<double> next_y_vals;
            vector<double> ptsx;
          	vector<double> ptsy;

            // Determine lane
            lane = int(car_d/lane_width);
            cout << "lane = " << lane << endl;
            // reference d will be the center of the lane
          	ref_d = lane*lane_width+lane_center;
            int prev_path_size = previous_path_x.size(); // = 0 for the first iteration

            // for initial point, previous list last point is set to current position
            if(prev_path_size==0) {
              end_path_s = car_s;
            }

            // DETERMINE REFERENCE VELOCITY
            bool too_close = false;
            cout << "car speed: " << car_speed <<endl;

            ref_vel = max_vel;    // by default ref velocity is equal to optimal velocity

            // Check if car are present in in the lane and store their velocity
            for (int i=0; i< sensor_fusion.size(); ++i) {
              float d = sensor_fusion[i][6];
              if (d<(ref_d+2) && d>(ref_d-2))
              {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx+vy*vy);
                double check_car_s = sensor_fusion[i][5];

                // propagate car's position to last timetag of previous trajectory and check if it lies within
                // collision distance of ego car
                check_car_s += (double)prev_path_size*check_speed*delta_T;
                if ((check_car_s > end_path_s) && (check_car_s<(end_path_s+collision_dist)) && check_speed<car_speed) {
                  cout << "car: " << check_car_s << "," << check_speed << endl;
                  ref_vel =  check_speed;
                  too_close = true;
                }
              }
            }

            cout << "ref_speed = " << ref_vel << endl;
            // increase/decrease velocity, with compliance to max acceleration
            inc_vel = min(abs(ref_vel - car_speed), inc_vel);
            if (ref_vel > car_speed) {
              ref_vel = car_speed + inc_vel;
            }
            else {
              ref_vel = car_speed - inc_vel;
            }
            ref_vel = min(ref_vel, max_vel);
            cout << "new_ref_speed = " << ref_vel << endl;

            //// DEFINE NEW TRAJECTORY
            // reference pose for the new trajectory
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = car_yaw;

            if(prev_path_size >= 2)
            {
              // Keep the first "n_prev_keep" points of previous trajectory
              // Select the two last points of them
              ref_x = previous_path_x[n_prev_keep-1];
              ref_y = previous_path_y[n_prev_keep-1];
              double prev_ref_x = previous_path_x[n_prev_keep-2];
              double prev_ref_y = previous_path_y[n_prev_keep-2];
              // Compute yaw
              ref_yaw = atan2(ref_y-prev_ref_y,ref_x-prev_ref_x);

              ptsx.push_back(prev_ref_x);
              ptsx.push_back(ref_x);
              ptsy.push_back(prev_ref_y);
              ptsy.push_back(ref_y);

              }
            else
            {
              // Take current position and propagate backwards to create a second point
              double prev_car_x = ref_x - cos(car_yaw);
              double prev_car_y = ref_y - sin(car_yaw);
              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
              n_prev_keep = 0;
            }
            cout << "---" << endl;
            /*
            for (int i = 0; i<ptsx.size();++i) {
              cout << "prev_pos: " << ptsx[i] << "," << ptsy[i] << endl;
            }
            */

            // define the ref trajectory as a set of 4 points further ahead of previous trajectory's end point
            // in the middle of the lane. Reference d is the center of the lane.
            //cout << "ref_pos: " << ref_x << "," << ref_y << endl;
            for(int i = 0; i < 4; ++i)
            {
              pos_s = end_path_s +(i+1)*dist_inc;
              vector<double> pos_xy = getXY(pos_s, ref_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              ptsx.push_back(pos_xy[0]);
              ptsy.push_back(pos_xy[1]);
              //cout << "ref point: " << pos_xy[0] << "," << pos_xy[1] << endl;
            }

            // convert to local frame, centered on ref pose (ref_x, ref_y, ref_yaw)
            for (int i = 0; i<ptsx.size();++i) {
              //cout << ptsx[i] << "," << ptsy[i];
              double shift_x = ptsx[i]-ref_x;
              double shift_y = ptsy[i]-ref_y;
              ptsx[i] = shift_x*cos(ref_yaw)+shift_y*sin(ref_yaw);
              ptsy[i] = -shift_x*sin(ref_yaw)+shift_y*cos(ref_yaw);
              //cout << "->" << ptsx[i] << "," << ptsy[i] << endl;
            }

            // create a spline
            tk::spline s;
            s.set_points(ptsx,ptsy);

            // add previous path points to next path
            for (int i = 0; i<n_prev_keep; ++i) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // fix a target some distance away
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_distance = distance(target_x, target_y, 0, 0);

            double pos_x_map;
            double pos_y_map;

            // Determine number of points within target distance
            // -> d = v*(N*dT)
            double N =(target_distance)/(delta_T*ref_vel);
            double x_add_on = 0;
            double x_point;
            double y_point;

            for (int i = 1; i<=(n_samp-n_prev_keep); ++i) {
              x_point = x_add_on+target_x/N;
              y_point = s(x_point);
              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // convert from local to global frame
              x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw) + ref_x;
              y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw) + ref_y;

              //cout << "add point: " << x_point << "," << y_point << endl;
              // store
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

            // Output results
            if (outfile.is_open()){
              outfile << endl << "## Iteration ##" << endl;
              outfile << "Current pose : (" << car_x << ", " << car_y << ", " << car_yaw << ") " << endl;
              outfile << "# Previous path" << endl;
              for(int i = 0; i < prev_path_size; i++)
              {
                  outfile << previous_path_x[i] << ", " << previous_path_y[i] << endl;
              }
              outfile << "# Next path" << endl;
              for(int i = 0; i < next_x_vals.size(); i++) {
                outfile << next_x_vals[i] << ", " << next_y_vals[i] << endl;
              }
            }

            // Send results to simulator
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
  if (outfile.is_open()){
    outfile.close();
  }
}
