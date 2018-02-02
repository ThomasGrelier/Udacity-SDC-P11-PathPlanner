#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:
  int id;
  int lane;
  double s;
  double d;
  double x;
  double y;
  double v;
  int lanes_available;
  int goal_lane;
  int goal_s;
  string state;
  double ref_vel;               // reference speed for path planning (m/s)
  double ref_d;                 // reference d for path planning (m/s)
  double yaw = yaw;
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  double end_path_s;
  double end_path_d;
  int prev_path_size;
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> next_path_s;
  vector<double> next_path_d;
  vector<double> next_path_x;
  vector<double> next_path_y;
  double target_speed = 49.8*0.44704;  // goal speed (m/s)
  double dist_inc = 30;         // step of distance between points for the reference trajectory (m)
  int n_samp = 50;              // nb of samples of trajectory
  int n_prev_keep = 10;          // nb of points of previous list (beginning) to keep for the new list
  double lane_width = 4;        // lane width (m)
  double lane_center = 2;       // position of lane center (m)
  double delta_T = 0.02;        // time increment (s)
  double max_acceleration = 10; // max acceleration (m/s2)
  double buffer_ahead_far = 40;     // distance range for collision checking with vehicle ahead (m)
  double buffer_ahead_close = 10;     // buffer distance to keep with vehicle ahead (m)
  double margin_ahead = 3;      // distance for collision checking on lane change, vehicle ahead (m)
  double margin_behind = 5;     // distance for collision checking on lane change, vehicle behind (m)

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(int id, float x, float y, float v, float s, float d);
  Vehicle(int id, float x, float y, float v, float s, float d, float yaw, string state,
                 vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s,
                 double end_path_d, vector<double> map_waypoints_x, vector<double> map_waypoints_y,
                 vector<double> map_waypoints_s);
  /**
  * Destructor
  */
  virtual ~Vehicle();

  void def_new_traj(vector<Vehicle> vehicles);
  vector<string> successor_states();
  vector<vector<double>> generate_trajectory(string state, vector<Vehicle> vehicles);
  double get_velocity(vector<Vehicle> vehicles, int lane);
  bool get_vehicle_ahead(vector<Vehicle> vehicles, int lane, double buffer_ahead_close, double buffer_ahead_far);
  bool get_vehicle_alongside(vector<Vehicle> vehicles, int lane);
  void generate_predictions(vector<Vehicle> &vehicles);
  vector<vector<double>> gen_raw_traj();
};

#endif
