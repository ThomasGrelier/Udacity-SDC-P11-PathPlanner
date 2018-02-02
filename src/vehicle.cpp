#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"
#include "spline.h"
#include "helper_functions.h"
#include "vehicle.h"

using namespace std;

/**
 * Initializes Vehicle
 */

double lane_width = 4;        // lane width (m)

Vehicle::Vehicle(int id, float x, float y, float v, float s, float d) {

    this->id = id;
    this->x = x;
    this->y = y;
    this->v = v;
    this->s = s;
    this->d = d;
    this->lane = int(d/lane_width);
}

Vehicle::Vehicle(int id, float x, float y, float v, float s, float d, float yaw, string state,
                 vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s,
                 double end_path_d, vector<double> map_waypoints_x, vector<double> map_waypoints_y,
                 vector<double> map_waypoints_s) {
    this->id = id;
    this->x = x;
    this->y = y;
    this->v = v;
    this->s = s;
    this->d = d;
    this->lane = int(d/lane_width);
    this->v = v;
    this->yaw = yaw;
    this->state = state;
    this->previous_path_x = previous_path_x;
    this->previous_path_y = previous_path_y;
    this->end_path_s = end_path_s;
    this->end_path_d = end_path_d;
    this->prev_path_size = previous_path_x.size();
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;
}

Vehicle::~Vehicle() {}

void Vehicle::def_new_traj(vector<Vehicle> vehicles) {
    /*
    Determine the next trajectory, using a state machine and cost functions.
    INPUT: a vector of vehicles
    OUTPUT: void

    Functions that are used:
    1. successor_states() - Uses the current state to return a vector of possible successor states for the finite
       state machine.
    2. generate_trajectory(string state, vector<Vehicle> vehicles) - Returns a vector of [x,y] coordinates, which represent
       the next trajectory
    3. calculate_cost(Vehicle vehicle, map<int, vector<Vehicle>> vehicles, vector<Vehicle> trajectory) - Included from
       cost.cpp, computes the cost for a trajectory.
    */

    // Determine successor states
    vector<string> poss_successor_state = successor_states();
    // compute cost for each possible successor state
    double lower_cost = 99999;
    double cost;
    vector<vector<double>> trajectory;
    vector<vector<double>> best_trajectory;
    string best_state;
    for (unsigned int i = 0; i<poss_successor_state.size(); ++i) {
      string successor_state = poss_successor_state[i];
      cout << " - State: " << successor_state << endl;
      trajectory = generate_trajectory(successor_state, vehicles);
      cost = calculate_cost(*this, vehicles, trajectory);
      if (cost<lower_cost) {
        best_trajectory = trajectory;
        lower_cost = cost;
        best_state = successor_state;
      }
    }
    cout << "Best state: " << best_state << endl;

    for (int i = 0; i<best_trajectory.size(); ++i) {
      this->next_path_x.push_back(best_trajectory[i][0]);
      this->next_path_y.push_back(best_trajectory[i][1]);
    }
}

vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM
    There are 3 states: keep lane, lane change left, lane change right
    */
    vector<string> states;
    states.push_back("KL");
    if (this->lane>0) {
      states.push_back("LCL");
    }
    if (this->lane<2){
      states.push_back("LCR");
    }
    return states;
}

vector<vector<double>> Vehicle::generate_trajectory(string state, vector<Vehicle> vehicles) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<vector<double>> trajectory;
    if (state.compare("KL") == 0) {
      this->goal_lane = this->lane;
    }
    else if (state.compare("LCL") == 0) {
      this->goal_lane = this->lane-1;
    }
    else if (state.compare("LCR") == 0) {
      this->goal_lane = this->lane+1;
    }
    // determine the velocity in the goal lane
    this->ref_vel = get_velocity(vehicles, this->goal_lane);
    // determine reference d for trajectory computation
    this->ref_d = (double)this->goal_lane*this->lane_width+this->lane_center;
    cout << "ref_d = " << this->ref_d << ", ref_vel = " << this->ref_vel << endl;
    trajectory = gen_raw_traj();
    return trajectory;
}

double Vehicle::get_velocity(vector<Vehicle> vehicles, int lane) {
    /*
    Gets next timestep velocity for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
    double vel_max_accel = this->v + this->max_acceleration*this->delta_T;
    double vel_max_decel = this->v - this->max_acceleration*this->delta_T;
    double new_velocity;

    if (get_vehicle_ahead(vehicles, lane, this->buffer_ahead_close, this->buffer_ahead_far)) {
      // if a vehicle is ahead and slower, decrease velocity to avoid collision
      new_velocity = max(vel_max_decel, 0.);
    } else {
      // speed up, as much as possible
      new_velocity = min(vel_max_accel, this->target_speed);
    }
    return new_velocity;
}

bool Vehicle::get_vehicle_ahead(vector<Vehicle> vehicles, int lane, double buffer_ahead_close, double buffer_ahead_far) {
    // Returns true if:
    // - a slower vehicle is found ahead of the current vehicle within "buffer_ahead_far" distance,
    // - a vehicle is found ahead of the current vehicle within "buffer_ahead_close" distance.
    // False otherwise.
    // For this compare end point of previous trajectory with end point of prediction, and compare velocity

    bool found_vehicle = false;
    for (vector<Vehicle>::iterator it = vehicles.begin(); it != vehicles.end(); ++it) {
        double vehicle_end_s = it->next_path_s[n_samp-1];
        if (it->lane == lane && (vehicle_end_s > this->end_path_s) && (vehicle_end_s < (this->end_path_s+buffer_ahead_far))) {
          if ((it->v<this->v)||(vehicle_end_s < (this->end_path_s+buffer_ahead_close))){
            found_vehicle = true;
          }
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_alongside(vector<Vehicle> vehicles, int lane) {
    // Returns a true if a vehicle is found behind the current vehicle, within a predifined distance. False otherwise.
    // For this compare end point of previous trajectory with end point of prediction, and compare velocity

    bool found_vehicle = false;
    for (vector<Vehicle>::iterator it = vehicles.begin(); it != vehicles.end(); ++it) {
        double vehicle_end_s = it->next_path_s[n_samp-1];
        if (it->lane == lane && (vehicle_end_s < (this->end_path_s+this->margin_ahead)) && (vehicle_end_s > (this->end_path_s-this->margin_behind))) {
          found_vehicle = true;
        }
    }
    return found_vehicle;
}

void Vehicle::generate_predictions(vector<Vehicle> &vehicles) {
    //Generates next trajectory (s,d) for non-ego vehicles
    float next_s;
    float next_d;
    vector<double> pos_xy;
    for(int i = 0; i < vehicles.size(); ++i) {
      for (int j = 0; j < this->n_samp; ++j) {
        next_s = vehicles[i].s+vehicles[i].v*(j+1)*this->delta_T;
        next_d = vehicles[i].d;
        pos_xy = getXY(next_s, next_d, this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
        vehicles[i].next_path_s.push_back(next_s);
        vehicles[i].next_path_d.push_back(next_d);
        vehicles[i].next_path_x.push_back(pos_xy[0]);
        vehicles[i].next_path_y.push_back(pos_xy[1]);
      }
  	}
}

vector<vector<double>> Vehicle::gen_raw_traj() {

  vector<double> next_x_vals;
  vector<double> next_y_vals;
  vector<double> ptsx;
 	vector<double> ptsy;
 	double pos_s;
  double pos_x;
  double pos_y;
  double angle;

  // reference pose for the new trajectory
  double ref_x = this->x;
  double ref_y = this->y;
  double ref_yaw = this->yaw;
  double prev_ref_x;
  double prev_ref_y;

  if(this->prev_path_size >= 2) {
    // Keep the first "n_prev_keep" points of previous trajectory
    // Select the two last points of them
    ref_x = this->previous_path_x[this->n_prev_keep-1];
    ref_y = this->previous_path_y[this->n_prev_keep-1];
    prev_ref_x = this->previous_path_x[this->n_prev_keep-2];
    prev_ref_y = this->previous_path_y[this->n_prev_keep-2];
    // Compute yaw
    ref_yaw = atan2(ref_y-prev_ref_y,ref_x-prev_ref_x);

  }
  else
  {
    // Take current position and propagate backwards to create a second point
    prev_ref_x = ref_x - cos(ref_yaw);
    prev_ref_y = ref_y - sin(ref_yaw);
    this->n_prev_keep = 0;
    this->end_path_s = this->s; // for the first iteration, end_path_s is not valid.
  }
  ptsx.push_back(prev_ref_x);
  ptsx.push_back(ref_x);
  ptsy.push_back(prev_ref_y);
  ptsy.push_back(ref_y);

  // define the ref trajectory as a set of 4 points further ahead of previous trajectory's end point
  // in the middle of the lane. Reference d is the center of the lane.
  //cout << "prev_ref_pos: " << prev_ref_x << "," << prev_ref_y << endl;
  //cout << "ref_pos: " << ref_x << "," << ref_y << endl;
  //cout << "end_s: " << this->end_path_s << endl;
  for(int i = 0; i < 3; ++i) {
    pos_s = this->end_path_s +(i+1)*this->dist_inc;
    vector<double> pos_xy = getXY(pos_s, this->ref_d, this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
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
  for (int i = 0; i<this->n_prev_keep; ++i) {
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
  double N =(target_distance)/(this->delta_T*this->ref_vel);
  double x_add_on = 0;
  double x_point;
  double y_point;

  for (int i = 1; i<=(this->n_samp-this->n_prev_keep); ++i) {
    x_point = x_add_on+target_x/N;
    y_point = s(x_point);
    x_add_on = x_point;

    double x_temp = x_point;
    double y_temp = y_point;

    // convert from local to global frame
    x_point = x_temp*cos(ref_yaw)-y_temp*sin(ref_yaw) + ref_x;
    y_point = x_temp*sin(ref_yaw)+y_temp*cos(ref_yaw) + ref_y;

    // store
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  // output trajectory
  vector<vector<double>> trajectory;
  for (int i =0; i<this->n_samp; ++i) {
    vector<double> point;
    point.push_back(next_x_vals[i]);
    point.push_back(next_y_vals[i]);
    trajectory.push_back(point);
    //cout << "Point: " << next_x_vals[i] << "," << next_y_vals[i] << endl;
  }
  return trajectory;
}
