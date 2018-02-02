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

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
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

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

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

             // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            /*
          	// shaping data
          	vector<Vehicle>
          	// 1) determine target using behavior planner
          	string next_state = choose_next_state(sensor_fusion)
          	best = PTG(start_s, start_d, target, delta, T, predictions)
            */
            // Parameters
            int lane;  // 0 = left, 2 = right
            double max_vel = 49.5*0.44704;  // optimal speed (m/s)
            double ref_vel;           // reference speed for path planning (m/s)
            double dist_inc = 20;     // step of distance between points for the reference trajectory (m)
            int n_samp = 50;          // nb of samples of ref trajectory
            double lane_width = 4;    // lane width (m)
            double lane_center = lane_width/2;   // position of lane center (m)
            double delta_T = 0.02;  // time increment (s)
            double a_max = 30;      // max acceleration (m/s2)
            double inc_vel = a_max*delta_T; // maximum velocity change between two points (m/s)
            double collision_dist = 30;   // distance for collision checking (m)

            // local variables
            double pos_s;
            double pos_d;
            double ref_s;
            double ref_d;
            double pos_x;
            double pos_y;
            double angle;
            vector<double> next_x_vals;
          	vector<double> next_y_vals;
            vector<double> ptsx;
          	vector<double> ptsy;

            // determine lane
            lane = int(car_d/lane_width);
            cout << "lane = " << lane << endl;
            // reference d will be the center of the lane
          	ref_d = lane*lane_width+lane_center;
            int prev_path_size = previous_path_x.size(); // = 0 for the first iteration

            // for initial point, previous list last point is set to current position
            if(prev_path_size==0) {
              end_path_s = car_s;
            }

            bool too_close = false;
            cout << "car speed: " << car_speed <<endl;
            // find ref_v to use
            ref_vel = max_vel;
            for (int i=0; i< sensor_fusion.size(); ++i) {
              // check if car in in the lane
              float d = sensor_fusion[i][6];
              if (d<(ref_d+2) && d>(ref_d-2))
              {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx+vy*vy);
                double check_car_s = sensor_fusion[i][5];

                // propagate car's position to last timetag of previous trajectory and check if it lies within
                // collision distance
                check_car_s += (double)prev_path_size*check_speed*delta_T;
                if ((check_car_s > end_path_s) && (check_car_s<(end_path_s+collision_dist)) && check_speed<car_speed) {
                  cout << "car: " << check_car_s << "," << check_speed << endl;
                  ref_vel =  check_speed;
                  too_close = true;
                }
              }
            }
            // determine new reference velocity
            cout << "ref_speed = " << ref_vel << endl;
            // limit velocity change
            inc_vel = min(abs(ref_vel - car_speed), inc_vel);
            if (ref_vel > car_speed) {
              ref_vel = car_speed + inc_vel;
            }
            else {
              ref_vel = car_speed - inc_vel;
            }
            ref_vel = min(ref_vel, max_vel);
            cout << "new_ref_speed = " << ref_vel << endl;

            // limit acceleration


            // reference pose for the new trajectory
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = car_yaw;

            if(prev_path_size >= 2)
            {
              // Set reference pose as the last point of the previous trajectory
              // Add the one before the last
              ref_x = previous_path_x[prev_path_size-1];
              ref_y = previous_path_y[prev_path_size-1];
              double prev_ref_x = previous_path_x[prev_path_size-2];
              double prev_ref_y = previous_path_y[prev_path_size-2];
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
            }
            cout << "---" << endl;
            /*
            for (int i = 0; i<ptsx.size();++i) {
              cout << "prev_pos: " << ptsx[i] << "," << ptsy[i] << endl;
            }
            */

            // define the ref trajectory as a set a 4 points further ahead of previous trajectory's end point
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

            // add previous path to next path
            for (int i = 0; i<prev_path_size; ++i) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // fix a target some distance away
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_distance = distance(target_x, target_y, 0, 0);

            double pos_x_map;
            double pos_y_map;

            // number of points to add to trajectory to comply with ref_vel
            double N =(target_distance)/(delta_T*ref_vel);
            double x_add_on = 0;
            double x_point;
            double y_point;

            for (int i = 1; i<=(n_samp-prev_path_size); ++i) {
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
