#include <functional>
#include <math.h>
#include "cost.h"
#include "vehicle.h"
#include "helper_functions.h"

//Weights for cost functions.
const double COLLISION = 2.;
const double EFFICIENCY = 1.;
const double BUFFER  = 1.;

const double VEHICLE_RADIUS = 1.; // model vehicle as circle to simplify collision detection

double collision_cost(Vehicle & vehicle, vector<Vehicle> vehicles, vector<vector<double>> trajectory) {
    /*
    Binary cost function which penalizes collisions
    */
    double nearest = nearest_approach_to_any_vehicle(vehicles, trajectory);
    if (nearest < 3*VEHICLE_RADIUS) {
      cout << "!! COLLISION !!" << endl;
      return 1.0;
    }
    else { return 0.0; }
}

double buffer_cost(Vehicle & vehicle, vector<Vehicle> vehicles, vector<vector<double>> trajectory) {
    /*
    Penalizes lane change without margin
    */
    double cost = 0;
    if (vehicle.goal_lane != vehicle.lane) {
      if (vehicle.get_vehicle_alongside(vehicles, vehicle.goal_lane)) {
        cout << "!! NO MARGIN !!" << endl;
        cost = 1.0;
      }
    }
    return cost;
}

double inefficiency_cost(Vehicle & vehicle, vector<Vehicle> vehicles, vector<vector<double>> trajectory) {
    /*
    Rewards high average speeds.
    */
    double cost;
    if (vehicle.ref_vel>0) {
      cost = 2.0 / (1 + exp(-1/vehicle.ref_vel)) - 1.0;
    } else { cost = 1; };
    return cost;
}

double calculate_cost(Vehicle & vehicle, vector<Vehicle> vehicles, vector<vector<double>> trajectory) {
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    double cost = 0.0;

    //Add additional cost functions here.
    vector< function<double(Vehicle & , vector<Vehicle> , vector<vector<double>> )> > cf_list;
    cf_list.push_back(collision_cost);
    cf_list.push_back(inefficiency_cost);
    cf_list.push_back(buffer_cost);
    vector<double> weight_list;
    weight_list.push_back(COLLISION);
    weight_list.push_back(EFFICIENCY);
    weight_list.push_back(BUFFER);
    double new_cost;
    for (int i = 0; i < cf_list.size(); i++) {
        new_cost = weight_list[i]*cf_list[i](vehicle, vehicles, trajectory);
        cost += new_cost;
    }
    cout << "Cost = " << cost << endl;
    return cost;
}

