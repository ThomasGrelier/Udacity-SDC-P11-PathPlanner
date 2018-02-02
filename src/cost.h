#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

double collision_cost(Vehicle & vehicle, vector<Vehicle> vehicles, vector<vector<double>> trajectory);
double inefficiency_cost(Vehicle & vehicle, vector<Vehicle> vehicles, vector<vector<double>> trajectory);
double buffer_cost(Vehicle & vehicle, vector<Vehicle> vehicles, vector<vector<double>> trajectory);
double calculate_cost(Vehicle & vehicle, vector<Vehicle> vehicles, vector<vector<double>> trajectory);

#endif
