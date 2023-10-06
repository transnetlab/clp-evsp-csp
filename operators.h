#ifndef EBUS_VNS_OPERATORS_H
#define EBUS_VNS_OPERATORS_H

#include "logger.h"
#include "constants.h"
#include "vehicle.h"
#include <random>
#include <sstream>

class Exchange {
public:
    int first_vehicle_index;
    int first_trip_index;
    int second_vehicle_index;
    int second_trip_index;
};

class Shift {
public:
    int source_vehicle_index;
    int source_trip_index;
    int dest_vehicle_index;
    int dest_trip_index; // The new trip is inserted after this index
};

namespace scheduling {
double exchange_trips(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Exchange&);
double shift_trips(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Shift&);
void perform_exchange(std::vector<Vehicle>&, Exchange&, Logger&);
void perform_shift(std::vector<Vehicle>&, Shift&, Logger&);
void optimize_rotations(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Logger& logger);
}

namespace operators {
void optimize_locations(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Logger&);
void close_charging_stations(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, int, Logger&);
void open_charging_stations(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, int, Logger&);
void split_trip(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, int, int, std::vector<int>&,
        Logger&);
}

namespace evaluation {
double calculate_objective(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Logger&);
bool is_exchange_compatible(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
bool is_shift_compatible(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
bool are_rotations_charge_feasible(std::vector<Trip>&, std::vector<Terminal>&, std::vector<std::vector<int>>);
double calculate_trip_replacement_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
double calculate_trip_addition_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
double calculate_trip_removal_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int);
void calculate_utilization(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Logger&);
void check_rotation_feasibility(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, std::vector<int>&,
        Logger&);
bool is_charge_adequate_next_trip(std::vector<Trip>& trip, int curr_trip, int next_trip, bool is_curr_trip_end_charge_terminal, bool is_next_trip_start_charge_terminal, int charge_time_window, double& charge_level);
}

#endif //EBUS_VNS_OPERATORS_H
