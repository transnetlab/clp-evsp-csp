#ifndef EBUS_VNS_OPERATORS_H
#define EBUS_VNS_OPERATORS_H

#include "logger.h"
#include "constants.h"
#include "vehicle.h"
#include "csp.h"
#include <omp.h>
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

class ThreeExchange {
public:
    int first_vehicle_index;
    int first_trip_index;
    int second_vehicle_index;
    int second_trip_index;
    int third_vehicle_index;
    int third_trip_index;
};

class TwoShift {
public:
    int source_vehicle_index;
    int first_source_trip_index;
    int second_source_trip_index;
    int first_dest_vehicle_index;
    int first_dest_trip_index; // The new trip is inserted after this index
    int second_dest_vehicle_index;
    int second_dest_trip_index; // The new trip is inserted after this index
};

namespace scheduling {
double exchange_trips(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Exchange&, Logger&);
double exchange_depots(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Exchange&, Logger&);
double shift_trips(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Shift&, Logger&);

void perform_exchange(std::vector<Vehicle>&, Exchange&, Logger&);
void perform_shift(std::vector<Vehicle>&, Shift&, Logger&);

void apply_best_improvement(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Logger&);
void optimize_rotations(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Logger&);
}

namespace locations {
void split_trips(std::vector<Vehicle>&, std::vector<int>&, int, int, Logger&);
bool are_rotations_charge_feasible(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, std::vector<int>&, Logger&);
void open_charging_station(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, int, Logger&);
void close_charging_station(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, int, Logger&);
double swap_charging_station(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, int, int, Logger&);
void optimize_stations(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Logger&);
}

namespace diversification {
double exchange_three_trips(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ThreeExchange&);
void shift_two_trips(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Logger&);
double shift_all_trips(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip, std::vector<Terminal>& terminal, int source_vehicle_index, Logger& logger);
void perform_three_exchange(std::vector<Vehicle>& vehicle, ThreeExchange& three_exchange, Logger& logger);
void perform_two_shift(std::vector<Vehicle>& vehicle, TwoShift& two_shift, Logger& logger);

void apply_operators(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Logger&);
void optimize_rotations(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Logger&);
}

namespace evaluation {
double calculate_objective(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Logger&);
double calculate_deadheading_cost(std::vector<Vehicle>&, std::vector<Trip>&);
double calculate_deadheading_cost(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<int>&);
void calculate_utilization(std::vector<Vehicle>& vehicle, std::vector<Trip>&, std::vector<Terminal>&, Logger&);

bool is_two_exchange_compatible(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip, int first_vehicle_index, int second_vehicle_index, int first_vehicle_trip_index, int second_vehicle_trip_index);
bool is_three_exchange_compatible(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip, int first_vehicle_index, int second_vehicle_index, int third_vehicle_index, int first_vehicle_trip_index, int second_vehicle_trip_index, int third_vehicle_trip_index);
bool is_shift_compatible(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);

bool is_charge_adequate_next_trip(std::vector<Trip>&, int, int, bool, bool, int, double&);
bool are_rotations_charge_feasible(std::vector<Trip>&, std::vector<Terminal>&, std::vector<std::vector<int>>);

double calculate_trip_replacement_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
double calculate_depot_replacement_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
double calculate_trip_addition_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
double calculate_trip_removal_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int);
}

#endif //EBUS_VNS_OPERATORS_H
