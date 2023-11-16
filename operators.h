#ifndef EBUS_VNS_OPERATORS_H
#define EBUS_VNS_OPERATORS_H

#include "logger.h"
#include "constants.h"
#include "vehicle.h"
#include "csp.h"
#include <omp.h>
#include <random>
#include <sstream>
#include <climits>
#include <utility>
#include <map>

bool extern SOLVE_CSP_JOINTLY;
bool extern PERFORM_THREE_EXCHANGES;  // Flag to turn on/off 3-exchange operators
bool extern SHIFT_ALL_TRIPS;  // Flag to turn on/off 2-shift operators
int extern NUM_THREADS;  // Number of threads to use for parallelization

class Exchange {
public:
    int first_vehicle_index;
    int first_trip_index;
    int second_vehicle_index;
    int second_trip_index;

    // Create a constructor that initializes these variables with maximum integer values
    Exchange() : first_vehicle_index(INT_MAX),
                 first_trip_index(INT_MAX),
                 second_vehicle_index(INT_MAX),
                 second_trip_index(INT_MAX) {}
};

class Shift {
public:
    int source_vehicle_index;
    int source_trip_index;
    int dest_vehicle_index;
    int dest_trip_index; // The new trip is inserted after this index

    // Create a constructor that initializes these variables with maximum integer values
    Shift() : source_vehicle_index(INT_MAX),
              source_trip_index(INT_MAX),
              dest_vehicle_index(INT_MAX),
              dest_trip_index(INT_MAX) {}
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
double exchange_trips(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data&, Exchange&);
double exchange_depots(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data&, Exchange&);
double shift_trips(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data& data, Shift&);

void perform_exchange(std::vector<Vehicle>&, Exchange&);
void perform_shift(std::vector<Vehicle>&, Shift&);

void apply_best_improvement(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data&);
void optimize_rotations(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data&);
}

namespace locations {
void split_trips(std::vector<Vehicle>&, std::vector<int>&, int, int);
bool are_rotations_charge_feasible(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, std::vector<int>&);
void open_charging_station(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data&, int);
void close_charging_station(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data&, int);
double swap_charging_station(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data&, int, int);
void optimize_stations(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data&);
}


namespace diversification {
double exchange_three_trips(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ThreeExchange&);
double shift_all_trips(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data& data,
        int);
void perform_three_exchange(std::vector<Vehicle>&, ThreeExchange&);

void apply_operators(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data&);
void optimize_rotations(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data&);
}

namespace evaluation {
double calculate_objective(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data&);
double calculate_deadheading_cost(std::vector<Vehicle>&, std::vector<Trip>&);
double calculate_deadheading_cost(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<int>&);
void calculate_utilization(std::vector<Vehicle>& vehicle, std::vector<Trip>&, std::vector<Terminal>&);

bool is_two_exchange_compatible(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
bool is_three_exchange_compatible(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int, int, int);
bool is_shift_compatible(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);

bool is_charge_adequate_next_trip(std::vector<Trip>&, int, int, bool, bool, int, double&);
bool are_rotations_charge_feasible(std::vector<Trip>&, std::vector<Terminal>&, std::vector<std::vector<int>>);

double calculate_trip_replacement_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
double calculate_depot_replacement_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
double calculate_trip_addition_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
double calculate_trip_removal_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int);

bool is_savings_maximum(double, double, int, int, int, int, Exchange&);
bool is_savings_maximum(double, double, int, int, int, int, Shift&);
}

#endif //EBUS_VNS_OPERATORS_H
