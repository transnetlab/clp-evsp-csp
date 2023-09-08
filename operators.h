#ifndef EBUS_VNS_OPERATORS_H
#define EBUS_VNS_OPERATORS_H

#include "logger.h"
#include "constants.h"
#include "vehicle.h"
#include <random>

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

namespace operators {
void optimize_scheduling(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip, std::vector<Terminal>& terminal,
        Logger& logger);
double exchange_trips(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip, std::vector<Terminal>& terminal,
        Exchange& exchange, Logger& logger);
double shift_trips(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Shift& shift, Logger&);
void perform_exchange(std::vector<Vehicle>&, std::vector<Terminal>&, Exchange&);
void perform_shift(std::vector<Vehicle>&, std::vector<Terminal>&, Shift&);
void optimize_locations(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip, std::vector<Terminal>& terminal,
        Logger& logger);
}

namespace shake {

}

namespace evaluation {
double calculate_objective(std::vector<Trip>&, std::vector<Terminal>&, std::vector<Vehicle>&, Logger&);
bool is_exchange_compatible(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
bool is_shift_compatible(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
bool are_rotations_charge_feasible(std::vector<Trip>&, std::vector<Terminal>&, std::vector<std::vector<int>>);

double calculate_trip_replacement_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
double calculate_trip_addition_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
double calculate_trip_removal_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int);
void calculate_utilization(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Logger&);
void update_best_solution(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, std::vector<Vehicle>&,
        std::vector<Terminal>&, double&, double, Logger&);
}

#endif //EBUS_VNS_OPERATORS_H
