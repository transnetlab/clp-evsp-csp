#ifndef EBUS_VNS_OPERATORS_H
#define EBUS_VNS_OPERATORS_H

#include "logger.h"
#include "constants.h"
#include "vehicle.h"

namespace exchanges {
void two_opt(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Logger&);

}

namespace shift {

}

namespace vns {

}

namespace shake {

}

namespace evaluation {
void calculate_objective(std::vector<Trip>&, std::vector<Terminal>&, std::vector<Vehicle>&, Logger&);
bool is_exchange_compatible(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
bool make_exchange_charge_feasible(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, std::vector<int>&, int, int, int, int);
double calculate_trip_replacement_cost(std::vector<Vehicle>&, std::vector<Trip>&, int, int, int, int);
}

#endif //EBUS_VNS_OPERATORS_H
