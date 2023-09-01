#ifndef EBUS_VNS_HELPERS_H
#define EBUS_VNS_HELPERS_H

#include "logger.h"
#include "constants.h"
#include "vehicle.h"
#include <vector>
#include <sstream>

namespace preprocessing {
void initialize_inputs(std::string, std::vector<Trip> &, std::vector<Terminal> &, std::vector<Vehicle> &, int &, int &, Logger&);
void read_trip_data(std::string, std::vector<Trip> &, int &, Logger&);
void read_terminal_data(std::string, std::vector<Terminal> &, int &, Logger&);
void create_depot_trips(std::vector<Trip> &trip, std::vector<Terminal> &terminal, int &num_trips, int &num_terminals, Logger &logger);
void read_trip_pair_data(std::string, std::vector<Trip> &, int &, Logger&);
void initialize_vehicle_rotations(std::string, std::vector<Vehicle> &, Logger&);
void initialize_charge_locations(std::string, std::vector<Terminal> &, int &, Logger&);
}

namespace test {
void testing();
}

#endif //EBUS_VNS_HELPERS_H
