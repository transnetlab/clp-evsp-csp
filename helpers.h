#ifndef EBUS_VNS_HELPERS_H
#define EBUS_VNS_HELPERS_H

#include "logger.h"
#include "constants.h"
#include "vehicle.h"
#include <vector>
#include <sstream>
#include <iomanip>

namespace preprocessing {
void initialize_inputs(std::string, std::vector<Trip> &, std::vector<Terminal> &, std::vector<Vehicle> &, int &, int &, Logger&);
void read_trip_data(std::string, std::vector<Trip> &, int &, Logger&);
void read_terminal_data(std::string, std::vector<Terminal> &, int &, Logger&);
void create_depot_trips(std::vector<Trip> &, std::vector<Terminal> &, int &, Logger &);
void read_trip_pair_data(std::string, std::vector<Trip> &, int &, Logger&);
void initialize_vehicle_rotations(std::string, std::vector<Vehicle> &, Logger&);
void log_input_data(std::vector<Trip> &, std::vector<Terminal> &, std::vector<Vehicle> &, Logger&);
}



#endif //EBUS_VNS_HELPERS_H
