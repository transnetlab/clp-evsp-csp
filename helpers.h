#ifndef EBUS_VNS_HELPERS_H
#define EBUS_VNS_HELPERS_H

#include "logger.h"
#include "constants.h"
#include "vehicle.h"
#include <vector>
#include <sstream>

namespace preprocessing {
    void read_input_data(std::string, std::vector<Trip> &, std::vector<Terminal> &, std::vector<Vehicle> &, int &, int &, Logger&);
    void read_trip_data(std::string, std::vector<Trip> &, int &, Logger&);
}

namespace test {
    void testing();
}

#endif //EBUS_VNS_HELPERS_H
