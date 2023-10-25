#ifndef EBUS_VNS_HELPERS_H
#define EBUS_VNS_HELPERS_H

#include "logger.h"
#include "constants.h"
#include "vehicle.h"
#include "operators.h"
#include <vector>
#include <sstream>
#include <iomanip>

template <typename T, size_t N>
std::string array_to_string(const std::array<T, N>& arr) {
    std::ostringstream ss;
    ss << "{";
    for (size_t i = 0; i < N; ++i) {
        ss << arr[i];
        if (i < N - 1)
            ss << ", ";
    }
    ss << "}";
    return ss.str();
}

namespace preprocessing {
void read_trip_data(std::string, std::vector<Trip>&, int&, Logger&);
void read_terminal_data(std::string, std::vector<Terminal>&, int&, Logger&);
void create_depot_trips(std::vector<Trip>&, std::vector<Terminal>&, int&, int&, Logger&);
void read_trip_pair_data(std::string, std::vector<Trip>&, int&, Logger&);
void initialize_vehicle_rotations(std::string, std::vector<Vehicle>&, Logger&);
void log_input_data(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Logger&);
void initialize_inputs(std::string, std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, int&, int&, int&,
        Logger&);
}

namespace postprocessing {
void check_solution(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, int, Logger&);
void write_output_data(std::string, Logger&);
void write_output_data(std::string, std::time_t, Logger&);
void write_output_data(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, double, int, int, double, Logger&);
}

#endif //EBUS_VNS_HELPERS_H
