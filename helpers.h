#ifndef EBUS_VNS_HELPERS_H
#define EBUS_VNS_HELPERS_H

#include "logger.h"
#include "constants.h"
#include "vehicle.h"
#include "operators.h"
#include <vector>
#include <map>
#include <sstream>
#include <iomanip>

template<typename T, size_t N>
std::string array_to_string(const std::array<T, N>& arr)
{
    std::ostringstream ss;
    ss << "{";
    for (size_t i = 0; i<N; ++i) {
        ss << arr[i];
        if (i<N-1)
            ss << ", ";
    }
    ss << "}";
    return ss.str();
}

namespace preprocessing {
void read_trip_data(std::vector<Trip>&, Data&);
void read_terminal_data(std::vector<Terminal>&, Data&);
void create_depot_trips(std::vector<Trip>&, std::vector<Terminal>&, Data&);
void read_trip_pair_data(std::vector<Trip>&, Data&);
void initialize_vehicle_rotations(std::vector<Vehicle>&, Data&);
void create_energy_price_intervals(Data&);
void log_input_data(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data&);
void initialize_inputs(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data&);
}

namespace postprocessing {
void check_solution(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data&);
void write_output_data(std::string);
void write_output_data(std::string, std::time_t);
void write_output_data(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, double, Data&);
}

#endif //EBUS_VNS_HELPERS_H
