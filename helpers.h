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
void read_trip_data(std::vector<Trip>&, ProcessedData&);
void read_terminal_data(std::vector<Terminal>&, ProcessedData&);
void create_depot_trips(std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&);
void read_trip_pair_data(std::vector<Trip>&, ProcessedData&);
void initialize_vehicle_rotations(std::vector<Vehicle>&, ProcessedData&);
void create_energy_price_intervals(ProcessedData&);
void log_input_data(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&);
void initialize_inputs(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&);
}

namespace postprocessing {
void check_solution(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, ProcessedData&);
void write_summary(std::string message);
void write_summary(std::string instance, std::time_t curr_time);
void write_summary(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip, std::vector<Terminal>& terminal, double csp_cost, ProcessedData& processed_data);
void write_vehicle_results(std::vector<Vehicle>&, ProcessedData&);
void write_terminal_results(std::vector<Terminal>&, ProcessedData&);
}

#endif //EBUS_VNS_HELPERS_H
