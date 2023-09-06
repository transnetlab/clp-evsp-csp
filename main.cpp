#include "constants.h"
#include "csp.h"
#include "operators.h"
#include "logger.h"
#include "vehicle.h"
#include "helpers.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <sstream>
#include <chrono>
#include <algorithm>
#include <iomanip>

int main() {
    // Set the instance name
    std::string instance = "Ann_Arbor";

    // Delete any old log files if present and create a new one. Set logging level.
    std::remove(("../output/" + instance + "_log.txt").c_str());
    Logger logger("../output/" + instance + "_log.txt", true);
    logger.set_log_level_threshold(LogLevel::Info);

    // Initialize variables  TODO: Check if we need to keep track of number of original trips
    int num_trips, num_terminals;  // Number of trips and terminals in the network
    std::vector<Trip> trip;  // Vector of trips
    std::vector<Terminal> terminal;  // Vector of terminals
    std::vector<Vehicle> vehicle;  // Vector of vehicles

    // Read input data on trips and stops and initialize bus rotations
    preprocessing::initialize_inputs(instance, trip, terminal, vehicle, num_trips, num_terminals, logger);

    // Calculate the objective value of the initial solution
    evaluation::calculate_objective(trip, terminal, vehicle, logger);

    // VNS algorithm
    logger.log(LogLevel::Info, "Running VNS algorithm...");
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    exchanges::two_opt(vehicle, trip, terminal, logger);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    logger.log(LogLevel::Info, "VNS algorithm completed in " + std::to_string(std::chrono::duration_cast<std::chrono::seconds>(end - begin).count()) + " seconds.");

    return 0;
}
