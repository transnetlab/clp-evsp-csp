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

int main()
{
    // Set the instance name
    std::string instance = "Ann_Arbor";

    // Delete any old log files if present and create a new one. Set logging level.
    std::remove(("../output/"+instance+"_log.txt").c_str());
    Logger logger("../output/"+instance+"_log.txt", true);
    logger.set_log_level_threshold(LogLevel::Info);

    // Initialize variables  TODO: Check if we need to keep track of number of original trips vs. augmented trips
    int num_trips, num_terminals;  // Number of trips and terminals in the network
    std::vector<Trip> trip;  // Vector of trips
    std::vector<Terminal> terminal;  // Vector of terminals
    std::vector<Vehicle> vehicle;  // Vector of vehicles

    // Read input data on trips and stops and initialize bus rotations
    preprocessing::initialize_inputs(instance, trip, terminal, vehicle, num_trips, num_terminals, logger);

    // Calculate the objective value of the initial solution
    double best_objective = evaluation::calculate_objective(trip, terminal, vehicle, logger);
    // Local search for scheduling
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    double old_objective = 1e12;
    int num_iterations = 0;
    while (best_objective<old_objective) {
        ++num_iterations;
        old_objective = best_objective;
        operators::optimize_scheduling(vehicle, trip, terminal, logger);  // TODO: Have consistent tenses of namespaces
        best_objective = evaluation::calculate_objective(trip, terminal, vehicle, logger);
    }

    // Local search for charging locations
    operators::optimize_locations(vehicle, trip, terminal, logger);

    // Print the best solution found
    evaluation::calculate_objective(trip, terminal, vehicle, logger);
    evaluation::calculate_utilization(vehicle, trip, terminal, logger);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    logger.log(LogLevel::Info, "Local search completed in "
            +std::to_string(std::chrono::duration_cast<std::chrono::seconds>(end-begin).count())+" seconds.");

    // Print final vehicle rotations
    for (auto& v : vehicle)
        v.print_members();
}
