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

/* TODOs:
 * Phase 1:
 * Review code for two shifts and charging station swaps
 * Create a function for sequential operators shifting more than two trips? Can we do this with exchanges as well?
 * Code review and clean up for consistency
 * Create a pull request and merge with main
 * Phase 2:
 * Create a skeleton for charge scheduling
 * Implement and integrate CSP code and its variants for a given set of rotations
 * Add CSP to scheduling operators
 * Create a pull request and merge with main
 * Phase 3:
 * Parallelize operators
 * Check logging outputs for different levels
 * Save compatibility checks in scheduling if it is used repeatedly
 * Create a pull request and merge with main*/

int main(int argc, char* argv[])
{
    // Read the instance as command line argument. If not provided, use the default instance
    std::string instance;
    instance = (argc>1) ? argv[1] : "Cornwall";

    // Delete any old log files if present and create a new one. Set logging level.
    std::remove(("../output/"+instance+"_log.txt").c_str());
    Logger logger("../output/"+instance+"_log.txt", true);
    logger.set_log_level_threshold(LogLevel::Info);
    logger.log(LogLevel::Info, "Starting local search for instance "+instance+"...");
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    postprocessing::write_output_data(instance, std::time(nullptr), logger);

    // Initialize variables
    int num_trips, num_augmented_trips, num_terminals;  // Number of trips and terminals in the network
    std::vector<Trip> trip;  // Vector of trips
    std::vector<Terminal> terminal;  // Vector of terminals
    std::vector<Vehicle> vehicle;  // Vector of vehicles

    // Read input data on trips and stops and initialize bus rotations
    preprocessing::initialize_inputs(instance, vehicle, trip, terminal, num_trips, num_augmented_trips, num_terminals,
            logger);

    // Local search for charging locations which also includes scheduling operators
    locations::optimize_stations(vehicle, trip, terminal, logger);

    // Diversify the solution by optimizing rotations. No changes to charging locations are made here.
    diversification::optimize_rotations(vehicle, trip, terminal, logger);

    // Solve the charge scheduling problem
    initialization::update_vehicles(trip, terminal, vehicle, logger);
    lp::solve_csp(trip, terminal, vehicle, logger);

    // Find runtime
    logger.log(LogLevel::Info, "Finishing local search...");
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    double runtime =
            double(std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count())/1000.0; // in seconds
    logger.log(LogLevel::Info, "Local search completed in "+std::to_string(runtime)+" seconds.");

    // Log final vehicle rotations
    evaluation::calculate_utilization(vehicle, trip, terminal, logger);
    for (auto& curr_vehicle : vehicle)
        curr_vehicle.log_member_data(logger);

    // Postprocessing
    postprocessing::check_solution(vehicle, trip, terminal, num_trips, logger);
    postprocessing::write_output_data(vehicle, trip, terminal, num_trips, num_terminals, runtime, logger);
}
