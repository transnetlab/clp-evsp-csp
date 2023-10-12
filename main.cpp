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
 * Code depot exchange operator in such a way that it can be used anywhere
 * Code start and end depots separately?
 * Whole code in a try catch status and log the result
 * Clean up existing code
 * Commit changes
 * Phase 2:
 * Expand operators to three exchanges and two shifts (from the same vehicle). Use flags to turn this feature on and off
 * Expand location operator to swap an open and closed charging station. Or pass it as a parameter to the function
 * Create a pull request and merge with main
 * Phase 3:
 * Create a skeleton for charge scheduling and implement CAG and GAC strategies
 * Implement and integrate CSP code
 * Phase 4:
 * Parallelize operators
 * Check logging outputs for different levels
 * Save compatibility checks in scheduling if it is used repeatedly
*/

int main(int argc, char* argv[])
{
    // Read the instance as command line argument. If not provided, use the default instance
    std::string instance;
    instance = (argc>1) ? argv[1] : "Ann_Arbor";

    // Delete any old log files if present and create a new one. Set logging level.
    std::remove(("../output/"+instance+"_log.txt").c_str());
    Logger logger("../output/"+instance+"_log.txt", true);
    logger.set_log_level_threshold(LogLevel::Debug);
    logger.log(LogLevel::Info, "Starting local search for instance "+instance+"...");
    postprocessing::write_output_data(instance, std::time(nullptr), logger);

    // Initialize variables
    int num_trips, num_augmented_trips, num_terminals;  // Number of trips and terminals in the network
    std::vector<Trip> trip;  // Vector of trips
    std::vector<Terminal> terminal;  // Vector of terminals
    std::vector<Vehicle> vehicle;  // Vector of vehicles

    // Read input data on trips and stops and initialize bus rotations
    preprocessing::initialize_inputs(instance, vehicle, trip, terminal, num_trips, num_augmented_trips, num_terminals, logger);

    // Calculate the objective value of the initial solution
    double best_objective = evaluation::calculate_objective(vehicle, trip, terminal, logger);

    // Local search for scheduling
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    double old_objective = INF;
    int num_iterations = 0;
    while (best_objective<old_objective) {
        ++num_iterations;
        old_objective = best_objective;
        scheduling::optimize_rotations(vehicle, trip, terminal,
                logger);  // Pick the best among exchanges and shifts // TODO: Have consistent tenses of namespaces
        best_objective = evaluation::calculate_objective(vehicle, trip, terminal,
                logger);  // TODO: Optimize scheduling can tell us if there is an improvement, so we don't need to recalculate the objective
    }

    // Local search for charging locations
    locations::optimize_stations(vehicle, trip, terminal, logger);

    // Print the best solution found
    logger.log(LogLevel::Info, "Finishing local search...");
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    // Find runtime
    double runtime = double(std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count())/1000.0; // in seconds
    logger.log(LogLevel::Info, "Local search completed in "+std::to_string(runtime)+" seconds.");

    // Print final vehicle rotations
    evaluation::calculate_utilization(vehicle, trip, terminal, logger);
    for (auto& curr_vehicle : vehicle)
        curr_vehicle.log_member_data(logger);

    // Postprocessing
    postprocessing::check_solution(vehicle, trip, terminal, num_trips, logger);
    postprocessing::write_output_data(vehicle, trip, terminal, num_trips, num_terminals, runtime, logger);
}
