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
    Logger logger("../output/" + instance + "_log.txt", false);
    logger.set_log_level_threshold(LogLevel::Info);

    // Initialize variables  TODO: Check if we need to keep track of number of original trips
    int num_trips, num_terminals;  // Number of trips and terminals in the network
    std::vector<Trip> trip;  // Vector of trips
    std::vector<Terminal> terminal;  // Vector of terminals
    std::vector<Vehicle> vehicle;  // Vector of vehicles

    // Read input data on trips and stops and initialize bus rotations
    preprocessing::initialize_inputs(instance, trip, terminal, vehicle, num_trips, num_terminals, logger);

    // Calculate the objective value of the initial solution
    double best_objective = evaluation::calculate_objective(trip, terminal, vehicle, logger);

    // VNS algorithm
    logger.log(LogLevel::Info, "Running VNS algorithm...");
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    // TODO: Change order to vehicle, trip, terminal

    // Run the VNS algorithm for a limited number of iterations within a while loop
    int iteration = 0;

    while (iteration < MAX_ITERATIONS) {
        // Log iteration counter details
        logger.log(LogLevel::Info, "Iteration " + std::to_string(iteration + 1));

        // Find the best operator among trip exchanges and shifts trips and perform it. Repeat until no improvement is observed.
        while (true) {
            // Find the best exchange operator. The optimize_scheduling operator will ensure strict decrease in the objective value
            operators::optimize_scheduling(vehicle, trip, terminal, logger);  // TODO: Have consistent tenses of namespaces
            double new_objective = evaluation::calculate_objective(trip, terminal, vehicle, logger);

            // If the objective value has not improved, break out of the loop
            if ((new_objective-best_objective)*(new_objective-best_objective) < std::numeric_limits<double>::epsilon())
                break;

            best_objective = new_objective;  // Update the best objective value
        }

        // Log the rotation data
        logger.log(LogLevel::Info, "Final rotation data:");
        for (int i = 0; i < vehicle.size(); ++i) {
            logger.log(LogLevel::Info, "Vehicle " + std::to_string(i + 1));
            for (int j = 0; j < vehicle[i].trip_id.size(); ++j) {
                logger.log(LogLevel::Info, "Trip " + std::to_string(vehicle[i].trip_id[j]));
            }
        }

        // Find utilization of different terminals TODO: These can be tracked upfront and be updated incrementally after every shift and exchange
        evaluation::calculate_utilization(vehicle, trip, terminal, logger);

        // Find the best charging stations to open or close
        operators::optimize_locations(vehicle, trip, terminal, logger);
        evaluation::calculate_objective(trip, terminal, vehicle, logger);

        // Increment the iteration counter
        iteration++;
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    logger.log(LogLevel::Info, "VNS algorithm completed in " + std::to_string(std::chrono::duration_cast<std::chrono::seconds>(end - begin).count()) + " seconds.");

    // Calculate the objective value of the final solution
    //evaluation::calculate_objective(trip, terminal, vehicle, logger);
}
