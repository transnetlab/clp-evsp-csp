#include "csp.h"
#include "operators.h"
#include "logger.h"
#include "vehicle.h"
#include "helpers.h"
#include <string>
#include <vector>
#include <chrono>

/* TODOs:
 * Find the first and last time steps and use them in the CSP
 * Use static variables in functions to count the number of times certain functions were used
 * Run profiler https://www.jetbrains.com/help/clion/cmake-profiling.html
 * Parallelize operators
 * Modify bash files to run concurrently on Gandalf
 * Check logging outputs for different levels
 * Create a pull request and merge with main
 * Log results at every exit point
 * Enforce time budgets for each operator
 * Break ties lexicographically in exchanges and shifts to make parallel results match the serial code?
 * Check if the version where opening gives savings and we break works better*/

/* Solve joint problem from the starting point of the sequential problem. What is the %savings
 * Integrate CSP in objective function calculation
 * Try variants of scheduling -- shift first and exchange later, random between the two, integrate diversification
*/

Logger logger(true);
bool SOLVE_CSP_JOINTLY = true;
bool PERFORM_THREE_EXCHANGES = false;
bool SHIFT_ALL_TRIPS = true;

int main(int argc, char* argv[])
{
    // Read the instance as command line argument. If not provided, use the default instance
    Data data; // Vector of parameters
    data.instance = (argc>1) ? argv[1] : "CityLink";

    // Delete any old log files if present and create a new one. Set logging level.
    std::remove(("../output/"+data.instance+"_log.txt").c_str());
    logger.set_file_path("../output/"+data.instance+"_log.txt");
    logger.set_log_level_threshold(LogLevel::Info);

    logger.log(LogLevel::Info, "Starting local search for instance "+data.instance+"...");
    data.start_time_stamp = std::chrono::steady_clock::now();
    postprocessing::write_output_data(data.instance, std::time(nullptr));  // TODO: Check if this is needed

    // Initialize variables
    std::vector<Trip> trip;  // Vector of trips
    std::vector<Terminal> terminal;  // Vector of terminals
    std::vector<Vehicle> vehicle;  // Vector of vehicles

    // Read input data on trips and stops and initialize bus rotations
    preprocessing::initialize_inputs(vehicle, trip, terminal, data);

    // Diversify the solution by optimizing rotations. No changes to charging locations are made here.
    diversification::optimize_rotations(vehicle, trip, terminal, data);

    // Only optimize rotations. No changes to charging locations are made here.
    // scheduling::optimize_rotations(vehicle, trip, terminal, data);

    // Local search for charging locations which also includes scheduling operators
    locations::optimize_stations(vehicle, trip, terminal, data);

    // Diversify the solution by optimizing rotations. No changes to charging locations are made here.
    // PERFORM_THREE_EXCHANGES = true;
    // SHIFT_ALL_TRIPS = false;
    // diversification::optimize_rotations(vehicle, trip, terminal, data);

    // Diversify the solution by optimizing rotations. No changes to charging locations are made here.
    // diversification::optimize_rotations(vehicle, trip, terminal, data);

    // Solve the charge scheduling problem
    double csp_cost = csp::select_optimization_model(vehicle, trip, terminal, data);  // TODO: Do we need both versions?

    // Find runtime
    logger.log(LogLevel::Info, "Finishing local search...");
    data.end_time_stamp = std::chrono::steady_clock::now();
    data.runtime = double(std::chrono::duration_cast<std::chrono::milliseconds>(
            data.end_time_stamp-data.start_time_stamp).count())/1000.0; // in seconds

    // Log final vehicle rotations
    evaluation::calculate_utilization(vehicle, trip, terminal);
    for (auto& curr_vehicle : vehicle)
        curr_vehicle.log_member_data();

    // Postprocessing
    postprocessing::check_solution(vehicle, trip, terminal, data);
    postprocessing::write_output_data(vehicle, trip, terminal, csp_cost, data);
    logger.log(LogLevel::Info, "Local search completed in "+std::to_string(data.runtime)+" seconds.");
}
