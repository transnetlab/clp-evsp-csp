#include "csp.h"
#include "operators.h"
#include "logger.h"
#include "vehicle.h"
#include "helpers.h"
#include <string>
#include <vector>
#include <chrono>

/* TODOs:
 * Perform sanity checks by adding up all the savings using static double variables
 * Use static variables in functions to count the number of times certain functions were used
 * Run profiler https://www.jetbrains.com/help/clion/cmake-profiling.html
 * Modify bash files to run concurrently on Gandalf
 * Check logging outputs for different levels
 * Log results at every exit point
 * Switch to plurals for vectors
 * Test results for a few iterations by replacing update vehicle indices with a full update
 * Enforce time budgets for each operator
 * Check if the version where opening gives savings, and we break works better*/

/* Solve joint problem from the starting point of the sequential problem. What is the %savings
 * Combine regular and depot exchanges
 * Incorporate some of the code optimization techniques from the notes
 * Use the reduced costs to update the savings
 * Optimize rotation-opportunity pairs
 * Optimize pricing for objective in uniform CSP
 * Try variants of scheduling -- Shift first and exchange later, random between the two, integrate diversification
*/

Logger logger(true);
bool SOLVE_CSP_JOINTLY = false;
bool PERFORM_THREE_EXCHANGES = false;
bool SHIFT_ALL_TRIPS = true;

int main(int argc, char* argv[])
{
    // Read the instance as command line argument. If not provided, use the default instance
    Data data; // Vector of parameters
    data.instance = (argc>1) ? argv[1] : "LA_Go";

    // Delete any old log files if present and create a new one. Set logging level.
    std::remove(("../output/"+data.instance+"_log.txt").c_str());
    logger.set_file_path("../output/"+data.instance+"_log.txt");
    logger.set_log_level_threshold(LogLevel::Info);

    logger.log(LogLevel::Info, "Starting local search for instance "+data.instance+"...");
    data.start_time_stamp = std::chrono::steady_clock::now();
    postprocessing::write_summary(data.instance, std::time(nullptr));  // TODO: Check if this is needed

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

    // Solve the charge scheduling problem
    data.log_csp_solution = true;
    double csp_cost = csp::select_optimization_model(vehicle, trip, terminal, data, "Split");

    // Log number of successful and unsuccessful openings from data
    logger.log(LogLevel::Info, "Number of successful openings: "+std::to_string(data.num_successful_openings));
    logger.log(LogLevel::Info, "Number of successful closures: "+std::to_string(data.num_successful_closures));

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
    postprocessing::write_summary(vehicle, trip, terminal, csp_cost, data);
    postprocessing::write_vehicle_results(vehicle, data);
    postprocessing::write_terminal_results(terminal, data);
    logger.log(LogLevel::Info, "Local search completed in "+std::to_string(data.runtime)+" seconds.");
}
