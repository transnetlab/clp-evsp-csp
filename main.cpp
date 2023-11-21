#include "csp.h"
#include "operators.h"
#include "logger.h"
#include "vehicle.h"
#include "helpers.h"
#include <string>
#include <vector>
#include <chrono>
#include <omp.h>

/* TODOs:
 * Run profiler https://www.jetbrains.com/help/clion/cmake-profiling.html
 * Modify bash files to run concurrently on Gandalf
 * Check logging outputs for different levels
 * Log results at every exit point
 * Switch to plurals for vectors and change to processed data
 * Integrate MIP CSP code */

/* Double check pricing in CSP uniform model
 * Enforce time budgets for each operator
 * Should CSP-based capacity costs be considered at the time of exchanges and shifts?
 * Try variants of scheduling -- Shift first and exchange later, random between the two, integrate diversification
 * Log iteration level outputs for final plots
*/

// Glocal variables
Logger logger(true);
bool SOLVE_CSP_JOINTLY = true;
int NUM_THREADS = omp_get_num_procs();  // Set this to appropriate number of threads

int main(int argc, char* argv[])
{
    // Read the instance as command line argument. If not provided, use the default instance
    ProcessedData processed_data; // Vector of parameters
    processed_data.instance = (argc>1) ? argv[1] : "Ann_Arbor";

    // Delete any old log files if present and create a new one. Set logging level.
    std::remove(("../output/"+processed_data.instance+"_log.txt").c_str());
    logger.set_file_path("../output/"+processed_data.instance+"_log.txt");
    logger.set_log_level_threshold(LogLevel::Info);

    logger.log(LogLevel::Info, "Starting local search for instance "+processed_data.instance+"...");
    processed_data.start_time_stamp = std::chrono::steady_clock::now();
    postprocessing::write_summary(processed_data.instance, std::time(nullptr));  // TODO: Check if this is needed

    // Initialize variables
    std::vector<Trip> trip;  // Vector of trips
    std::vector<Terminal> terminal;  // Vector of terminals
    std::vector<Vehicle> vehicle;  // Vector of vehicles

    // Read input processed_data on trips and stops and initialize bus rotations
    preprocessing::initialize_inputs(vehicle, trip, terminal, processed_data);

    // Diversify the solution by optimizing rotations. No changes to charging locations are made here.
    diversification::optimize_all_shifts(vehicle, trip, terminal, processed_data);

    // Local search for charging locations which also includes scheduling operators
    // locations::optimize_stations(vehicle, trip, terminal, processed_data);

    // Diversify the solution by optimizing rotations. No changes to charging locations are made here.
    // diversification::optimize_three_exchanges(vehicle, trip, terminal, processed_data);

    locations::optimize_integrated_model(vehicle, trip, terminal, processed_data);

    // Solve the charge scheduling problem
    processed_data.log_csp_solution = true;
    double csp_cost = csp::select_optimization_model(vehicle, trip, terminal, processed_data, "clp_csp");
    // Remove the cost of open charging stations
    for (auto& curr_terminal : terminal) {
        if (curr_terminal.is_charge_station)
            csp_cost -= CHARGE_LOC_COST;
    }

    // Close charging stations where CSP solution is zero based on charge capacity variables
    /*int num_extra_terminals_closed = 0;
    for (auto& curr_terminal : terminal) {
        if (curr_terminal.is_charge_station and fabs(curr_terminal.charge_capacity) < SMALL_EPSILON)
        {
            curr_terminal.is_charge_station = false;
            num_extra_terminals_closed++;
        }
    }
    logger.log(LogLevel::Info, "Closed "+std::to_string(num_extra_terminals_closed)+" extra terminals based on CSP.");*/

    // Find runtime
    logger.log(LogLevel::Info, "Finishing local search...");
    processed_data.end_time_stamp = std::chrono::steady_clock::now();
    processed_data.runtime = double(std::chrono::duration_cast<std::chrono::milliseconds>(
            processed_data.end_time_stamp-processed_data.start_time_stamp).count())/1000.0; // in seconds

    // Log final vehicle rotations
    evaluation::calculate_utilization(vehicle, trip, terminal, processed_data);
    for (auto& curr_vehicle : vehicle)
        curr_vehicle.log_member_data();

    // Postprocessing
    postprocessing::check_solution(vehicle, trip, terminal, processed_data);
    postprocessing::write_summary(vehicle, trip, terminal, csp_cost, processed_data);
    postprocessing::write_vehicle_results(vehicle, processed_data);
    postprocessing::write_terminal_results(terminal, processed_data);
    logger.log(LogLevel::Info, "Local search completed in "+std::to_string(processed_data.runtime)+" seconds.");
}
