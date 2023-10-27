#include "operators.h"

/* VEHICLE AND CHARGE SCHEDULING OPERATORS
 * Two main types of operators are used to improving vehicle and charge scheduling (in joint models).
 * Exchanges are used to swap trips between two vehicle rotations.
 * Shifts are used to move trips from one vehicle rotation to another.
 * In each round, the best exchange or shift is applied in a greedy manner.
 * Additionally, end depots are also exchanged at the end which keeps capacity limits at depots intact.
 * The process is repeated until there is no improvement. */

// Function to find the best savings from exchanging trips
double scheduling::exchange_trips(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal,
        Exchange& exchange, Logger& logger)
{
    // Find a pair of vehicle rotation as a set from vehicles
    double max_savings = 0.0;  // Stores the maximum savings among all exchanges



    // If CSP is to be solved jointly with scheduling, find the CSP solution before exchanges
    double old_csp_cost = 0.0;
    if (SOLVE_CSP_JOINTLY)
        old_csp_cost = csp::select_optimization_model(vehicle, trip, terminal, logger);

    //  Exchange trips k and l of vehicles u and v
    #pragma omp parallel for collapse(2)
    for (int u = 0; u<vehicle.size(); ++u) {
        for (int v = u+1; v<vehicle.size(); ++v) {
            std::vector<int> update_vehicle_indices = {u, v};
            for (int k = 1; k<vehicle[u].trip_id.size()-1; ++k) {
                for (int l = 1; l<vehicle[v].trip_id.size()-1; ++l) {
                    // Store a vector of trip_id vectors after swapping trips
                    std::vector<std::vector<int>> swapped_rotations;

                    double savings = 0.0;
                    // Check if the exchanges is time compatible
                    if (evaluation::is_two_exchange_compatible(vehicle, trip, u, v, k, l)) {
                        // Check if exchanges are charge feasible. Push the original trip_ids to swapped_rotations
                        swapped_rotations.clear();
                        swapped_rotations.push_back(vehicle[u].trip_id);  // This has index 0 in swapped_rotations
                        swapped_rotations.push_back(vehicle[v].trip_id);  // This has index 1 in swapped_rotations

                        // Exchange trips k and l of vehicles u and v in swapped_rotations
                        int temp = swapped_rotations[0][k];
                        swapped_rotations[0][k] = swapped_rotations[1][l];
                        swapped_rotations[1][l] = temp;

                        if (evaluation::are_rotations_charge_feasible(trip, terminal, swapped_rotations)) {
                            // Calculate savings in deadheading from performing the exchange
                            savings += evaluation::calculate_trip_replacement_cost(vehicle, trip, u, v, k, l);
                            savings += evaluation::calculate_trip_replacement_cost(vehicle, trip, v, u, l, k);

                            if (SOLVE_CSP_JOINTLY) {
                                // Update a copy of the vehicle rotations
                                std::vector<Vehicle> vehicle_copy = vehicle;
                                vehicle_copy[u].trip_id = swapped_rotations[0];
                                vehicle_copy[v].trip_id = swapped_rotations[1];

                                // Solve the CSP model for the copy
                                double new_csp_cost = csp::select_optimization_model(vehicle_copy, trip, terminal, update_vehicle_indices, logger);
                                savings += old_csp_cost-new_csp_cost;
                            }

                            // Check if the exchange is the best so far
                            #pragma omp critical
                            if (savings>max_savings) {
                                max_savings = savings;
                                exchange.first_vehicle_index = u;
                                exchange.second_vehicle_index = v;
                                exchange.first_trip_index = k;
                                exchange.second_trip_index = l;
                            }
                        }
                    }
                }
            }
        }
    }
    return max_savings;
}

// Function to find the best savings from shifting trips
double scheduling::shift_trips(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip, std::vector<Terminal>& terminal,
        Shift& shift, Logger& logger)
{
    // Variables for calculating the savings from trip shifts
    double max_savings = 0.0;



    // If CSP is to be solved jointly with scheduling, find the CSP solution before exchanges
    double old_csp_cost = 0.0;
    if (SOLVE_CSP_JOINTLY)
        old_csp_cost = csp::select_optimization_model(vehicle, trip, terminal, logger);

    //  Insert trip l of vehicle v after trip k of vehicle u
    #pragma omp parallel for collapse(2)
    for (int u = 0; u<vehicle.size(); ++u) {
        for (int v = 0; v<vehicle.size(); ++v) {
            for (int k = 1; k<vehicle[u].trip_id.size()-1; ++k) {
                for (int l = 1; l<vehicle[v].trip_id.size()-1; ++l) {
                    if (u!=v) {
                        // Store a vector of trip_id vectors after shifting trips
                        std::vector<std::vector<int>> shifted_rotations;

                        double savings = 0.0;  // TODO: Can this become a subroutine that diversification can use?
                        // Check if the exchanges is time compatible and charge feasible
                        if (evaluation::is_shift_compatible(vehicle, trip, u, v, k, l)) {
                            // Check if exchanges are charge feasible. Insert the original trip_ids to shifted_rotations
                            shifted_rotations.clear();
                            shifted_rotations.push_back(vehicle[u].trip_id);  // This has index 0 in shifted_rotations
                            shifted_rotations.push_back(vehicle[v].trip_id);  // This has index 1 in shifted_rotations

                            // Insert trip l of vehicle v after trip k of vehicle u in shifted_rotations
                            shifted_rotations[0].insert(shifted_rotations[0].begin()+k+1, shifted_rotations[1][l]);
                            // Remove trip l of vehicle v from shifted_rotations
                            shifted_rotations[1].erase(shifted_rotations[1].begin()+l);

                            // Check if shifted_rotations[1] has only two trips. If so, delete it
                            if (shifted_rotations[1].size()==2)
                                shifted_rotations.erase(shifted_rotations.begin()+1);

                            if (evaluation::are_rotations_charge_feasible(trip, terminal, shifted_rotations)) {
                                // Calculate savings in deadheading from performing the exchange
                                savings += evaluation::calculate_trip_addition_cost(vehicle, trip, u, v, k, l);
                                savings += evaluation::calculate_trip_removal_cost(vehicle, trip, v, l);

                                if (SOLVE_CSP_JOINTLY) {
                                    // Update a copy of the vehicle rotations
                                    std::vector<Vehicle> vehicle_copy = vehicle;
                                    std::vector<int> update_vehicle_indices;
                                    vehicle_copy[u].trip_id = shifted_rotations[0];
                                    if (shifted_rotations.size()==1) {  // Delete vehicle with index v if needed
                                        vehicle_copy.erase(vehicle_copy.begin()+v);
                                        update_vehicle_indices = {u};
                                    }
                                    else {
                                        vehicle_copy[v].trip_id = shifted_rotations[1];
                                        update_vehicle_indices = {u, v};
                                    }

                                    // Solve the CSP model for the copy
                                    double new_csp_cost = csp::select_optimization_model(vehicle_copy, trip, terminal,
                                            update_vehicle_indices, logger);
                                    savings += old_csp_cost-new_csp_cost;
                                }

                                // Check if the exchange is the best so far
                                // Perform this check in a critical section
                                #pragma omp critical
                                if (savings>max_savings) {
                                    max_savings = savings;
                                    shift.dest_vehicle_index = u;
                                    shift.dest_trip_index = k;
                                    shift.source_vehicle_index = v;
                                    shift.source_trip_index = l;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return max_savings;
}

// Function to exchange the depot trips of two vehicles
double scheduling::exchange_depots(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Exchange& exchange, Logger& logger)
{
    // Find a pair of vehicle rotation as a set from vehicles
    double max_savings = 0.0;  // Stores the maximum savings among all exchanges

    // If CSP is to be solved jointly with scheduling, find the CSP solution before exchanges
    double old_csp_cost = 0.0;
    if (SOLVE_CSP_JOINTLY)
        old_csp_cost = csp::select_optimization_model(vehicle, trip, terminal, logger);

    //  Exchange the last trips of vehicles u and v. Depot trips are always compatible.
    #pragma omp parallel for collapse(2)
    for (int u = 0; u<vehicle.size(); ++u) {
        for (int v = u+1; v<vehicle.size(); ++v) {
            // Store a vector of trip_id vectors after swapping trips
            std::vector<std::vector<int>> swapped_rotations;

            std::vector<int> update_vehicle_indices = {u, v};
            int k = vehicle[u].trip_id.size()-1;
            int l = vehicle[v].trip_id.size()-1;
            double savings = 0.0;

            swapped_rotations.clear();
            swapped_rotations.push_back(vehicle[u].trip_id);  // This has index 0 in swapped_rotations
            swapped_rotations.push_back(vehicle[v].trip_id);  // This has index 1 in swapped_rotations

            // Exchange trips k and l of vehicles u and v in swapped_rotations
            int temp = swapped_rotations[0][k];
            swapped_rotations[0][k] = swapped_rotations[1][l];
            swapped_rotations[1][l] = temp;

            if (evaluation::are_rotations_charge_feasible(trip, terminal, swapped_rotations)) {
                // Calculate savings in deadheading from performing the exchange
                savings += evaluation::calculate_depot_replacement_cost(vehicle, trip, u, v, k, l);
                savings += evaluation::calculate_depot_replacement_cost(vehicle, trip, v, u, l, k);

                if (SOLVE_CSP_JOINTLY) {
                    // Update a copy of the vehicle rotations
                    std::vector<Vehicle> vehicle_copy = vehicle;
                    vehicle_copy[u].trip_id = swapped_rotations[0];
                    vehicle_copy[v].trip_id = swapped_rotations[1];

                    // Solve the CSP model for the copy
                    double new_csp_cost = csp::select_optimization_model(vehicle_copy, trip, terminal, update_vehicle_indices, logger);
                    savings += old_csp_cost-new_csp_cost;
                }

                // Check if the exchange is the best so far
                #pragma omp critical
                if (savings>max_savings) {
                    max_savings = savings;
                    exchange.first_vehicle_index = u;
                    exchange.second_vehicle_index = v;
                    exchange.first_trip_index = k;
                    exchange.second_trip_index = l;
                }
            }
        }
    }
    return max_savings;
}

// Function that actually performs the exchange using the exchange object
void scheduling::perform_exchange(std::vector<Vehicle>& vehicle, Exchange& exchange, Logger& logger)
{
    logger.log(LogLevel::Info, "Performing exchange...");

    // Exchange trips k and l of vehicles u and v
    int first_vehicle_index = exchange.first_vehicle_index;
    int second_vehicle_index = exchange.second_vehicle_index;
    int first_trip_index = exchange.first_trip_index;
    int second_trip_index = exchange.second_trip_index;

    // Log trip IDs before exchange
    logger.log(LogLevel::Debug, "First vehicle trip IDs: "+vector_to_string(vehicle[first_vehicle_index].trip_id));
    logger.log(LogLevel::Debug,
            "Second vehicle trip IDs: "+vector_to_string(vehicle[second_vehicle_index].trip_id));

    // Log exchange information
    logger.log(LogLevel::Debug, "Exchanging trip index "+std::to_string(first_trip_index)+" of vehicle index "
            +std::to_string(first_vehicle_index)+" with trip index "+std::to_string(second_trip_index)
            +" of vehicle index "
            +std::to_string(second_vehicle_index)+"...");

    // Swap the trips
    int temp = vehicle[first_vehicle_index].trip_id[first_trip_index];
    vehicle[first_vehicle_index].trip_id[first_trip_index] = vehicle[second_vehicle_index].trip_id[second_trip_index];
    vehicle[second_vehicle_index].trip_id[second_trip_index] = temp;

    // Log trip IDs after exchange for both first vehicle and second vehicle
    logger.log(LogLevel::Debug,
            "First vehicle new trip IDs: "+vector_to_string(vehicle[first_vehicle_index].trip_id));
    logger.log(LogLevel::Debug,
            "Second vehicle new trip IDs: "+vector_to_string(vehicle[second_vehicle_index].trip_id));
}

// Function that actually performs the shift using the shift object
void scheduling::perform_shift(std::vector<Vehicle>& vehicle, Shift& shift, Logger& logger)
{
    logger.log(LogLevel::Info, "Performing shift...");

    // Insert trip l of vehicle v after trip k of vehicle u
    int dest_vehicle_index = shift.dest_vehicle_index;
    int source_vehicle_index = shift.source_vehicle_index;
    int dest_trip_index = shift.dest_trip_index;
    int source_trip_index = shift.source_trip_index;

    // Log trip IDs before shift
    logger.log(LogLevel::Debug,
            "Source vehicle trip IDs: "+vector_to_string(vehicle[source_vehicle_index].trip_id));
    logger.log(LogLevel::Debug,
            "Destination vehicle trip IDs: "+vector_to_string(vehicle[dest_vehicle_index].trip_id));

    // Log shift information
    logger.log(LogLevel::Debug, "Shifting trip index "+std::to_string(source_trip_index)+" of vehicle index "
            +std::to_string(source_vehicle_index)+" after trip index "+std::to_string(dest_trip_index)
            +" of vehicle index "
            +std::to_string(dest_vehicle_index)+"...");

    // Insert the trip
    vehicle[dest_vehicle_index].trip_id.insert(vehicle[dest_vehicle_index].trip_id.begin()+dest_trip_index+1,
            vehicle[source_vehicle_index].trip_id[source_trip_index]);

    // Remove the trip
    vehicle[source_vehicle_index].trip_id.erase(vehicle[source_vehicle_index].trip_id.begin()+source_trip_index);

    // If the source vehicle has only two trips, remove the vehicle
    if (vehicle[source_vehicle_index].trip_id.size()==2) {
        logger.log(LogLevel::Debug, "Source vehicle has only two trips. Removing it...");
        vehicle.erase(vehicle.begin()+source_vehicle_index);
    }
    else
        logger.log(LogLevel::Debug,
                "Source vehicle new trip IDs: "+vector_to_string(vehicle[source_vehicle_index].trip_id));

    // Log the rotations after the shift for the destination vehicle
    logger.log(LogLevel::Debug,
            "Destination vehicle new trip new IDs: "+vector_to_string(vehicle[dest_vehicle_index].trip_id));
}

// Best improvement function to optimize rotations using shifts and exchanges
void scheduling::apply_best_improvement(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Logger& logger)
{
    logger.log(LogLevel::Info, "Finding the best improvement operator...");

    Exchange exchange;
    Shift shift;

    // Run the exchange and shift locations
    double exchange_savings = exchange_trips(vehicle, trip, terminal, exchange, logger);
    double shift_savings = shift_trips(vehicle, trip, terminal, shift, logger);

    // Check if the exchanges operator is better than the shifts operator
    logger.log(LogLevel::Info, "Savings from exchange operator: "+std::to_string(exchange_savings));
    logger.log(LogLevel::Info, "Savings from shift operator: "+std::to_string(shift_savings));

    // Check if savings are positive
    if (exchange_savings<EPSILON and shift_savings<EPSILON) {
        logger.log(LogLevel::Info, "No improvement possible from trip exchanges or shifts...");

        logger.log(LogLevel::Info, "Checking for improvement from depot exchanges...");
        double depot_exchange_savings = exchange_depots(vehicle, trip, terminal, exchange, logger);
        logger.log(LogLevel::Info,
                "Savings from depot exchange operator: "+std::to_string(depot_exchange_savings));
        (depot_exchange_savings>EPSILON) ? perform_exchange(vehicle, exchange, logger) : logger.log(LogLevel::Info,
                "No improvement possible from depot exchanges...");
        return;
    }

    if (exchange_savings>shift_savings)
        perform_exchange(vehicle, exchange, logger);
    else
        perform_shift(vehicle, shift, logger);
}

// Function to optimize rotations using repeated shifts and exchanges till there is no improvement
void scheduling::optimize_rotations(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Logger& logger)
{
    logger.log(LogLevel::Info, "Optimizing rotations...");

    // Initialize variables
    double old_objective = INF;
    double new_objective = evaluation::calculate_objective(vehicle, trip, terminal, logger);

    // Loop until there is no improvement
    while (new_objective<old_objective) {
        old_objective = new_objective;
        scheduling::apply_best_improvement(vehicle, trip, terminal, logger);
        new_objective = evaluation::calculate_objective(vehicle, trip, terminal, logger);
    }
}

/* LOCATION OPERATORS
 * These are used to open/close charge stations or swap an open-closed station pair and serve as first-level decisions.
 * Closing charging station can lead to charge infeasibility. This is addressed by splitting rotations.
 * Opening and closing charging stations can lead to suboptimal rotations.
 * Hence, the scheduling operators are applied after opening and closing charging stations. */

// Create new rotations by splitting after trip_index if it is not charge feasible
void locations::split_trips(std::vector<Vehicle>& vehicle, std::vector<int>& scan_eligible_vehicle_indices,
        int vehicle_index, int trip_index, Logger& logger)
{
    // Log operations
    logger.log(LogLevel::Info, "Splitting vehicle index "+std::to_string(vehicle_index)+" after trip ID "
            +std::to_string(vehicle[vehicle_index].trip_id[trip_index])+"...");

    // Log trip IDs before splitting
    logger.log(LogLevel::Info, "Trip IDs before splitting: "+vector_to_string(vehicle[vehicle_index].trip_id));
    Vehicle new_vehicle;  // Create a new vehicle with the remaining trips

    new_vehicle.id = vehicle[vehicle.size()-1].id+1;
    new_vehicle.trip_id.push_back(vehicle[vehicle_index].trip_id[0]);  // Add the depot
    new_vehicle.trip_id.insert(new_vehicle.trip_id.begin()+1, vehicle[vehicle_index].trip_id.begin()+trip_index+1,
            vehicle[vehicle_index].trip_id.end()); // Add the second half of the original rotation
    vehicle.push_back(new_vehicle);

    // Remove existing trips from vehicle_index after trip_index till the last but one element
    vehicle[vehicle_index].trip_id.erase(vehicle[vehicle_index].trip_id.begin()+trip_index+1,
            vehicle[vehicle_index].trip_id.end()-1);

    // Log trip IDs after splitting
    logger.log(LogLevel::Info, "Old vehicle trip IDs: "+vector_to_string(vehicle[vehicle_index].trip_id));
    logger.log(LogLevel::Info, "New vehicle trip IDs: "+vector_to_string(vehicle[vehicle.size()-1].trip_id));

    // Add the new and old vehicles to scan_eligible_vehicle_indices
    scan_eligible_vehicle_indices.push_back(vehicle.size()-1);
    scan_eligible_vehicle_indices.push_back(vehicle_index);
}

// Function that checks if a rotation is charge feasible. If not it also calls a function to split trips
bool locations::are_rotations_charge_feasible(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, std::vector<int>& scan_eligible_vehicle_indices, Logger& logger)
{
    // Loop until the list of un-checked rotations is empty
    while (not scan_eligible_vehicle_indices.empty()) {
        bool is_splitting_required = false;  // Flag to indicate if splitting is required
        int split_trip_index = -1;  // Farthest trip index after which splitting is possible while reaching the depot

        // Pop the last element from scan_eligible_vehicle_indices
        int v = scan_eligible_vehicle_indices.back();
        scan_eligible_vehicle_indices.pop_back();
        logger.log(LogLevel::Debug, "Checking feasibility of vehicle: "+std::to_string(v));

        // Calculate the energy required for deadheading from the depot to the end of the first trip
        double charge_level =
                MAX_CHARGE_LEVEL-(trip[vehicle[v].trip_id[0]-1].deadhead_distance[vehicle[v].trip_id[1]-1]
                        +trip[vehicle[v].trip_id[1]-1].distance)*ENERGY_PER_KM;

        // If energy level is below the minimum threshold, splitting will not help. Return false and break.
        // This condition isn't required for existing rotations. But second parts of split rotations can be infeasible.
        if (charge_level<MIN_CHARGE_LEVEL)
            return false;

        int last_trip = vehicle[v].trip_id[vehicle[v].trip_id.size()-1];  // This represents the auxiliary depot trip
        double charge_level_until_depot;

        int curr_trip, next_trip;  // Current trip and next trip IDs
        int end_terminal_curr_trip, start_terminal_next_trip; // End terminal of current trip and start terminal of the next trip
        bool is_curr_trip_end_charge_terminal, is_next_trip_start_charge_terminal;
        int charge_time_window;  // Idle time during which charging is allowed

        // Scan through the trips in the rotation
        for (int i = 1; i<vehicle[v].trip_id.size()-2; ++i) {
            curr_trip = vehicle[v].trip_id[i];
            next_trip = vehicle[v].trip_id[i+1];

            end_terminal_curr_trip = trip[curr_trip-1].end_terminal;  // End terminal id of current trip
            start_terminal_next_trip = trip[next_trip-1].start_terminal;  // Start terminal id of next trip

            is_curr_trip_end_charge_terminal = terminal[end_terminal_curr_trip-1].is_charge_station;
            is_next_trip_start_charge_terminal = terminal[start_terminal_next_trip-1].is_charge_station;

            charge_time_window = trip[curr_trip-1].idle_time[next_trip-1];

            charge_level_until_depot =
                    charge_level-(trip[curr_trip-1].deadhead_distance[last_trip-1])*ENERGY_PER_KM;

            if (charge_level_until_depot>=MIN_CHARGE_LEVEL)
                split_trip_index = i;

            if (not evaluation::is_charge_adequate_next_trip(trip, curr_trip, next_trip,
                    is_curr_trip_end_charge_terminal, is_next_trip_start_charge_terminal, charge_time_window,
                    charge_level)) {
                is_splitting_required = true;
                break;
            }
        }

        // If splitting is required, we need not check for charge feasibility from the last actual trip to depot
        if (not is_splitting_required) {
            // Add distance from the last actual trip to the depot to cumulative_energy
            int penultimate_trip = vehicle[v].trip_id[vehicle[v].trip_id.size()-2];
            charge_level -= trip[penultimate_trip-1].deadhead_distance[last_trip-1]*ENERGY_PER_KM;
            if (charge_level<MIN_CHARGE_LEVEL)
                is_splitting_required = true;
        }

        if (is_splitting_required) {
            if (split_trip_index == -1){
                logger.log(LogLevel::Error, "Splitting is required, but split trip index not found...");
                return false;
            }
            locations::split_trips(vehicle, scan_eligible_vehicle_indices, v, split_trip_index, logger);
        }
    }
    return true;
}

// Function to open charging stations and check if savings can be achieved from scheduling operators
void locations::open_charging_station(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, int open_terminal_id, Logger& logger)
{
    logger.log(LogLevel::Info,
            "Checking for improvement from opening charge station at terminal ID "
                    +std::to_string(open_terminal_id));
    logger.log(LogLevel::Info, "Before opening the charging station...");
    double best_objective = evaluation::calculate_objective(vehicle, trip, terminal, logger);

    // Opening charging station will increase the cost. Check if savings can be achieved from scheduling operators
    logger.log(LogLevel::Info, "After opening the charging station...");
    terminal[open_terminal_id-1].is_charge_station = true;  // Open the terminal with the chosen index

    logger.log(LogLevel::Info, "Applying local search operators to adjust scheduling...");
    std::vector<Vehicle> vehicle_copy = vehicle;  // Create a copy of the vehicle vector to check for savings
    scheduling::optimize_rotations(vehicle_copy, trip, terminal, logger);

    // Check if the current objective is better than the best objective that we started with
    double curr_objective = evaluation::calculate_objective(vehicle_copy, trip, terminal, logger);
    if (curr_objective<best_objective) {
        logger.log(LogLevel::Info, "Objective improved by "+std::to_string(best_objective-curr_objective));
        vehicle = vehicle_copy;
    }
    else {
        logger.log(LogLevel::Info, "No improvement from opening the charging station. Reverting changes...");
        terminal[open_terminal_id-1].is_charge_station = false;
    }
}

// Function that closes charging stations and creates new rotations if required
void locations::close_charging_station(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, int close_terminal_id, Logger& logger)
{
    logger.log(LogLevel::Info,
            "Checking for improvement from closing charge station at terminal ID "
                    +std::to_string(close_terminal_id));
    logger.log(LogLevel::Info, "Before closing the charging station...");
    double best_objective = evaluation::calculate_objective(vehicle, trip, terminal, logger);

    // Scan through the rotations and find vehicle indices that have the chosen terminal and add them to a scan eligible list
    std::vector<int> scan_eligible_vehicle_indices;
    int curr_trip, next_trip;  // Current trip and next trip IDs
    bool is_curr_trip_end_charge_terminal;
    for (int v = 0; v<vehicle.size(); ++v) {
        for (int i = 1; i<vehicle[v].trip_id.size()-2; ++i) {
            curr_trip = vehicle[v].trip_id[i];
            next_trip = vehicle[v].trip_id[i+1];
            is_curr_trip_end_charge_terminal = terminal[trip[curr_trip-1].end_terminal-1].is_charge_station;

            if ((trip[curr_trip-1].end_terminal==close_terminal_id)
                    or (trip[next_trip-1].start_terminal==close_terminal_id
                            and not is_curr_trip_end_charge_terminal)) {
                scan_eligible_vehicle_indices.push_back(v);
                break;
            }
        }
    }
    logger.log(LogLevel::Info, "Vehicle indices affected: "+vector_to_string(scan_eligible_vehicle_indices));
    logger.log(LogLevel::Info, "After closing the charging station and creating new rotations (if any)...");
    terminal[close_terminal_id-1].is_charge_station = false;  // Close the terminal with the chosen index
    std::vector<Vehicle> vehicle_copy = vehicle;  // Create a copy of the vehicle vector to check for savings

    // Check if rotations are feasible after deletion of the terminal. If not, create new rotations.
    if (not locations::are_rotations_charge_feasible(vehicle_copy, trip, terminal,
            scan_eligible_vehicle_indices, logger)) {
        logger.log(LogLevel::Info, "Closing station makes the problem infeasible. Reverting changes...");
        terminal[close_terminal_id-1].is_charge_station = true;
        return;
    }

    // Check if savings can be achieved from applying the scheduling operators
    logger.log(LogLevel::Info, "Applying local search locations to adjust scheduling...");
    scheduling::optimize_rotations(vehicle_copy, trip, terminal, logger);

    // Check if the current objective is better than the best objective that we started with
    double curr_objective = evaluation::calculate_objective(vehicle_copy, trip, terminal, logger);
    if (curr_objective<best_objective) {
        logger.log(LogLevel::Info, "Objective improved by "+std::to_string(best_objective-curr_objective));
        vehicle = vehicle_copy;
    }
    else {
        logger.log(LogLevel::Info, "No improvement from closing the charging station. Reverting changes...");
        terminal[close_terminal_id-1].is_charge_station = true;
    }
}

double locations::swap_charging_station(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, int open_terminal_id, int close_terminal_id, Logger& logger)
{
    logger.log(LogLevel::Info,
            "Checking for improvement from swapping charge station at terminal ID "
                    +std::to_string(close_terminal_id)+" with terminal ID "+std::to_string(open_terminal_id));

    logger.log(LogLevel::Info, "Before swapping the charging station...");
    double best_objective = evaluation::calculate_objective(vehicle, trip, terminal, logger);

    logger.log(LogLevel::Info, "After opening the charging station...");
    terminal[open_terminal_id-1].is_charge_station = true;  // Open the terminal with the chosen index

    // Scan through the rotations and find vehicle indices that have the chosen terminal and add them to a scan eligible list
    std::vector<int> scan_eligible_vehicle_indices;
    int curr_trip, next_trip;  // Current trip and next trip IDs
    bool is_curr_trip_end_charge_terminal;
    for (int v = 0; v<vehicle.size(); ++v) {
        for (int i = 1; i<vehicle[v].trip_id.size()-2; ++i) {
            curr_trip = vehicle[v].trip_id[i];
            next_trip = vehicle[v].trip_id[i+1];
            is_curr_trip_end_charge_terminal = terminal[trip[curr_trip-1].end_terminal-1].is_charge_station;

            if ((trip[curr_trip-1].end_terminal==close_terminal_id)
                    or (trip[next_trip-1].start_terminal==close_terminal_id
                            and not is_curr_trip_end_charge_terminal)) {
                scan_eligible_vehicle_indices.push_back(v);
                break;
            }
        }
    }

    logger.log(LogLevel::Info, "Vehicle indices affected: "+vector_to_string(scan_eligible_vehicle_indices));
    logger.log(LogLevel::Info, "After closing the charging station and creating new rotations (if any)...");
    terminal[close_terminal_id-1].is_charge_station = false;  // Close the terminal with the chosen index
    std::vector<Vehicle> vehicle_copy = vehicle;  // Create a copy of the vehicle vector to check for savings
    // Check if rotations are feasible after deletion of the terminal. If not, create new rotations.
    if (not locations::are_rotations_charge_feasible(vehicle_copy, trip, terminal,
            scan_eligible_vehicle_indices, logger)) {
        logger.log(LogLevel::Info, "Closing station makes the problem infeasible. Reverting changes...");
        terminal[close_terminal_id-1].is_charge_station = true;
        return INF;
    }

    // Check if savings can be achieved from applying the scheduling operators
    logger.log(LogLevel::Info, "Applying local search locations to adjust scheduling...");
    scheduling::optimize_rotations(vehicle_copy, trip, terminal, logger);

    // Check if the current objective is better than the best objective that we started with
    double curr_objective = evaluation::calculate_objective(vehicle_copy, trip, terminal, logger);
    if (curr_objective<best_objective) {
        logger.log(LogLevel::Info,
                "Swapping charging stations improves the objective by "+std::to_string(best_objective-curr_objective));
        vehicle = vehicle_copy;
    }
    /*else {
        logger.log(LogLevel::Info, "No improvement from swapping charge stations. Reverting changes...");
        terminal[close_terminal_id-1].is_charge_station = true;
        terminal[open_terminal_id-1].is_charge_station = false;
    }*/

    return best_objective-curr_objective;
}

// Function to open or close locations. This could create new rotations as well when locations are closed
void locations::optimize_stations(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Logger& logger)
{
    logger.log(LogLevel::Info, "Optimizing locations...");
    std::vector<int> open_terminal_ids;
    std::vector<int> close_terminal_ids;

    // Step 1: Close open charging stations with zero utilization
    logger.log(LogLevel::Info, "Checking if charge stations with zero utilization can be closed...");
    evaluation::calculate_utilization(vehicle, trip, terminal, logger);
    int num_zero_utilization_terminals = 0;
    for (auto& curr_terminal : terminal) {
        if (curr_terminal.is_charge_station and curr_terminal.current_idle_time==0) {
            curr_terminal.is_charge_station = false;
            ++num_zero_utilization_terminals;
        }
    }
    logger.log(LogLevel::Info, "Number of charge stations closed: "+std::to_string(num_zero_utilization_terminals));

    // Step 2: Open charging stations
    // Open a closed charging station in decreasing order of utilization. Calculate savings considering scheduling.
    evaluation::calculate_utilization(vehicle, trip, terminal, logger);
    open_terminal_ids.clear();
    for (const auto& curr_terminal : terminal)
        if (not curr_terminal.is_charge_station and curr_terminal.potential_idle_time>0.0)
            open_terminal_ids.push_back(curr_terminal.id);

    // Sort the terminals that are closed in descending order of utilization
    std::sort(open_terminal_ids.begin(), open_terminal_ids.end(),
            [&terminal](int a, int b) { return terminal[a-1].potential_idle_time>terminal[b-1].potential_idle_time; });

    // Print the sorted terminal IDs
    logger.log(LogLevel::Info, "Sorted terminals to be opened: "+vector_to_string(open_terminal_ids));

    // Loop through the sorted terminal ID list and check if opening them leads to an improvement in the objective
    for (const auto& open_terminal_id : open_terminal_ids)
        locations::open_charging_station(vehicle, trip, terminal, open_terminal_id, logger);

    // Step 3: Close charging stations
    // Sort the terminals that are closed in ascending order of their utilization
    evaluation::calculate_utilization(vehicle, trip, terminal, logger);
    close_terminal_ids.clear();
    for (const auto& curr_terminal : terminal)
        if (curr_terminal.is_charge_station)
            close_terminal_ids.push_back(curr_terminal.id);

    // Sort the terminals that are open in ascending order of utilization
    std::sort(close_terminal_ids.begin(), close_terminal_ids.end(),
            [&terminal](int a, int b) { return terminal[a-1].current_idle_time<terminal[b-1].current_idle_time; });

    // Print the sorted terminal IDs
    logger.log(LogLevel::Info, "Sorted terminals to be closed: "+vector_to_string(close_terminal_ids));

    // Loop through the sorted terminal ID list and check if closing them leads to an improvement in the objective
    for (const auto& close_terminal_id : close_terminal_ids)  // TODO: Should we recalculate utilization after closing?
        locations::close_charging_station(vehicle, trip, terminal, close_terminal_id, logger);

    // Step 4: Swap open and closed charging stations
    if (SWAP_CHARGE_STATIONS) {
        evaluation::calculate_utilization(vehicle, trip, terminal, logger);
        // Find the list of terminals that can be opened
        open_terminal_ids.clear();
        for (const auto& curr_terminal : terminal)
            if (not curr_terminal.is_charge_station and curr_terminal.potential_idle_time>IDLE_TIME_THRESHOLD)
                open_terminal_ids.push_back(curr_terminal.id);

        // Sort the terminals that are closed in descending order of utilization
        std::sort(open_terminal_ids.begin(), open_terminal_ids.end(),
                [&terminal](int a, int b) {
                  return terminal[a-1].potential_idle_time>terminal[b-1].potential_idle_time;
                });

        // Find the list of terminals that can be closed
        close_terminal_ids.clear();
        for (const auto& curr_terminal : terminal)
            if (curr_terminal.is_charge_station and curr_terminal.current_idle_time>IDLE_TIME_THRESHOLD)
                close_terminal_ids.push_back(curr_terminal.id);

        // Sort the terminals that are open in ascending order of utilization
        std::sort(close_terminal_ids.begin(), close_terminal_ids.end(),
                [&terminal](int a, int b) { return terminal[a-1].current_idle_time<terminal[b-1].current_idle_time; });

        // Loop through the list of open and closed terminals and check if swapping improves the objective
        double max_savings = 0.0;
        double savings;
        std::vector<Vehicle> optimal_vehicle;
        std::vector<Vehicle> original_vehicle = vehicle;
        std::vector<Terminal> original_terminal = terminal;
        int optimal_open_terminal_id, optimal_close_terminal_id;
        if (open_terminal_ids.size()>1 and close_terminal_ids.size()>1) {
            // Log the list of open and closed charging stations being considered for swapping
            logger.log(LogLevel::Info, "Terminals that could be opened: "+vector_to_string(open_terminal_ids));
            logger.log(LogLevel::Info, "Terminals that could be closed: "+vector_to_string(close_terminal_ids));
            for (const auto& open_terminal_id : open_terminal_ids) {
                for (const auto& close_terminal_id : close_terminal_ids) {
                    vehicle = original_vehicle;
                    savings = locations::swap_charging_station(vehicle, trip, terminal, open_terminal_id,
                            close_terminal_id,
                            logger);
                    terminal = original_terminal;
                    if (savings>max_savings) {
                        max_savings = savings;
                        optimal_vehicle = vehicle;
                        optimal_open_terminal_id = open_terminal_id;
                        optimal_close_terminal_id = close_terminal_id;
                        logger.log(LogLevel::Info,
                                "Savings of "+std::to_string(max_savings)+" found from swapping charging stations");
                    }
                }
            }
        }

        // Perform the swap if the savings are positive
        if (max_savings>0.0) {
            logger.log(LogLevel::Info, "Swapping charging stations...");
            vehicle = optimal_vehicle;
            terminal[optimal_open_terminal_id-1].is_charge_station = true;
            terminal[optimal_close_terminal_id-1].is_charge_station = false;
        }
        else
            logger.log(LogLevel::Info, "No improvement from swapping charging stations...");
    }
}

/* DIVERSIFICATION OPERATORS
 * This section of the code contains diversification operators.
 * These operators are employed to escape local minima.
 * They include shifting more than one trip and exchanging more than two trips.
 * However, they are computationally expensive and hence are not employed by default.*/

// Function to find the best savings from exchanging three trips in three rotations
double diversification::exchange_three_trips(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, ThreeExchange& three_exchange)
{
    // Find a rotation of three vehicles which gives the maximum savings among all exchanges
    double max_savings = 0.0;

    // Store a vector of trip_id vectors after swapping trips
    std::vector<std::vector<int>> swapped_rotations;

    //  Exchange trips (k, l, m) of vehicles (u, v, w) to (m, l, k)
    for (int u = 0; u<vehicle.size(); ++u) {
        for (int v = 0; v<vehicle.size(); ++v) {
            for (int w = 0; w<vehicle.size(); ++w) {
                // Pick unique u, v, w
                if (u==v or u==w or v==w)
                    continue;

                // Select a trip from each rotation
                for (int k = 1; k<vehicle[u].trip_id.size()-1; ++k) {
                    for (int l = 1; l<vehicle[v].trip_id.size()-1; ++l) {
                        for (int m = 1; m<vehicle[w].trip_id.size()-1; ++m) {
                            double savings = 0.0;
                            // Check if the exchanges is time compatible
                            if (evaluation::is_three_exchange_compatible(vehicle, trip, u, v, w, k, l, m)) {
                                // Check if exchanges are charge feasible. Push the original trip_ids to swapped_rotations
                                swapped_rotations.clear();
                                swapped_rotations.push_back(
                                        vehicle[u].trip_id);  // This has index 0 in swapped_rotations
                                swapped_rotations.push_back(
                                        vehicle[v].trip_id);  // This has index 1 in swapped_rotations
                                swapped_rotations.push_back(
                                        vehicle[w].trip_id);  // This has index 2 in swapped_rotations

                                // Exchange trips such that m goes to u, k goes to v, and l goes to w
                                int temp_id_k = swapped_rotations[0][k];
                                int temp_id_l = swapped_rotations[1][l];
                                int temp_id_m = swapped_rotations[2][m];

                                swapped_rotations[0][k] = temp_id_m;
                                swapped_rotations[1][l] = temp_id_k;
                                swapped_rotations[2][m] = temp_id_l;

                                if (evaluation::are_rotations_charge_feasible(trip, terminal, swapped_rotations)) {
                                    // Calculate savings in deadheading from performing the exchange
                                    savings += evaluation::calculate_trip_replacement_cost(vehicle, trip, u, w, k, m);
                                    savings += evaluation::calculate_trip_replacement_cost(vehicle, trip, v, u, l, k);
                                    savings += evaluation::calculate_trip_replacement_cost(vehicle, trip, w, v, m, l);

                                    // Check if the exchange is the best so far
                                    if (savings>max_savings) {
                                        max_savings = savings;
                                        three_exchange.first_vehicle_index = u;
                                        three_exchange.second_vehicle_index = v;
                                        three_exchange.third_vehicle_index = w;
                                        three_exchange.first_trip_index = k;
                                        three_exchange.second_trip_index = l;
                                        three_exchange.third_trip_index = m;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return max_savings;
}

// Function to find the best savings from shifting two trips
void diversification::shift_two_trips(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Logger& logger)
{
    // Variables for calculating the savings from trip shifts
    double max_savings = 0.0;

    // Store a vector of trip_id vectors after shifting trips
    std::vector<std::vector<int>> shifted_rotations;

    // Copy the vehicle to another vector since the first shift has to be performed to check compatibility of the second
    std::vector<Vehicle> vehicle_copy;
    std::vector<Vehicle> temp_optimal_vehicle;
    TwoShift two_shift;

    //  Perform a 2-shift
    double original_objective = evaluation::calculate_objective(vehicle, trip, terminal, logger);
    for (int v = 0; v<vehicle.size(); ++v) {
        for (int l = 1; l<vehicle[v].trip_id.size()-1; ++l) {  // l is the earlier trip served by the source vehicle
            for (int m = l+1; m<vehicle[v].trip_id.size()-1; ++m) {  // m is the later trip served by the source vehicle
                for (int u = 0; u<vehicle.size(); ++u) {
                    for (int w = 0; w<vehicle.size(); ++w) {
                        if (u==v or w==v)
                            continue;
                        if (u!=w) {
                            for (int k = 1; k<vehicle[u].trip_id.size()-1; ++k) {
                                for (int n = 1; n<vehicle[w].trip_id.size()-1; ++n) {
                                    // Check if shifts are time compatible
                                    if (evaluation::is_shift_compatible(vehicle, trip, u, v, k, l)
                                            and evaluation::is_shift_compatible(vehicle, trip, w, v, n, m)) {
                                        vehicle_copy = vehicle;
                                        two_shift.source_vehicle_index = v;
                                        two_shift.first_source_trip_index = l;
                                        two_shift.second_source_trip_index = m;

                                        two_shift.first_dest_vehicle_index = u;
                                        two_shift.first_dest_trip_index = k;
                                        two_shift.second_dest_vehicle_index = w;
                                        two_shift.second_dest_trip_index = n;

                                        diversification::perform_two_shift(vehicle_copy, two_shift, logger);

                                        // Check if rotations are charge feasible
                                        shifted_rotations.clear();
                                        shifted_rotations.push_back(vehicle_copy[u].trip_id);
                                        shifted_rotations.push_back(vehicle_copy[w].trip_id);
                                        if (v<vehicle_copy.size()) {
                                            if (vehicle_copy[v].id==vehicle[v].id)
                                                shifted_rotations.push_back(vehicle_copy[v].trip_id);
                                        }
                                        if (evaluation::are_rotations_charge_feasible(trip, terminal,
                                                shifted_rotations)) {
                                            // Calculate savings in deadheading from performing the exchange
                                            double savings = 0.0;
                                            savings = original_objective
                                                    -evaluation::calculate_objective(vehicle_copy, trip, terminal,
                                                            logger);

                                            // Check if the exchange is the best so far
                                            if (savings>max_savings) {
                                                max_savings = savings;
                                                temp_optimal_vehicle = vehicle_copy;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        else {
                            for (int k = 1; k<vehicle[u].trip_id.size()-1; ++k) {
                                for (int n = k; n<vehicle[w].trip_id.size()-1; ++n) {
                                    // Check if shifts are time compatible
                                    if (evaluation::is_shift_compatible(vehicle, trip, u, v, k, l)
                                            and evaluation::is_shift_compatible(vehicle, trip, w, v, n, m)) {
                                        vehicle_copy = vehicle;
                                        two_shift.source_vehicle_index = v;
                                        two_shift.first_source_trip_index = l;
                                        two_shift.second_source_trip_index = m;

                                        two_shift.first_dest_vehicle_index = u;
                                        two_shift.first_dest_trip_index = k;
                                        two_shift.second_dest_vehicle_index = w;
                                        two_shift.second_dest_trip_index = n;

                                        diversification::perform_two_shift(vehicle_copy, two_shift, logger);

                                        // Check if rotations are charge feasible
                                        shifted_rotations.clear();
                                        shifted_rotations.push_back(vehicle_copy[u].trip_id);
                                        shifted_rotations.push_back(vehicle_copy[w].trip_id);
                                        if (v<vehicle_copy.size()) {
                                            if (vehicle_copy[v].id==vehicle[v].id)
                                                shifted_rotations.push_back(vehicle_copy[v].trip_id);
                                        }

                                        if (evaluation::are_rotations_charge_feasible(trip, terminal,
                                                shifted_rotations)) {
                                            // Calculate savings in deadheading from performing the exchange
                                            double savings = 0.0;
                                            savings = original_objective
                                                    -evaluation::calculate_objective(vehicle_copy, trip, terminal,
                                                            logger);

                                            // Check if the exchange is the best so far
                                            if (savings>max_savings) {
                                                max_savings = savings;
                                                temp_optimal_vehicle = vehicle_copy;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // If savings are positive, perform the shift and update vehicle
    if (max_savings>0.0) {
        logger.log(LogLevel::Info, "Savings of "+std::to_string(max_savings)+" found from 2-shift...");
        vehicle = temp_optimal_vehicle;
    }
}

double diversification::shift_all_trips(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, int source_vehicle_index, Logger& logger)
{
    logger.log(LogLevel::Info, "Shifting all trips of vehicle index "+std::to_string(source_vehicle_index)+"...");
    logger.log(LogLevel::Info, "Trip IDs before shifting: "+vector_to_string(vehicle[source_vehicle_index].trip_id));

    // Store a vector of trip_id vectors after shifting trips
    Shift shift;

    std::vector<Vehicle> vehicle_copy = vehicle;
    std::vector<std::vector<int>> shifted_rotations;

    double savings, max_savings;
    double trip_removal_cost;
    bool is_shift_feasible;

    for (int l = 1; l<vehicle[source_vehicle_index].trip_id.size()-1; ++l) {
        max_savings = -INF;
        is_shift_feasible = false;
        trip_removal_cost = evaluation::calculate_trip_removal_cost(vehicle, trip, source_vehicle_index, l);
        for (int u = 0; u<vehicle.size(); ++u) {
            if (u==source_vehicle_index)
                continue;
            for (int k = 1; k<vehicle[u].trip_id.size()-1; ++k) {
                // Check if the exchanges is feasible
                if (evaluation::is_shift_compatible(vehicle, trip, u, source_vehicle_index, k, l)) {
                    // Check if exchanges are charge feasible. Insert the original trip_ids to shifted_rotations
                    shifted_rotations.clear();
                    shifted_rotations.push_back(vehicle[u].trip_id);  // This has index 0
                    shifted_rotations.push_back(vehicle[source_vehicle_index].trip_id);  // This has index 1

                    // Insert trip l of vehicle v after trip k of vehicle u in shifted_rotations
                    shifted_rotations[0].insert(shifted_rotations[0].begin()+k+1, shifted_rotations[1][l]);

                    // Remove trip l of vehicle v from shifted_rotations
                    shifted_rotations[1].erase(shifted_rotations[1].begin()+l);

                    // Check if shifted_rotations[1] has only two trips. If so, delete it
                    if (shifted_rotations[1].size()==2)
                        shifted_rotations.erase(shifted_rotations.begin()+1);

                    if (evaluation::are_rotations_charge_feasible(trip, terminal, shifted_rotations)) {
                        // Calculate savings in deadheading from performing the exchange
                        savings = evaluation::calculate_trip_addition_cost(vehicle, trip, u, source_vehicle_index, k, l)
                                +trip_removal_cost;

                        // Check if the exchange is the best so far
                        if (savings>max_savings) {
                            is_shift_feasible = true;
                            max_savings = savings;
                            shift.source_vehicle_index = source_vehicle_index;
                            shift.source_trip_index = l;
                            shift.dest_vehicle_index = u;
                            shift.dest_trip_index = k;
                        }
                    }
                }
            }
        }

        if (is_shift_feasible)
            scheduling::perform_shift(vehicle, shift, logger);
        else {
            logger.log(LogLevel::Info, "No feasible shift found for trip index "+std::to_string(l));
            return -INF;
        }
    }

    return max_savings;
}

// Function to find the best savings from shifting three trips simultaneously
void diversification::perform_three_exchange(std::vector<Vehicle>& vehicle, ThreeExchange& three_exchange,
        Logger& logger)
{
    logger.log(LogLevel::Info, "Performing three exchange...");

    int first_vehicle_index = three_exchange.first_vehicle_index;
    int second_vehicle_index = three_exchange.second_vehicle_index;
    int third_vehicle_index = three_exchange.third_vehicle_index;
    int first_trip_index = three_exchange.first_trip_index;
    int second_trip_index = three_exchange.second_trip_index;
    int third_trip_index = three_exchange.third_trip_index;

    // Log trip IDs before exchange
    logger.log(LogLevel::Info, "First vehicle trip IDs: "+vector_to_string(vehicle[first_vehicle_index].trip_id));
    logger.log(LogLevel::Info,
            "Second vehicle trip IDs: "+vector_to_string(vehicle[second_vehicle_index].trip_id));
    logger.log(LogLevel::Info,
            "Third vehicle trip IDs: "+vector_to_string(vehicle[third_vehicle_index].trip_id));

    // Log exchange information
    logger.log(LogLevel::Info, "Replacing trip index "+std::to_string(first_trip_index)+" of vehicle index "
            +std::to_string(first_vehicle_index)+" with trip index "+std::to_string(third_trip_index)
            +" of vehicle index "+std::to_string(third_vehicle_index));
    logger.log(LogLevel::Info, "Replacing trip index "+std::to_string(second_trip_index)+" of vehicle index "
            +std::to_string(second_vehicle_index)+" with trip index "+std::to_string(first_trip_index)
            +" of vehicle index "+std::to_string(first_vehicle_index));
    logger.log(LogLevel::Info, "Replacing trip index "+std::to_string(third_trip_index)+" of vehicle index "
            +std::to_string(third_vehicle_index)+" with trip index "+std::to_string(second_trip_index)
            +" of vehicle index "+std::to_string(second_vehicle_index));

    // Swap the trips
    int temp_trip_id_first = vehicle[first_vehicle_index].trip_id[first_trip_index];
    int temp_trip_id_second = vehicle[second_vehicle_index].trip_id[second_trip_index];
    int temp_trip_id_third = vehicle[third_vehicle_index].trip_id[third_trip_index];

    vehicle[first_vehicle_index].trip_id[first_trip_index] = temp_trip_id_third;
    vehicle[second_vehicle_index].trip_id[second_trip_index] = temp_trip_id_first;
    vehicle[third_vehicle_index].trip_id[third_trip_index] = temp_trip_id_second;

    // Log trip IDs after exchange for both first vehicle and second vehicle
    logger.log(LogLevel::Info,
            "First vehicle new trip IDs: "+vector_to_string(vehicle[first_vehicle_index].trip_id));
    logger.log(LogLevel::Info,
            "Second vehicle new trip IDs: "+vector_to_string(vehicle[second_vehicle_index].trip_id));
    logger.log(LogLevel::Info,
            "Third vehicle new trip IDs: "+vector_to_string(vehicle[third_vehicle_index].trip_id));
}

// Function to perform a simultaneous shift of two trips
void diversification::perform_two_shift(std::vector<Vehicle>& vehicle, TwoShift& two_shift, Logger& logger)
{
    logger.log(LogLevel::Debug, "Performing 2-shift...");

    // Insert trip l of vehicle v after trip k of vehicle u and trip m of vehicle v after trip n of vehicle w
    int source_vehicle_index = two_shift.source_vehicle_index;
    int first_source_trip_index = two_shift.first_source_trip_index;
    int second_source_trip_index = two_shift.second_source_trip_index;

    int first_dest_vehicle_index = two_shift.first_dest_vehicle_index;
    int first_dest_trip_index = two_shift.first_dest_trip_index;
    int second_dest_vehicle_index = two_shift.second_dest_vehicle_index;
    int second_dest_trip_index = two_shift.second_dest_trip_index;

    // Log trip IDs before shift
    logger.log(LogLevel::Debug,
            "Source vehicle trip IDs: "+vector_to_string(vehicle[source_vehicle_index].trip_id));
    logger.log(LogLevel::Debug,
            "First destination vehicle trip IDs: "+vector_to_string(vehicle[first_dest_vehicle_index].trip_id));
    logger.log(LogLevel::Debug,
            "Second destination vehicle trip IDs: "+vector_to_string(vehicle[second_dest_vehicle_index].trip_id));

    // Log shift information
    logger.log(LogLevel::Debug, "Shifting trip index "+std::to_string(first_source_trip_index)+" of vehicle index "
            +std::to_string(source_vehicle_index)+" after trip index "+std::to_string(first_dest_trip_index)
            +" of vehicle index "+std::to_string(first_dest_vehicle_index)+"...");
    logger.log(LogLevel::Debug, "Shifting trip index "+std::to_string(second_source_trip_index)+" of vehicle index "
            +std::to_string(source_vehicle_index)+" after trip index "+std::to_string(second_dest_trip_index)
            +" of vehicle index "+std::to_string(second_dest_vehicle_index)+"...");

    // Insert the trips
    vehicle[first_dest_vehicle_index].trip_id.insert(
            vehicle[first_dest_vehicle_index].trip_id.begin()+first_dest_trip_index+1,
            vehicle[source_vehicle_index].trip_id[first_source_trip_index]);

    vehicle[second_dest_vehicle_index].trip_id.insert(
            vehicle[second_dest_vehicle_index].trip_id.begin()+second_dest_trip_index+1,
            vehicle[source_vehicle_index].trip_id[second_source_trip_index]);

    // Remove the trips with indices first_source_trip_index and second_source_trip_index from the source vehicle
    vehicle[source_vehicle_index].trip_id.erase(
            vehicle[source_vehicle_index].trip_id.begin()+second_source_trip_index);
    vehicle[source_vehicle_index].trip_id.erase(  // Removing second trip first will not change the index of first
            vehicle[source_vehicle_index].trip_id.begin()+first_source_trip_index);

    // If the source vehicle has only two trips, remove the vehicle
    if (vehicle[source_vehicle_index].trip_id.size()==2) {
        logger.log(LogLevel::Info, "Source vehicle has only two trips. Removing it...");
        vehicle.erase(vehicle.begin()+source_vehicle_index);
    }
    else
        logger.log(LogLevel::Debug,
                "Source vehicle new trip IDs: "+vector_to_string(vehicle[source_vehicle_index].trip_id));

    // Log the rotations after the shift for the destination vehicle
    logger.log(LogLevel::Debug,
            "First destination vehicle new trip new IDs: "
                    +vector_to_string(vehicle[first_dest_vehicle_index].trip_id));
    logger.log(LogLevel::Debug,
            "Second destination vehicle new trip new IDs: "
                    +vector_to_string(vehicle[second_dest_vehicle_index].trip_id));
}

// Operators that diversify exchanges and shifts
void diversification::apply_operators(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Logger& logger)
{
    if (PERFORM_TWO_SHIFTS) {
        //logger.log(LogLevel::Info, "Checking for improvement from two shifts...");
        //shift_two_trips(vehicle, trip, terminal, logger);  // TODO: Add a flag for this

        // Sequentially shift all trips of vehicles which fewer trips to vehicles which have more trips
        logger.log(LogLevel::Info, "Shifting all trips of vehicles with few trips...");

        //Find vehicle indices which ave less than or equal to six trips
        std::vector<int> vehicle_indices;
        for (int v = 0; v<vehicle.size(); ++v)
            if (vehicle[v].trip_id.size()<=SHIFT_ALL_TRIPS_THRESHOLD)
                vehicle_indices.push_back(v);
        logger.log(LogLevel::Info, "Vehicle indices with few trips: "+vector_to_string(vehicle_indices));

        double savings;
        double max_savings = 0.0;
        int optimal_vehicle_index;
        std::vector<Vehicle> vehicle_copy = vehicle;
        for (int i = 0; i<vehicle_indices.size(); ++i) {
            int v = vehicle_indices[i];
            vehicle_copy = vehicle;
            savings = shift_all_trips(vehicle_copy, trip, terminal, v, logger);
            if (savings>max_savings) {
                max_savings = savings;
                optimal_vehicle_index = vehicle_indices[i];
            }
        }
        logger.log(LogLevel::Info, "Max savings from shifting all trips of a vehicle: "+std::to_string(max_savings));

        // Perform the shift for the optimal index
        if (max_savings>EPSILON) {
            logger.log(LogLevel::Info, "Performing shift for vehicle index "+std::to_string(optimal_vehicle_index));
            shift_all_trips(vehicle, trip, terminal, optimal_vehicle_index, logger);
        }
        else
            logger.log(LogLevel::Info, "No improvement possible from shifting all trips of a vehicle...");
    }

    if (PERFORM_THREE_EXCHANGES) {
        logger.log(LogLevel::Info, "Checking for improvement from three exchanges...");

        ThreeExchange three_exchange;
        double three_exchange_savings = exchange_three_trips(vehicle, trip, terminal, three_exchange);

        logger.log(LogLevel::Info, "Savings from three exchange operator: "+std::to_string(three_exchange_savings));
        (three_exchange_savings>EPSILON) ? perform_three_exchange(vehicle, three_exchange, logger) : logger.log(
                LogLevel::Info, "No improvement possible from three exchanges...");
    }
}

// Function to optimize rotations using repeated shifts and exchanges till there is no improvement
void diversification::optimize_rotations(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Logger& logger)
{
    logger.log(LogLevel::Info, "Optimizing rotations using diversification operators...");

    // Initialize variables
    double old_objective = INF;
    double new_objective = evaluation::calculate_objective(vehicle, trip, terminal, logger);

    // Loop until there is no improvement
    while (new_objective<old_objective) {
        old_objective = new_objective;
        scheduling::apply_best_improvement(vehicle, trip, terminal, logger);
        diversification::apply_operators(vehicle, trip, terminal, logger);
        new_objective = evaluation::calculate_objective(vehicle, trip, terminal, logger);
    }
}

/* EVALUATION FUNCTIONS
 * This section contains functions that evaluate various aspects of a solution/rotation.
 * These include the objective function, the time spent at charging terminals, etc.
 * They also have functions to check if local search leads to solutions that are time compatible and charge feasible.
 * Additional functions track changes in the objective due to local searches without actually performing them. */

// Calculate the objective that includes the cost of deadheading and opening charging stations
double evaluation::calculate_objective(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Logger& logger)
{
    // Calculate the objective value of the initial solution
    logger.log(LogLevel::Debug, "Calculating the objective value of the solution...");

    // Calculate fixed costs of opening charging stations
    // TODO: This can be calculated faster if we keep track of the stations. Do we need to? Only if this is called often.
    double location_cost = 0.0;
    for (const auto& curr_terminal : terminal)
        location_cost += (curr_terminal.is_charge_station) ? CHARGE_LOC_COST : 0;

    // Calculate fixed cost of vehicle acquisition based on the number of vehicles
    double vehicle_acquisition_cost = VEHICLE_COST*vehicle.size();

    // Calculate variable cost of deadheading
    double deadhead_cost = 0.0;
    for (auto& curr_vehicle : vehicle) {
        curr_vehicle.calculate_deadhead_cost(trip);
        // Log the deadheading cost of each trip
        logger.log(LogLevel::Verbose, "Deadheading cost of vehicle "+std::to_string(curr_vehicle.id)+": "
                +std::to_string(curr_vehicle.deadhead_cost));
        deadhead_cost += curr_vehicle.deadhead_cost;
    }

    // Calculate CSP cost
    double csp_cost = csp::select_optimization_model(vehicle, trip, terminal, logger);

    // Calculate the total cost and log the cost components
    double total_cost = location_cost+vehicle_acquisition_cost+deadhead_cost+csp_cost;
    logger.log(LogLevel::Debug, "Fixed cost of opening charging stations: "+std::to_string(location_cost));
    logger.log(LogLevel::Debug, "Fixed cost of vehicle acquisition: "+std::to_string(vehicle_acquisition_cost));
    logger.log(LogLevel::Debug, "Total cost of deadheading: "+std::to_string(deadhead_cost));
    logger.log(LogLevel::Debug, "CSP cost: "+std::to_string(csp_cost));
    logger.log(LogLevel::Debug, "Number of charging stations: "+std::to_string(int(location_cost/CHARGE_LOC_COST)));
    logger.log(LogLevel::Debug, "Number of vehicles used: "+std::to_string(vehicle.size()));
    logger.log(LogLevel::Info, "Total cost: "+std::to_string(total_cost));

    return total_cost;
}

// Function that calculates the total deadheading cost across all vehicle rotations
double evaluation::calculate_deadheading_cost(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip)
{
    double total_deadheading_cost = 0.0;
    for (auto& curr_vehicle : vehicle) {
        curr_vehicle.calculate_deadhead_cost(trip);
        total_deadheading_cost += curr_vehicle.deadhead_cost;
    }
    return total_deadheading_cost;
}

// Function that calculates the total deadheading cost across a subset of vehicle rotations
double evaluation::calculate_deadheading_cost(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<int>& vehicle_indices)
{
    double total_deadheading_cost = 0.0;
    for (int i = 0; i<vehicle_indices.size(); ++i) {
        int v = vehicle_indices[i];
        vehicle[v].calculate_deadhead_cost(trip);
        total_deadheading_cost += vehicle[v].deadhead_cost;
    }
    return total_deadheading_cost;
}  // TODO: Check if we want this version at a rotation basis without reference to the full vehicle class

// Determine the time spent by vehicles at charging terminals along their routes in the current station configuration
void evaluation::calculate_utilization(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Logger& logger)
{
    // Estimate the amount of idle time available at each charging and non-charging terminals across all vehicles
    for (auto& curr_terminal : terminal) {
        curr_terminal.current_idle_time = 0;
        curr_terminal.potential_idle_time = 0;
    }

    int curr_trip, next_trip;  // Current trip and next trip IDs
    int end_terminal_curr_trip, start_terminal_next_trip; // End terminal of current trip and start terminal of the next trip
    bool is_curr_trip_end_charge_terminal, is_next_trip_start_charge_terminal;
    for (auto& curr_vehicle : vehicle) {
        for (int i = 1; i<curr_vehicle.trip_id.size()-2; ++i) {
            curr_trip = curr_vehicle.trip_id[i];
            next_trip = curr_vehicle.trip_id[i+1];

            end_terminal_curr_trip = trip[curr_trip-1].end_terminal;  // End terminal id of current trip
            start_terminal_next_trip = trip[next_trip-1].start_terminal;  // Start terminal id of next trip

            is_curr_trip_end_charge_terminal = terminal[end_terminal_curr_trip-1].is_charge_station;
            is_next_trip_start_charge_terminal = terminal[start_terminal_next_trip-1].is_charge_station;

            // n: no charging, e: end terminal, s: start terminal; Charging is preferred at the end terminal
            char scenario = 'n';
            if (is_curr_trip_end_charge_terminal)
                scenario = 'e';
            else if (is_next_trip_start_charge_terminal)
                scenario = 's';

            switch (scenario) {
            case 'e':terminal[end_terminal_curr_trip-1].current_idle_time += trip[curr_trip-1].idle_time[next_trip-1];
                break;
            case 's':  // If charging station is opened only at the start terminal of the next trip
                terminal[start_terminal_next_trip-1].current_idle_time += trip[curr_trip-1].idle_time[next_trip-1];
                terminal[end_terminal_curr_trip-1].potential_idle_time += trip[curr_trip-1].idle_time[next_trip-1];
                break;
            default:  // No charging location is available at either location
                terminal[start_terminal_next_trip-1].potential_idle_time += trip[curr_trip-1].idle_time[next_trip
                        -1];
                terminal[end_terminal_curr_trip-1].potential_idle_time += trip[curr_trip-1].idle_time[next_trip-1];
            }
        }
    }

    // Log utilization statistics of all terminals
    logger.log(LogLevel::Info,
            "Utilization statistics: (Terminal ID: Is charge station, Current idle time, Potential idle time)");
    for (const auto& curr_terminal : terminal) {
        logger.log(LogLevel::Info,
                "Terminal "+std::to_string(curr_terminal.id)+": "+std::to_string(curr_terminal.is_charge_station)
                        +" "
                        +std::to_string(curr_terminal.current_idle_time)+" "
                        +std::to_string(curr_terminal.potential_idle_time));
    }
}

// Checks if exchanging two trips across two different vehicle rotations is compatible
bool evaluation::is_two_exchange_compatible(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        int first_vehicle_index, int second_vehicle_index, int first_vehicle_trip_index,
        int second_vehicle_trip_index)
{
    int first_vehicle_prev_trip_id = vehicle[first_vehicle_index].trip_id[first_vehicle_trip_index-1];
    int first_vehicle_curr_trip_id = vehicle[first_vehicle_index].trip_id[first_vehicle_trip_index];
    int first_vehicle_next_trip_id = vehicle[first_vehicle_index].trip_id[first_vehicle_trip_index+1];

    int second_vehicle_prev_trip_id = vehicle[second_vehicle_index].trip_id[second_vehicle_trip_index-1];
    int second_vehicle_curr_trip_id = vehicle[second_vehicle_index].trip_id[second_vehicle_trip_index];
    int second_vehicle_next_trip_id = vehicle[second_vehicle_index].trip_id[second_vehicle_trip_index+1];

    // Check if the exchange is feasible using compatibility matrices
    if (trip[first_vehicle_prev_trip_id-1].is_compatible[second_vehicle_curr_trip_id-1]
            and trip[second_vehicle_curr_trip_id-1].is_compatible[first_vehicle_next_trip_id-1]
            and trip[second_vehicle_prev_trip_id-1].is_compatible[first_vehicle_curr_trip_id-1]
            and trip[first_vehicle_curr_trip_id-1].is_compatible[second_vehicle_next_trip_id-1])
        return true;
    else
        return false;
}

// Check compatibility of exchanging three trips
bool evaluation::is_three_exchange_compatible(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        int first_vehicle_index, int second_vehicle_index, int third_vehicle_index, int first_vehicle_trip_index,
        int second_vehicle_trip_index, int third_vehicle_trip_index)
{
    int first_vehicle_prev_trip_id = vehicle[first_vehicle_index].trip_id[first_vehicle_trip_index-1];
    int first_vehicle_curr_trip_id = vehicle[first_vehicle_index].trip_id[first_vehicle_trip_index];
    int first_vehicle_next_trip_id = vehicle[first_vehicle_index].trip_id[first_vehicle_trip_index+1];

    int second_vehicle_prev_trip_id = vehicle[second_vehicle_index].trip_id[second_vehicle_trip_index-1];
    int second_vehicle_curr_trip_id = vehicle[second_vehicle_index].trip_id[second_vehicle_trip_index];
    int second_vehicle_next_trip_id = vehicle[second_vehicle_index].trip_id[second_vehicle_trip_index+1];

    int third_vehicle_prev_trip_id = vehicle[third_vehicle_index].trip_id[third_vehicle_trip_index-1];
    int third_vehicle_curr_trip_id = vehicle[third_vehicle_index].trip_id[third_vehicle_trip_index];
    int third_vehicle_next_trip_id = vehicle[third_vehicle_index].trip_id[third_vehicle_trip_index+1];

    // Check if the exchange is feasible using compatibility matrices
    if (trip[first_vehicle_prev_trip_id-1].is_compatible[third_vehicle_curr_trip_id-1]
            and trip[third_vehicle_curr_trip_id-1].is_compatible[first_vehicle_next_trip_id-1]
            and trip[second_vehicle_prev_trip_id-1].is_compatible[first_vehicle_curr_trip_id-1]
            and trip[first_vehicle_curr_trip_id-1].is_compatible[second_vehicle_next_trip_id-1]
            and trip[third_vehicle_prev_trip_id-1].is_compatible[second_vehicle_curr_trip_id-1]
            and trip[second_vehicle_curr_trip_id-1].is_compatible[third_vehicle_next_trip_id-1])
        return true;
    else
        return false;
}

// Checks if shifting a trip from one vehicle rotation to another is compatible
bool evaluation::is_shift_compatible(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        int dest_vehicle_index, int source_vehicle_index, int dest_vehicle_trip_index,
        int source_vehicle_trip_index)
{
    int dest_vehicle_curr_trip_id = vehicle[dest_vehicle_index].trip_id[dest_vehicle_trip_index];
    int dest_vehicle_next_trip_id = vehicle[dest_vehicle_index].trip_id[dest_vehicle_trip_index+1];

    int source_vehicle_trip_id = vehicle[source_vehicle_index].trip_id[source_vehicle_trip_index];  // The trip being shifted

    // Check if the shift is feasible using compatibility matrices
    if (trip[dest_vehicle_curr_trip_id-1].is_compatible[source_vehicle_trip_id-1]
            and trip[source_vehicle_trip_id-1].is_compatible[dest_vehicle_next_trip_id-1])
        return true;
    else
        return false;
}

// Function to check feasibility and update the charge level at the end terminal of the next trip
// given the charge level at the end terminal of the current trip
bool evaluation::is_charge_adequate_next_trip(std::vector<Trip>& trip, int curr_trip, int next_trip,
        bool is_curr_trip_end_charge_terminal, bool is_next_trip_start_charge_terminal, int charge_time_window,
        double& charge_level)
{
    // n: no charging, e: end terminal, s: start terminal; Charging is preferred at the end terminal
    char scenario = 'n';
    if (is_curr_trip_end_charge_terminal)
        scenario = 'e';
    else if (is_next_trip_start_charge_terminal)
        scenario = 's';

    switch (scenario) {
    case 'e':charge_level = std::min(charge_level+(charge_time_window*MAX_ENERGY_PER_MIN), MAX_CHARGE_LEVEL);
        charge_level -= trip[curr_trip-1].deadhead_distance[next_trip-1]*ENERGY_PER_KM;
        if (charge_level < MIN_CHARGE_LEVEL)
            return false;
        break;
    case 's':charge_level -= trip[curr_trip-1].deadhead_distance[next_trip-1]*ENERGY_PER_KM;
        if (charge_level < MIN_CHARGE_LEVEL)
            return false;
        charge_level = std::min(charge_level+(charge_time_window*MAX_ENERGY_PER_MIN), MAX_CHARGE_LEVEL);
        break;
    default:  // No charging location is available at either location
        charge_level -= trip[curr_trip-1].deadhead_distance[next_trip-1]*ENERGY_PER_KM;
        if (charge_level < MIN_CHARGE_LEVEL)
            return false;
    }
    charge_level -= trip[next_trip-1].distance*ENERGY_PER_KM;

    return (charge_level >= MIN_CHARGE_LEVEL);
}

// Function to check if the new trip sequences are charge feasible
bool evaluation::are_rotations_charge_feasible(std::vector<Trip>& trip, std::vector<Terminal>& terminal,
        std::vector<std::vector<int>> rotations)
{
    int curr_trip, next_trip;  // Current trip and next trip IDs
    int end_terminal_curr_trip, start_terminal_next_trip; // End terminal of current trip and start terminal of the next trip
    bool is_curr_trip_end_charge_terminal, is_next_trip_start_charge_terminal;
    int charge_time_window;  // Idle time during which charging is allowed

    // Iterate across rotations
    for (auto curr_rotation : rotations) {
        // Calculate the energy required for deadheading from the depot to the end of the first trip
        double charge_level = MAX_CHARGE_LEVEL-(trip[curr_rotation[0]-1].deadhead_distance[curr_rotation[1]-1]
                +trip[curr_rotation[1]-1].distance)*ENERGY_PER_KM;

        // Check if energy level is above the minimum threshold, if so, return false and break
        if (charge_level<MIN_CHARGE_LEVEL)
            return false;

        // Iterate across trips in the rotations
        for (int i = 1; i<curr_rotation.size()-2; ++i) {
            curr_trip = curr_rotation[i];
            next_trip = curr_rotation[i+1];

            end_terminal_curr_trip = trip[curr_trip-1].end_terminal;  // End terminal id of current trip
            start_terminal_next_trip = trip[next_trip-1].start_terminal;  // Start terminal id of next trip

            is_curr_trip_end_charge_terminal = terminal[end_terminal_curr_trip-1].is_charge_station;
            is_next_trip_start_charge_terminal = terminal[start_terminal_next_trip-1].is_charge_station;

            charge_time_window = trip[curr_trip-1].idle_time[next_trip-1];

            // Update the charge level at the end terminal of the next trip.
            // If the charge level is below the minimum threshold, return false and break
            if (not is_charge_adequate_next_trip(trip, curr_trip, next_trip, is_curr_trip_end_charge_terminal,
                    is_next_trip_start_charge_terminal, charge_time_window, charge_level))
                return false;
        }

        // Add distance from the last actual trip to the depot to cumulative_energy
        int penultimate_trip = curr_rotation[curr_rotation.size()-2];
        int last_trip = curr_rotation[curr_rotation.size()-1];
        charge_level -= trip[penultimate_trip-1].deadhead_distance[last_trip-1]*ENERGY_PER_KM;
        if (charge_level<MIN_CHARGE_LEVEL)
            return false;
    }
    return true;
}

// Calculate cost changes due to replacing a trip of a vehicle rotation with a trip from another rotation
double evaluation::calculate_trip_replacement_cost(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        int first_vehicle_index, int second_vehicle_index, int first_vehicle_trip_index,
        int second_vehicle_trip_index)
{
    // Trip first_vehicle_trip_index on vehicle rotation first_vehicle_index is being swapped with trip second_vehicle_trip_index from vehicle rotation second_vehicle_index
    // The function calculates the changes in the deadheading costs
    int prev_trip_id = vehicle[first_vehicle_index].trip_id[first_vehicle_trip_index-1];
    int curr_trip_id = vehicle[first_vehicle_index].trip_id[first_vehicle_trip_index];
    int next_trip_id = vehicle[first_vehicle_index].trip_id[first_vehicle_trip_index+1];

    int new_trip_id = vehicle[second_vehicle_index].trip_id[second_vehicle_trip_index];

    double curr_cost = (trip[prev_trip_id-1].deadhead_distance[curr_trip_id-1]
            +trip[curr_trip_id-1].deadhead_distance[next_trip_id-1])*COST_PER_KM;

    double new_cost = (trip[prev_trip_id-1].deadhead_distance[new_trip_id-1]
            +trip[new_trip_id-1].deadhead_distance[next_trip_id-1])*COST_PER_KM;

    return curr_cost
            -new_cost;  // If current cost is higher, the savings are positive and the exchange is beneficial
}

// Calculate the cost changes from replacing a depot with another depot at the end of a trip
// Start depots are not exchanged because they were found from nearest neighbors
double evaluation::calculate_depot_replacement_cost(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        int first_vehicle_index, int second_vehicle_index, int first_vehicle_trip_index,
        int second_vehicle_trip_index)
{
    int penultimate_trip = vehicle[first_vehicle_index].trip_id[first_vehicle_trip_index-1];
    int old_depot = vehicle[first_vehicle_index].trip_id[first_vehicle_trip_index];
    int new_depot = vehicle[second_vehicle_index].trip_id[second_vehicle_trip_index];

    double curr_cost = (trip[penultimate_trip-1].deadhead_distance[old_depot-1])*COST_PER_KM;
    double new_cost = (trip[penultimate_trip-1].deadhead_distance[new_depot-1])*COST_PER_KM;

    return curr_cost
            -new_cost;  // If current cost is higher, the savings are positive and the exchange is beneficial
}

// Calculate cost changes due to addition of a new trip after an existing trip
double evaluation::calculate_trip_addition_cost(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        int dest_vehicle_index, int source_vehicle_index, int dest_vehicle_trip_index, int source_trip_index)
{
    // Trip source_trip_index is being added after trip dest_vehicle_trip_index on vehicle rotation dest_vehicle_index
    // The function calculates the changes in the deadheading costs
    int curr_trip_id = vehicle[dest_vehicle_index].trip_id[dest_vehicle_trip_index];
    int next_trip_id = vehicle[dest_vehicle_index].trip_id[dest_vehicle_trip_index+1];

    int new_trip_id = vehicle[source_vehicle_index].trip_id[source_trip_index];

    double curr_cost = trip[curr_trip_id-1].deadhead_distance[next_trip_id-1]*COST_PER_KM;

    double new_cost = (trip[curr_trip_id-1].deadhead_distance[new_trip_id-1]
            +trip[new_trip_id-1].deadhead_distance[next_trip_id-1])*COST_PER_KM;

    return curr_cost-new_cost;  // If current cost is higher, the savings are positive and the shift is beneficial
}

// Calculate cost changes due to removal of an existing trip
double evaluation::calculate_trip_removal_cost(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        int source_vehicle_index, int source_vehicle_trip_index)
{
    // Trip source_vehicle_trip_index is being removed from vehicle rotation source_vehicle_index
    // The function calculates the changes in the deadheading costs
    int prev_trip_id = vehicle[source_vehicle_index].trip_id[source_vehicle_trip_index-1];
    int curr_trip_id = vehicle[source_vehicle_index].trip_id[source_vehicle_trip_index];
    int next_trip_id = vehicle[source_vehicle_index].trip_id[source_vehicle_trip_index+1];

    double curr_cost = (trip[prev_trip_id-1].deadhead_distance[curr_trip_id-1]+
            +trip[curr_trip_id-1].deadhead_distance[next_trip_id-1])*COST_PER_KM;

    double new_cost = trip[prev_trip_id-1].deadhead_distance[next_trip_id-1]*COST_PER_KM;

    // Check if the size of trips in the source vehicle is 3 (two depots and one trip)
    // If so, add the cost of the vehicle to the savings
    if (vehicle[source_vehicle_index].trip_id.size()==3)
        new_cost -= VEHICLE_COST;

    return curr_cost-new_cost;  // If current cost is higher, the savings are positive and the shift is beneficial
}


