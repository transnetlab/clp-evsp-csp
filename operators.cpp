#include "operators.h"

// Function to find the best savings from exchanging trips
double scheduling::exchange_trips(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Exchange& exchange)
{
    // Find a pair of vehicle rotation as a set from vehicles
    double max_savings = 0.0;  // Stores the maximum savings among all exchanges

    // Store a vector of trip_id vectors after swapping trips
    std::vector<std::vector<int>> swapped_rotations;

    //  Exchange trips k and l of vehicles u and v
    for (int u = 0; u<vehicle.size(); ++u) {
        for (int v = u+1; v<vehicle.size(); ++v) {
            for (int k = 1; k<vehicle[u].trip_id.size()-1; ++k) {
                for (int l = 1; l<vehicle[v].trip_id.size()-1; ++l) {
                    double savings = 0.0;
                    // Check if the exchanges is time compatible
                    if (evaluation::is_exchange_compatible(vehicle, trip, u, v, k, l)) {
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

                            // Check if the exchange is the best so far
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
        Shift& shift)
{
    // Variables for calculating the savings from trip shifts
    double max_savings = 0.0;

    // Store a vector of trip_id vectors after shifting trips
    std::vector<std::vector<int>> shifted_rotations;

    //  Insert trip l of vehicle v after trip k of vehicle u
    for (int u = 0; u<vehicle.size(); ++u) {
        for (int v = 0; v<vehicle.size(); ++v) {
            for (int k = 1; k<vehicle[u].trip_id.size()-1; ++k) {
                for (int l = 1; l<vehicle[v].trip_id.size()-1; ++l) {
                    double savings = 0.0;
                    // Check if the exchanges is feasible
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

                            // Check if the exchange is the best so far
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
    return max_savings;
}

// Function to exchange the depot trips of two vehicles
double scheduling::exchange_depots(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Exchange& exchange)
{
    // Find a pair of vehicle rotation as a set from vehicles
    double max_savings = 0.0;  // Stores the maximum savings among all exchanges

    // Store a vector of trip_id vectors after swapping trips
    std::vector<std::vector<int>> swapped_rotations;

    //  Exchange the last trips of vehicles u and v. Depot trips are always compatible.
    for (int u = 0; u<vehicle.size(); ++u) {
        for (int v = u+1; v<vehicle.size(); ++v) {
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

                // Check if the exchange is the best so far
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
    logger.log(LogLevel::Debug, "First vehicle new trip IDs: "+vector_to_string(vehicle[first_vehicle_index].trip_id));
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
void scheduling::optimize_rotations(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Logger& logger)
{
    logger.log(LogLevel::Info, "Optimizing scheduling...");

    Exchange exchange;
    Shift shift;

    // Run the exchange and shift locations
    double exchange_savings = exchange_trips(vehicle, trip, terminal, exchange);
    double shift_savings = shift_trips(vehicle, trip, terminal, shift);

    // Check if the exchanges operator is better than the shifts operator
    logger.log(LogLevel::Info, "Savings from exchange operator: "+std::to_string(exchange_savings));
    logger.log(LogLevel::Info, "Savings from shifts operator: "+std::to_string(shift_savings));

    // Check if savings are positive
    if (exchange_savings<EPSILON and shift_savings<EPSILON) {
        logger.log(LogLevel::Info, "No improvement possible from trip exchanges or shifts...");

        if (PERFORM_DEPOT_EXCHANGES) {  // Check if additional savings can be obtained by exchanging depots
            logger.log(LogLevel::Info, "Checking for improvement from depot exchanges...");
            double depot_exchange_savings = exchange_depots(vehicle, trip, terminal, exchange);
            std::cout << depot_exchange_savings << std::endl;
            logger.log(LogLevel::Info, "Savings from depot exchange operator: "+std::to_string(depot_exchange_savings));
            if (depot_exchange_savings>EPSILON)
                perform_exchange(vehicle, exchange, logger);
            else
                logger.log(LogLevel::Info, "No improvement possible from depot exchanges...");
            return;
        }
        else
            return;
    }

    if (exchange_savings>shift_savings)
        perform_exchange(vehicle, exchange, logger);
    else
        perform_shift(vehicle, shift, logger);
}

// Function that closes charging stations and creates new rotations if requried
void locations::close_charging_station(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, int close_terminal_id, Logger& logger)
{
    logger.log(LogLevel::Info, "Checking for improvement from closing terminal ID "+std::to_string(close_terminal_id));

    // Check if savings can be achieved from applying the scheduling locations
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

            if (trip[curr_trip-1].end_terminal==close_terminal_id
                    or (trip[next_trip-1].start_terminal==close_terminal_id and not is_curr_trip_end_charge_terminal)) {
                scan_eligible_vehicle_indices.push_back(v);
                break;
            }
        }
    }
    logger.log(LogLevel::Info, "Vehicle indices affected: "+vector_to_string(scan_eligible_vehicle_indices));

    terminal[close_terminal_id-1].is_charge_station = false;  // Close the terminal with the chosen index
    std::vector<Vehicle> vehicle_copy = vehicle;  // Create a copy of the vehicle vector to check for savings

    // Check if rotations are feasible after deletion of the terminal. If not, create new rotations.
    if (not evaluation::check_charge_feasibility_and_split_rotations(vehicle_copy, trip, terminal,
            scan_eligible_vehicle_indices, logger)) {
        logger.log(LogLevel::Info, "Closing station makes the problem infeasible. Reverting changes...");
        terminal[close_terminal_id-1].is_charge_station = true;
        return;
    }

    // Check if savings can be achieved from applying the scheduling operators
    logger.log(LogLevel::Info, "After closing the charging station and creating new rotations (if any)...");
    double curr_objective = evaluation::calculate_objective(vehicle_copy, trip, terminal, logger);
    double old_objective = INF;

    logger.log(LogLevel::Info, "Applying local search locations to adjust scheduling...");
    while (curr_objective<old_objective) {
        old_objective = curr_objective;
        scheduling::optimize_rotations(vehicle_copy, trip, terminal, logger);
        curr_objective = evaluation::calculate_objective(vehicle_copy, trip, terminal, logger);
    }

    // Check if the current objective is better than the best objective that we started with
    if (curr_objective<best_objective) {
        logger.log(LogLevel::Info, "Objective improved by "+std::to_string(best_objective-curr_objective));
        vehicle = vehicle_copy;
    }
    else {
        logger.log(LogLevel::Info, "No improvement from closing the charging station. Reverting changes...");
        terminal[close_terminal_id-1].is_charge_station = true;
    }
}

// Function to open charging stations and check if savings can be achieved from scheduling operators
void locations::open_charging_station(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, int open_terminal_id, Logger& logger)
{
    logger.log(LogLevel::Info, "Checking for improvement from opening terminal ID "+std::to_string(open_terminal_id));

    // Check if savings can be achieved from applying the scheduling operators
    logger.log(LogLevel::Info, "Before opening the charging station...");
    double best_objective = evaluation::calculate_objective(vehicle, trip, terminal, logger);

    // Opening charging station will increase the cost. Check if savings can be achieved from scheduling operators
    logger.log(LogLevel::Info, "After opening the charging station...");
    terminal[open_terminal_id-1].is_charge_station = true;  // Open the terminal with the chosen index
    double curr_objective = evaluation::calculate_objective(vehicle, trip, terminal, logger);
    double old_objective = INF;

    logger.log(LogLevel::Info, "Applying local search operators to adjust scheduling...");
    std::vector<Vehicle> vehicle_copy = vehicle;  // Create a copy of the vehicle vector to check for savings
    while (curr_objective<old_objective) {
        old_objective = curr_objective;
        scheduling::optimize_rotations(vehicle_copy, trip, terminal, logger);
        curr_objective = evaluation::calculate_objective(vehicle_copy, trip, terminal, logger);
    }

    // Check if the current objective is better than the best objective that we started with
    if (curr_objective<best_objective) {
        logger.log(LogLevel::Info, "Objective improved by "+std::to_string(best_objective-curr_objective));
        vehicle = vehicle_copy;
    }
    else {
        logger.log(LogLevel::Info, "No improvement from opening the charging station. Reverting changes...");
        terminal[open_terminal_id-1].is_charge_station = false;
    }
}

// Function to open or close locations. This could create new rotations as well when locations are closed
void locations::optimize_stations(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Logger& logger)
{
    logger.log(LogLevel::Info, "Optimizing locations...");

    // Step 1: Close open charging stations with zero utilization
    evaluation::calculate_utilization(vehicle, trip, terminal, logger);
    logger.log(LogLevel::Info, "Checking if charge stations with zero utilization can be closed...");
    int num_zero_utilization_terminals = 0;
    for (auto& curr_terminal : terminal) {
        if (curr_terminal.is_charge_station and curr_terminal.current_idle_time==0) {
            curr_terminal.is_charge_station = false;
            ++num_zero_utilization_terminals;
        }
    }
    logger.log(LogLevel::Info, "Number of charge stations closed: "+std::to_string(num_zero_utilization_terminals));
    evaluation::calculate_objective(vehicle, trip, terminal, logger);

    // Step 2: Open charging stations
    // Open a closed charging station with the highest utilization. Calculate savings considering scheduling.
    evaluation::calculate_utilization(vehicle, trip, terminal, logger);
    std::vector<int> open_terminal_ids;
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
    std::vector<int> close_terminal_ids;
    for (const auto& curr_terminal : terminal)
        if (curr_terminal.is_charge_station)
            close_terminal_ids.push_back(curr_terminal.id);

    // Sort the terminals that are open in ascending order of utilization
    std::sort(close_terminal_ids.begin(), close_terminal_ids.end(),
            [&terminal](int a, int b) { return terminal[a-1].current_idle_time<terminal[b-1].current_idle_time; });

    // Print the sorted terminal IDs
    logger.log(LogLevel::Info, "Sorted terminals to be closed: "+vector_to_string(close_terminal_ids));

    // Loop through the sorted terminal ID list and check if closing them leads to an improvement in the objective
    for (const auto& close_terminal_id : close_terminal_ids) {
        locations::close_charging_station(vehicle, trip, terminal, close_terminal_id, logger);
    }
}

// Create new rotations if a rotation is not charge feasible
void locations::split_trips(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip, std::vector<Terminal>& terminal,
        int vehicle_index, int trip_in_rotation_index, std::vector<int>& scan_eligible_vehicle_indices, Logger& logger)
{
    // Log operations
    logger.log(LogLevel::Info,
            "Splitting rotation after trip ID "+std::to_string(vehicle[vehicle_index].trip_id[trip_in_rotation_index])
                    +" of vehicle index "+std::to_string(vehicle_index)+"...");

    // Log trip IDs before splitting
    logger.log(LogLevel::Info, "Trip IDs before splitting: "+vector_to_string(vehicle[vehicle_index].trip_id));

    // Create a new vehicle with the remaining trips
    Vehicle new_vehicle;

    new_vehicle.id = vehicle[vehicle.size()-1].id+1;
    new_vehicle.trip_id.push_back(
            vehicle[vehicle_index].trip_id[0]);  // Add the depot
    new_vehicle.trip_id.insert(new_vehicle.trip_id.begin()+1,
            vehicle[vehicle_index].trip_id.begin()+trip_in_rotation_index+1,
            vehicle[vehicle_index].trip_id.end()); // Add the second half of the original rotation
    vehicle.push_back(new_vehicle);

    // Remote existing trips on vehicle_index after trip_in_rotation_index till the last but one element
    vehicle[vehicle_index].trip_id.erase(vehicle[vehicle_index].trip_id.begin()+trip_in_rotation_index+1,
            vehicle[vehicle_index].trip_id.end()-1);

    // Log trip IDs after splitting
    logger.log(LogLevel::Info, "Old rotation trip IDs: "+vector_to_string(vehicle[vehicle_index].trip_id));
    logger.log(LogLevel::Info, "New rotation trip IDs: "+vector_to_string(vehicle[vehicle.size()-1].trip_id));

    // Add the old and new vehicle to scan_eligible_vehicle_indices
    scan_eligible_vehicle_indices.push_back(vehicle.size()-1);
    // Add the old vehicle if it is not already present in scan_eligible_vehicle_indices
    if (std::find(scan_eligible_vehicle_indices.begin(), scan_eligible_vehicle_indices.end(), vehicle_index)
            ==scan_eligible_vehicle_indices.end())
        scan_eligible_vehicle_indices.push_back(vehicle_index);
}

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
    for (int v = 0; v<vehicle.size(); ++v) {
        for (int i = 1; i<vehicle[v].trip_id.size()-2; ++i) {
            curr_trip = vehicle[v].trip_id[i];
            next_trip = vehicle[v].trip_id[i+1];

            end_terminal_curr_trip = trip[curr_trip-1].end_terminal;  // End terminal id of current trip
            start_terminal_next_trip = trip[next_trip-1].start_terminal;  // Start terminal id of next trip

            is_curr_trip_end_charge_terminal = terminal[end_terminal_curr_trip-1].is_charge_station;
            is_next_trip_start_charge_terminal = terminal[start_terminal_next_trip-1].is_charge_station;

            // n: no charging, e: end terminal, s: start terminal; Charging is preferred at the end terminal
            char scenario = 'n';
            if (is_curr_trip_end_charge_terminal)
                scenario = 'e';
            else if (is_next_trip_start_charge_terminal and not is_curr_trip_end_charge_terminal)
                scenario = 's';

            switch (scenario) {
            case 'e':terminal[end_terminal_curr_trip-1].current_idle_time += trip[curr_trip-1].idle_time[next_trip-1];
                break;
            case 's':  // If charging station is opened only at the start terminal of the next trip
                terminal[start_terminal_next_trip-1].current_idle_time += trip[curr_trip-1].idle_time[next_trip-1];
                terminal[end_terminal_curr_trip-1].potential_idle_time += trip[curr_trip-1].idle_time[next_trip-1];
                break;
            default:  // No charging location is available at either location
                terminal[start_terminal_next_trip-1].potential_idle_time += trip[curr_trip-1].idle_time[next_trip-1];
                terminal[end_terminal_curr_trip-1].potential_idle_time += trip[curr_trip-1].idle_time[next_trip-1];
            }
        }
    }

    // Log utilization statistics of all terminals
    logger.log(LogLevel::Info,
            "Utilization statistics: (Terminal ID: Is charge station, Current idle time, Potential idle time)");
    for (const auto& curr_terminal : terminal) {
        logger.log(LogLevel::Info,
                "Terminal "+std::to_string(curr_terminal.id)+": "+std::to_string(curr_terminal.is_charge_station)+" "
                        +std::to_string(curr_terminal.current_idle_time)+" "
                        +std::to_string(curr_terminal.potential_idle_time));
    }
}

// Calculate the objective that includes the cost of deadheading and opening charging stations
double evaluation::calculate_objective(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Logger& logger)
{
    // Calculate the objective value of the initial solution
    logger.log(LogLevel::Info, "Calculating the objective value of the solution...");

    // Calculate fixed costs of opening charging stations
    // TODO: This can be calculated faster if we keep track of the stations. Do we need to? Only if this is called often.
    double location_cost = 0.0;
    for (const auto& curr_terminal : terminal)
        location_cost += (curr_terminal.is_charge_station) ? CHARGE_LOC_COST : 0;

    // Calculate fixed cost of bus acquisition based on the number of vehicles
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

    // Calculate the total cost and log the cost components
    double total_cost = location_cost+vehicle_acquisition_cost+deadhead_cost;
    logger.log(LogLevel::Debug, "Fixed cost of opening charging stations: "+std::to_string(location_cost));
    logger.log(LogLevel::Debug, "Fixed cost of bus acquisition: "+std::to_string(vehicle_acquisition_cost));
    logger.log(LogLevel::Debug, "Total cost of deadheading: "+std::to_string(deadhead_cost));
    logger.log(LogLevel::Info, "Number of charging stations: "+std::to_string(int(location_cost/CHARGE_LOC_COST)));
    logger.log(LogLevel::Info, "Number of buses used: "+std::to_string(vehicle.size()));
    logger.log(LogLevel::Info, "Total cost: "+std::to_string(total_cost));

    return total_cost;
}

// Function that checks if a rotation is charge feasible
bool evaluation::check_charge_feasibility_and_split_rotations(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, std::vector<int>& scan_eligible_vehicle_indices, Logger& logger)
{
    // Loop until scan_eligible_vehicle_indices is not empty
    while (not scan_eligible_vehicle_indices.empty()) {
        bool is_splitting_required = false;  // Flag to indicate if splitting is required
        int split_trip_index = -1;  // Trip index after which splitting can be carried out and the depot can be reached
        double charge_level_until_depot = 0.0;

        // Pop the last element from scan_eligible_vehicle_indices
        int v = scan_eligible_vehicle_indices.back();
        scan_eligible_vehicle_indices.pop_back();

        logger.log(LogLevel::Info, "Checking feasibility of vehicle: "+std::to_string(v));

        // Calculate the energy required for deadheading from the depot to the end of the first trip
        double charge_level = MAX_CHARGE_LEVEL-(trip[vehicle[v].trip_id[0]-1].deadhead_distance[vehicle[v].trip_id[1]-1]
                +trip[vehicle[v].trip_id[1]-1].distance)*ENERGY_PER_KM;

        // Check if energy level is above the minimum threshold, if so, return false and break
        if (charge_level<MIN_CHARGE_LEVEL)
            return false;

        int last_trip = vehicle[v].trip_id[vehicle[v].trip_id.size()-1];
        charge_level_until_depot =
                charge_level-trip[vehicle[v].trip_id[1]-1].deadhead_distance[last_trip-1]*ENERGY_PER_KM;
        if (charge_level_until_depot>=MIN_CHARGE_LEVEL)
            split_trip_index = 1;

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

            charge_level_until_depot = charge_level-trip[curr_trip-1].deadhead_distance[last_trip-1]*ENERGY_PER_KM;
            if (charge_level_until_depot>=MIN_CHARGE_LEVEL)
                split_trip_index = i;

            if (not is_charge_adequate_next_trip(trip, curr_trip, next_trip, is_curr_trip_end_charge_terminal,
                    is_next_trip_start_charge_terminal, charge_time_window, charge_level)) {
                is_splitting_required = true;
                break;
            }
        }

        // Add distance from the last trip to the depot to cumulative_energy
        int last_but_one_trip = vehicle[v].trip_id[vehicle[v].trip_id.size()-2];
        charge_level -= trip[last_but_one_trip-1].deadhead_distance[last_trip-1]*ENERGY_PER_KM;
        if (charge_level<MIN_CHARGE_LEVEL)
            is_splitting_required = true;

        if (is_splitting_required)
            locations::split_trips(vehicle, trip, terminal, v, split_trip_index, scan_eligible_vehicle_indices, logger);
    }
    return true;
}

// Checks if exchanging two trips across two different vehicle rotations is compatible
bool evaluation::is_exchange_compatible(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        int first_vehicle_index, int second_vehicle_index, int first_vehicle_trip_index, int second_vehicle_trip_index)
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

// Checks if shifting a trip from one vehicle rotation to another is compatible
bool evaluation::is_shift_compatible(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        int dest_vehicle_index, int source_vehicle_index, int dest_vehicle_trip_index, int source_vehicle_trip_index)
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
    bool is_charge_sufficient;

    // n: no charging, e: end terminal, s: start terminal; Charging is preferred at the end terminal
    char scenario = 'n';
    if (is_curr_trip_end_charge_terminal)
        scenario = 'e';
    else if (is_next_trip_start_charge_terminal and not is_curr_trip_end_charge_terminal)
        scenario = 's';

    switch (scenario) {
    case 'e':charge_level = std::min(charge_level+(charge_time_window*MAX_ENERGY_PER_MIN), MAX_CHARGE_LEVEL);
        charge_level -= trip[curr_trip-1].deadhead_distance[next_trip-1]*ENERGY_PER_KM;
        is_charge_sufficient = (charge_level>=MIN_CHARGE_LEVEL);  // Sets LHS to true if charge level >= min threshold
        break;
    case 's':charge_level -= trip[curr_trip-1].deadhead_distance[next_trip-1]*ENERGY_PER_KM;
        is_charge_sufficient = (charge_level>=MIN_CHARGE_LEVEL);
        charge_level = std::min(charge_level+(charge_time_window*MAX_ENERGY_PER_MIN), MAX_CHARGE_LEVEL);
        break;
    default:  // No charging location is available at either location
        charge_level -= trip[curr_trip-1].deadhead_distance[next_trip-1]*ENERGY_PER_KM;
        is_charge_sufficient = (charge_level>=MIN_CHARGE_LEVEL);
    }
    charge_level -= trip[next_trip-1].distance*ENERGY_PER_KM;
    is_charge_sufficient = (charge_level>=MIN_CHARGE_LEVEL);

    return is_charge_sufficient;
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

        // Add distance from the last trip to the depot to cumulative_energy
        int last_but_one_trip = curr_rotation[curr_rotation.size()-2];
        int last_trip = curr_rotation[curr_rotation.size()-1];
        charge_level -= trip[last_but_one_trip-1].deadhead_distance[last_trip-1]*ENERGY_PER_KM;
        if (charge_level<MIN_CHARGE_LEVEL)
            return false;
    }
    return true;
}

// Calculate cost changes due to replacing a trip of a vehicle rotation with a trip from another rotation
double evaluation::calculate_trip_replacement_cost(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        int first_vehicle_index, int second_vehicle_index, int first_vehicle_trip_index, int second_vehicle_trip_index)
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

    return curr_cost-new_cost;  // If current cost is higher, the savings are positive and the exchange is beneficial
}

// Calculate the cost changes from replacing a depot with another depot at the end of a trip
// Start depots are not exchanged because they were found from nearest neighbors
double evaluation::calculate_depot_replacement_cost(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        int first_vehicle_index, int second_vehicle_index, int first_vehicle_trip_index, int second_vehicle_trip_index)
{
    int last_trip = vehicle[first_vehicle_index].trip_id[first_vehicle_trip_index-1];
    int old_depot = vehicle[first_vehicle_index].trip_id[first_vehicle_trip_index];
    int new_depot = vehicle[second_vehicle_index].trip_id[second_vehicle_trip_index];

    double curr_cost = (trip[last_trip-1].deadhead_distance[old_depot-1])*COST_PER_KM;
    double new_cost = (trip[last_trip-1].deadhead_distance[new_depot-1])*COST_PER_KM;

    return curr_cost-new_cost;  // If current cost is higher, the savings are positive and the exchange is beneficial
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

    return curr_cost-new_cost;  // If current cost is higher, the savings are positive and the exchange is beneficial
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
    // If so, add the cost of the bus to the savings
    if (vehicle[source_vehicle_index].trip_id.size()==3)
        new_cost -= VEHICLE_COST;

    return curr_cost-new_cost;  // If current cost is higher, the savings are positive and the exchange is beneficial
}
