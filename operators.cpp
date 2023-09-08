#include "operators.h"

template<typename T>
std::string vector_to_string(const std::vector<T>& input_vector)
{
    std::stringstream ss;
    for (const T& element : input_vector) {
        ss << element << " ";
    }
    return ss.str();
}

double operators::exchange_trips(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Exchange& exchange, Logger& logger)
{
    // Find a pair of vehicle rotation as a set from vehicles
    double max_savings = 0.0;  // Stores the maximum savings among all exchanges

    // Store a vector of trip_id vectors after swapping trips
    std::vector<std::vector<int>> swapped_rotations;

    for (int u = 0; u<vehicle.size(); u++) {
        for (int v = u+1; v<vehicle.size(); v++) {
            for (int k = 1; k<vehicle[u].trip_id.size()-1; k++) {  //  Exchange trips k and l of vehicles u and v
                for (int l = 1; l<vehicle[v].trip_id.size()-1; l++) {
                    double savings = 0.0;
                    // Check if the exchanges is feasible
                    if (evaluation::is_exchange_compatible(vehicle, trip, u, v, k, l)) {
                        // Check if exchanges are charge feasible
                        swapped_rotations.clear();

                        // Push the original trip_ids to swapped_rotations
                        swapped_rotations.push_back(vehicle[u].trip_id);
                        swapped_rotations.push_back(vehicle[v].trip_id);

                        // Exchange trips k and l of vehicles u and v in swapped_rotations
                        int temp = swapped_rotations[0][k];
                        swapped_rotations[0][k] = swapped_rotations[1][l];
                        swapped_rotations[1][l] = temp;

                        if (evaluation::are_rotations_charge_feasible(trip, terminal, swapped_rotations)) {
                            // Calculate savings in deadheading from performing the exchange  TODO: Can the replacement costs be calculated using the swapped trips?
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

double operators::shift_trips(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip, std::vector<Terminal>& terminal,
        Shift& shift, Logger& logger)
{
    // Variables for calculating the savings from trip shifts
    double max_savings = 0.0;

    // Store a vector of trip_id vectors after shifting trips
    std::vector<std::vector<int>> shifted_rotations;

    for (int u = 0; u<vehicle.size(); u++) {
        for (int v = u+1; v<vehicle.size(); v++) {
            for (int k = 1;
                 k<vehicle[u].trip_id.size()-1;
                 k++) {  //  Insert trip l of vehicle v after trip k of vehicle u
                for (int l = 1; l<vehicle[v].trip_id.size()-1; l++) {
                    double savings = 0.0;

                    // Check if the exchanges is feasible
                    if (evaluation::is_shift_compatible(vehicle, trip, u, v, k, l)) {
                        // Check if exchanges are charge feasible
                        shifted_rotations.clear();

                        // Insert the original trip_ids to shifted_rotations
                        shifted_rotations.push_back(vehicle[u].trip_id);
                        shifted_rotations.push_back(vehicle[v].trip_id);

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

// Write a function to actually perform the exchange using the exchange object
void operators::perform_exchange(std::vector<Vehicle>& vehicle, std::vector<Terminal>& terminal, Exchange& exchange)
{
    // Exchange trips k and l of vehicles u and v
    int first_vehicle_index = exchange.first_vehicle_index;
    int second_vehicle_index = exchange.second_vehicle_index;
    int first_trip_index = exchange.first_trip_index;
    int second_trip_index = exchange.second_trip_index;

    // Swap the trips
    int temp = vehicle[first_vehicle_index].trip_id[first_trip_index];
    vehicle[first_vehicle_index].trip_id[first_trip_index] = vehicle[second_vehicle_index].trip_id[second_trip_index];
    vehicle[second_vehicle_index].trip_id[second_trip_index] = temp;

    // TODO: Update charging stations if any new ones are opened
}

// Write a function to actually perform the shift using the shift object
void operators::perform_shift(std::vector<Vehicle>& vehicle, std::vector<Terminal>& terminal, Shift& shift)
{
    // Insert trip l of vehicle v after trip k of vehicle u
    int dest_vehicle_index = shift.dest_vehicle_index;
    int source_vehicle_index = shift.source_vehicle_index;
    int dest_trip_index = shift.dest_trip_index;
    int source_trip_index = shift.source_trip_index;

    // Insert the trip
    vehicle[dest_vehicle_index].trip_id.insert(vehicle[dest_vehicle_index].trip_id.begin()+dest_trip_index+1,
            vehicle[source_vehicle_index].trip_id[source_trip_index]);

    // Remove the trip
    vehicle[source_vehicle_index].trip_id.erase(vehicle[source_vehicle_index].trip_id.begin()+source_trip_index);

    // If the source vehicle has only two trips, remove the vehicle
    if (vehicle[source_vehicle_index].trip_id.size()==2)
        vehicle.erase(vehicle.begin()
                +source_vehicle_index);   //  TODO: Check if removal creates a problem with indexing +-1 issues
}

// Best improvement function
void operators::optimize_scheduling(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Logger& logger)
{
    logger.log(LogLevel::Debug, "Running best improvement operator...");

    // Run the exchanges operator
    Exchange exchange;
    double exchange_savings = exchange_trips(vehicle, trip, terminal, exchange, logger);

    // Run the shifts operator
    Shift shift;
    double shift_savings = shift_trips(vehicle, trip, terminal, shift, logger);

    // Check if the exchanges operator is better than the shifts operator
    // Print the values of savings from both operators
    logger.log(LogLevel::Info, "Savings from exchanges operator: "+std::to_string(exchange_savings));
    logger.log(LogLevel::Info, "Savings from shifts operator: "+std::to_string(shift_savings));

    // Generate a random number between 0 and 1
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    double random_number = dis(gen);

    // Check if savings are positive
    if (exchange_savings<=0.0 and shift_savings<=0.0 and random_number>0.05) {
        logger.log(LogLevel::Debug, "No improvement possible. Exiting...");
        return;
    }

    if (exchange_savings>shift_savings) {
        logger.log(LogLevel::Debug, "Exchanges operator is better than shifts operator. Performing exchange...");

        // Log the members of the exchange object
        logger.log(LogLevel::Debug, "First vehicle index, Second vehicle index, First trip index, Second trip index");
        logger.log(LogLevel::Debug,
                std::to_string(exchange.first_vehicle_index)+", "+std::to_string(exchange.second_vehicle_index)+", "
                        +std::to_string(exchange.first_trip_index)+", "+std::to_string(exchange.second_trip_index));

        // TODO: Add details of charging locations opened
        perform_exchange(vehicle, terminal, exchange);
    }
    else {
        logger.log(LogLevel::Debug, "Shifts operator is better than exchanges operator");

        // Log the members of the shift object
        logger.log(LogLevel::Debug,
                "Destination vehicle index, Source vehicle index, Destination trip index, Source trip index");
        logger.log(LogLevel::Debug,
                std::to_string(shift.dest_vehicle_index)+", "+std::to_string(shift.source_vehicle_index)+", "
                        +std::to_string(shift.dest_trip_index)+", "+std::to_string(shift.source_trip_index));

        // TODO: Add details of charging locations opened
        perform_shift(vehicle, terminal, shift);
    }
}

void operators::close_charging_stations(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, int close_terminal_id, Logger& logger)
{
    /*logger.log(LogLevel::Info, "Running location optimization operator...");
    // Print values
    logger.log(LogLevel::Info, "Sum of idle times at open terminals: "+std::to_string(sum_open_idle_times));
    logger.log(LogLevel::Info, "Sum of idle times at closed terminals: "+std::to_string(sum_closed_idle_times));*/

    // Track the savings
    double best_objective = evaluation::calculate_objective(trip, terminal, vehicle, logger);
    double old_objective = 1e12;

    // Close the terminal with the chosen index
    terminal[close_terminal_id-1].is_charge_station = false;

    // Scan through the rotations and find the ones that have the chosen terminal and add them to a scan eligible list
    std::vector<int> scan_eligible_rotations;
    for (int v = 0; v<vehicle.size(); ++v) {
        for (int i = 1; i<vehicle[v].trip_id.size()-1; ++i) {
            if (trip[vehicle[v].trip_id[i]-1].start_terminal==close_terminal_id
                    or trip[vehicle[v].trip_id[i]-1].end_terminal==close_terminal_id) {
                scan_eligible_rotations.push_back(v);
                break;
            }
        }
    }

    // Log the details of the scan eligible rotations
    logger.log(LogLevel::Info, "Scan eligible rotations: "+vector_to_string(scan_eligible_rotations));

    // Check if rotations are feasible after deletion of the terminal. If not, create new rotations.
    evaluation::check_rotation_feasibility(vehicle, trip, terminal, scan_eligible_rotations, logger);

    // Check if savings can be achieved from applying the scheduling operators
    int num_iterations = 0;
    while (best_objective
            <old_objective*1.05 and num_iterations < 100) {  // TODO: By default it will do this twice since deleting a terminal will create savings
        num_iterations++;
        old_objective = best_objective;
        operators::optimize_scheduling(vehicle, trip, terminal, logger);
        best_objective = evaluation::calculate_objective(trip, terminal, vehicle, logger);
    }
}

void operators::open_charging_stations(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, int open_terminal_id, Logger& logger)
{
    double best_objective = evaluation::calculate_objective(trip, terminal, vehicle, logger);
    double old_objective = 1e12;

    // Open the terminal with the chosen index
    terminal[open_terminal_id-1].is_charge_station = true;

    // Check if savings can be achieved from applying the scheduling operators
    int num_iterations = 0;
    while (best_objective<old_objective*1.05 and num_iterations < 100) {
        ++num_iterations;
        old_objective = best_objective;
        operators::optimize_scheduling(vehicle, trip, terminal, logger);
        best_objective = evaluation::calculate_objective(trip, terminal, vehicle, logger);
    }
}

// Function to open or close locations. This could create new rotations as well when locations are closed
void operators::optimize_locations(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Logger& logger)
{
    logger.log(LogLevel::Info, "Checking if charge stations can be closed...");
    // Find utilization of different terminals TODO: These can be tracked upfront and be updated incrementally after every shift and exchange
    evaluation::calculate_utilization(vehicle, trip, terminal, logger);

    // Close all open charging stations that have zero utilization
    logger.log(LogLevel::Info, "Closing charge stations with zero utilization...");
    for (auto& curr_terminal : terminal) {
        if (curr_terminal.is_charge_station and curr_terminal.total_idle_time==0) {
            curr_terminal.is_charge_station = false;
        }
    }

    // Among the ones that are open, choose a terminal randomly probability proportional to their total idle time
    // Create a vector of vectors with the terminal id and the total idle time
    std::vector<int> open_charge_station_ids;
    std::vector<double> deletion_proportions;
    for (const auto& curr_terminal : terminal) {
        if (curr_terminal.is_charge_station) {
            open_charge_station_ids.push_back(curr_terminal.id);
            deletion_proportions.push_back(1.0/double(curr_terminal.total_idle_time));
        }
    }

    // Choose an index at random proportional to the deletion probabilities
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> d(deletion_proportions.begin(), deletion_proportions.end());
    int index = d(gen);

    // Print the terminal id and the deletion probability of the chosen index
    logger.log(LogLevel::Info, "Closing terminal id: "+std::to_string(open_charge_station_ids[index]));
    logger.log(LogLevel::Info,
            "Deletion proportion of the chosen terminal: "+std::to_string(deletion_proportions[index]));
    operators::close_charging_stations(vehicle, trip, terminal, open_charge_station_ids[index], logger);

    // Consider opening charging stations based on utilization and savings
    evaluation::calculate_utilization(vehicle, trip, terminal, logger);

    // Select a closed charging station with the highest utilization and open it. Calculate savings considering scheduling.
    // If the savings are positive, open the charging station, and update the terminal and rotation data.
    int most_utilized_terminal_id;
    double most_utilization = 0.0;
    for (const auto& curr_terminal : terminal) {
        if (not curr_terminal.is_charge_station and curr_terminal.total_idle_time>most_utilization) {
            most_utilization = curr_terminal.total_idle_time;
            most_utilized_terminal_id = curr_terminal.id;
        }
    }

    logger.log(LogLevel::Info, "Opening terminal "+std::to_string(most_utilized_terminal_id)+"...");
    operators::open_charging_stations(vehicle, trip, terminal, most_utilized_terminal_id, logger);
}

void operators::split_trip(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip, std::vector<Terminal>& terminal,
        int vehicle_index, int trip_in_rotation_index, std::vector<int>& scan_eligible_indices, Logger& logger)
{
    // Log operations
    logger.log(LogLevel::Info, "Splitting trip "+std::to_string(vehicle[vehicle_index].trip_id[trip_in_rotation_index])
            +" of vehicle "+std::to_string(vehicle_index)+"...");
    // Log trip IDs before splitting
    logger.log(LogLevel::Info, "Trip IDs before splitting: "+vector_to_string(vehicle[vehicle_index].trip_id));

    // Create a new vehicle with the remaining trips
    Vehicle new_vehicle;
    new_vehicle.trip_id.push_back(vehicle[vehicle_index].trip_id[0]);  // Add the depot
    new_vehicle.trip_id.insert(new_vehicle.trip_id.begin(),
            vehicle[vehicle_index].trip_id.begin()+trip_in_rotation_index+1,
            vehicle[vehicle_index].trip_id.end()); // Add the second half of the original rotation
    vehicle.push_back(new_vehicle);

    // Truncate existing trips on vehicle_index after trip_in_rotation_index till the last but one element
    vehicle[vehicle_index].trip_id.erase(vehicle[vehicle_index].trip_id.begin()+trip_in_rotation_index+1,
            vehicle[vehicle_index].trip_id.end()-1);

    // Log trip IDs after splitting
    logger.log(LogLevel::Info, "Trip IDs after splitting: "+vector_to_string(vehicle[vehicle_index].trip_id));

    // Add the new vehicle to scan_eligible_indices
    scan_eligible_indices.push_back(vehicle.size()-1);
}

void evaluation::check_rotation_feasibility(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal,
        std::vector<int>& scan_eligible_vehicle_indices, Logger& logger)
{
    // Loop until scan_eligible_vehicle_indices is not empty
    while (not scan_eligible_vehicle_indices.empty()) {
        // Pop the last element from scan_eligible_vehicle_indices
        int v = scan_eligible_vehicle_indices.back();
        scan_eligible_vehicle_indices.pop_back();

        double charge_level = MAX_CHARGE_LEVEL-(trip[vehicle[v].trip_id[0]-1].deadhead_distance[
                vehicle[v].trip_id[1]-1]+trip[vehicle[v].trip_id[0]-1].distance)
                *ENERGY_PER_KM;

        int curr_trip, next_trip;  // Current trip and next trip IDs
        int end_terminal_curr_trip, start_terminal_next_trip; // End terminal of current trip and start terminal of the next trip
        bool is_trip_end_charge_terminal, is_next_trip_start_charge_terminal;
        int charge_time_window;  // Idle time during which charging is allowed

        // Scan through the trips in the rotation
        for (int i = 1; i<vehicle[v].trip_id.size()-2; ++i) {
            curr_trip = vehicle[v].trip_id[i];
            next_trip = vehicle[v].trip_id[i+1];

            end_terminal_curr_trip = trip[curr_trip-1].end_terminal;  // End terminal id of current trip
            start_terminal_next_trip = trip[next_trip-1].start_terminal;  // Start terminal id of next trip

            is_trip_end_charge_terminal = terminal[end_terminal_curr_trip-1].is_charge_station;
            is_next_trip_start_charge_terminal = terminal[start_terminal_next_trip-1].is_charge_station;

            charge_time_window = trip[curr_trip-1].idle_time[next_trip-1];

            char scenario = 'n';  // n: no charging, e: end terminal, s: start terminal
            // If both locations have charging stations, prefer charging at end terminal
            // Else if either one of them have a charging station add it to the charge_terminal
            if (is_trip_end_charge_terminal && is_next_trip_start_charge_terminal)
                scenario = 'e';
            else if (is_trip_end_charge_terminal || is_next_trip_start_charge_terminal)
                scenario = is_trip_end_charge_terminal ? 'e' : 's';

            switch (scenario) {
            case 'e':charge_level = std::min(charge_level+(charge_time_window*MAX_ENERGY_PER_MIN), MAX_CHARGE_LEVEL);
                charge_level -= trip[curr_trip-1].deadhead_distance[next_trip-1]*ENERGY_PER_KM;
                if (charge_level<MIN_CHARGE_LEVEL) {
                    operators::split_trip(vehicle, trip, terminal, v, i-1, scan_eligible_vehicle_indices,
                            logger);  // Splits after i - 1
                    break;
                }
                break;
            case 's':charge_level -= trip[curr_trip-1].deadhead_distance[next_trip-1]*ENERGY_PER_KM;
                if (charge_level<MIN_CHARGE_LEVEL) {
                    operators::split_trip(vehicle, trip, terminal, v, i, scan_eligible_vehicle_indices,
                            logger);  // Splits after i
                    break;
                }
                charge_level = std::min(charge_level+charge_time_window*MAX_ENERGY_PER_MIN, MAX_CHARGE_LEVEL);
                break;
            default:  // No charging location is available at either location
                charge_level -= trip[curr_trip-1].deadhead_distance[next_trip-1]*ENERGY_PER_KM;
                if (charge_level<MIN_CHARGE_LEVEL) {
                    operators::split_trip(vehicle, trip, terminal, v, i, scan_eligible_vehicle_indices,
                            logger);  // Splits after i
                    break;
                }
            }
            charge_level -= trip[next_trip-1].distance*ENERGY_PER_KM;
            if (charge_level<MIN_CHARGE_LEVEL) {
                operators::split_trip(vehicle, trip, terminal, v, i-1, scan_eligible_vehicle_indices,
                        logger);  // Splits after i
                break;
            }
        }

        // Add distance from the last trip to the depot to cumulative_energy
        int last_but_one_trip = vehicle[v].trip_id[vehicle[v].trip_id.size()-2];
        int last_trip = vehicle[v].trip_id[vehicle[v].trip_id.size()-1];
        charge_level -= trip[last_but_one_trip-1].deadhead_distance[last_trip-1]*ENERGY_PER_KM;
        if (charge_level<MIN_CHARGE_LEVEL) {
            operators::split_trip(vehicle, trip, terminal, v, vehicle[v].trip_id.size()-3,
                    scan_eligible_vehicle_indices, logger);  // Splits after i
            break;
        }
    }
}

void evaluation::calculate_utilization(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Logger& logger)
{
    logger.log(LogLevel::Info, "Calculating utilization statistics of all terminals...");

    // Estimate the amount of idle time available at each charging and non charging terminal across all vehicles
    for (auto& curr_terminal : terminal) {
        curr_terminal.total_idle_time = 0;
    }

    int curr_trip, next_trip;  // Current trip and next trip IDs
    int end_terminal_curr_trip, start_terminal_next_trip; // End terminal of current trip and start terminal of the next trip
    bool is_trip_end_charge_terminal, is_next_trip_start_charge_terminal;
    for (int v = 0; v<vehicle.size(); ++v) {
        for (int i = 1; i<vehicle[v].trip_id.size()-2; ++i) {
            curr_trip = vehicle[v].trip_id[i];
            next_trip = vehicle[v].trip_id[i+1];

            end_terminal_curr_trip = trip[curr_trip-1].end_terminal;  // End terminal id of current trip
            start_terminal_next_trip = trip[next_trip-1].start_terminal;  // Start terminal id of next trip

            is_trip_end_charge_terminal = terminal[end_terminal_curr_trip-1].is_charge_station;
            is_next_trip_start_charge_terminal = terminal[start_terminal_next_trip-1].is_charge_station;

            char scenario = 'n';  // n: no charging, e: end terminal, s: start terminal
            // If both locations have charging stations, prefer charging at end terminal
            // Else if either one of them have a charging station add it to the charge_terminal
            if (is_trip_end_charge_terminal && is_next_trip_start_charge_terminal)
                scenario = 'e';
            else if (is_trip_end_charge_terminal || is_next_trip_start_charge_terminal)
                scenario = is_trip_end_charge_terminal ? 'e' : 's';

            switch (scenario) {
            case 'e':terminal[end_terminal_curr_trip-1].total_idle_time += trip[curr_trip-1].idle_time[next_trip-1];
                break;
            case 's':  // If charging station is opened at the end terminal, it takes the total idle time under the CAG policy
                terminal[start_terminal_next_trip-1].total_idle_time += trip[curr_trip-1].idle_time[next_trip-1];
                terminal[end_terminal_curr_trip-1].total_idle_time += trip[curr_trip-1].idle_time[next_trip-1];
                break;
            default:  // No charging location is available at either location
                terminal[end_terminal_curr_trip-1].total_idle_time += trip[curr_trip-1].idle_time[next_trip-1];
            }
        }
    }

    // Log utilization statistics of all terminals
    logger.log(LogLevel::Info,
            "Utilization statistics of all terminals: (Terminal ID, Is charge station, Total idle time)");
    for (const auto& curr_terminal : terminal) {
        logger.log(LogLevel::Info,
                "Terminal "+std::to_string(curr_terminal.id)+": "+std::to_string(curr_terminal.is_charge_station)+" "
                        +std::to_string(curr_terminal.total_idle_time));
    }
}

double evaluation::calculate_objective(std::vector<Trip>& trip, std::vector<Terminal>& terminal,
        std::vector<Vehicle>& vehicle,
        Logger& logger)
{
    // Calculate the objective value of the initial solution
    logger.log(LogLevel::Info, "Calculating the objective value of the solution...");

    // Calculate fixed costs of opening charging stations
    // TODO: This can be calculated faster if we keep track of the stations. Do we need to? Only if this is called often.
    double location_cost = 0.0;
    for (const auto& current_terminal : terminal)
        location_cost += (current_terminal.is_charge_station) ? CHARGE_LOC_COST : 0;

    // Calculate fixed cost of bus acquisition based on the number of vehicles
    double vehicle_acquisition_cost = VEHICLE_COST*vehicle.size();

    // Calculate variable cost of deadheading
    double deadhead_cost = 0.0;
    for (auto& current_vehicle : vehicle) {
        current_vehicle.calculate_deadhead_cost(trip);
        // Log the deadheading cost of each trip
        logger.log(LogLevel::Debug, "Deadheading cost of vehicle "+std::to_string(current_vehicle.id)+": "
                +std::to_string(current_vehicle.deadhead_cost));
        deadhead_cost += current_vehicle.deadhead_cost;
    }

    // Calculate the total cost and log the cost components
    double total_cost = location_cost+vehicle_acquisition_cost+deadhead_cost;
    logger.log(LogLevel::Info, "Fixed cost of opening charging stations: "+std::to_string(location_cost));
    logger.log(LogLevel::Info, "Number of charging stations: "+std::to_string(location_cost/CHARGE_LOC_COST));
    logger.log(LogLevel::Info, "Fixed cost of bus acquisition: "+std::to_string(vehicle_acquisition_cost));
    logger.log(LogLevel::Info, "Number of buses used: "+std::to_string(vehicle.size()));
    logger.log(LogLevel::Info, "Cost of deadheading: "+std::to_string(deadhead_cost));
    logger.log(LogLevel::Info, "Total cost: "+std::to_string(total_cost));

    return total_cost;
}

// Checks if exchanging two trips across two different vehicle rotations is compatible
bool evaluation::is_exchange_compatible(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        int u, int v, int k, int l)
{
    int first_vehicle_prev_trip_id = vehicle[u].trip_id[k-1];
    int first_vehicle_curr_trip_id = vehicle[u].trip_id[k];
    int first_vehicle_next_trip_id = vehicle[u].trip_id[k+1];

    int second_vehicle_prev_trip_id = vehicle[v].trip_id[l-1];
    int second_vehicle_curr_trip_id = vehicle[v].trip_id[l];
    int second_vehicle_next_trip_id = vehicle[v].trip_id[l+1];

    // Check if the exchanges is feasible using compatibility matrices
    if (trip[first_vehicle_prev_trip_id-1].is_compatible[second_vehicle_curr_trip_id-1] and
            trip[second_vehicle_curr_trip_id-1].is_compatible[first_vehicle_next_trip_id-1] and
            trip[second_vehicle_prev_trip_id-1].is_compatible[first_vehicle_curr_trip_id-1] and
            trip[first_vehicle_curr_trip_id-1].is_compatible[second_vehicle_next_trip_id-1])
        return true;
    else
        return false;
}

bool evaluation::is_shift_compatible(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        int u, int v, int k, int l)
{
    int dest_vehicle_curr_trip_id = vehicle[u].trip_id[k];
    int dest_vehicle_next_trip_id = vehicle[u].trip_id[k+1];

    int source_vehicle_trip_id = vehicle[v].trip_id[l];  // The trip being shifted

    // Check if the exchanges is feasible using compatibility matrices
    if (trip[dest_vehicle_curr_trip_id-1].is_compatible[source_vehicle_trip_id-1] and
            trip[source_vehicle_trip_id-1].is_compatible[dest_vehicle_next_trip_id-1])
        return true;
    else
        return false;
}

// Function to check if the new trip sequences are charge feasible
bool evaluation::are_rotations_charge_feasible(std::vector<Trip>& trip, std::vector<Terminal>& terminal,
        std::vector<std::vector<int>> rotations)
{
    int curr_trip, next_trip;  // Current trip and next trip IDs
    int end_terminal_curr_trip, start_terminal_next_trip; // End terminal of current trip and start terminal of the next trip
    bool is_trip_end_charge_terminal, is_next_trip_start_charge_terminal;
    int charge_time_window;  // Idle time during which charging is allowed

    // Iterate across rotations
    for (auto curr_rotation : rotations) {
        // Iterate across trips in the rotations
        // Calculate the energy required for deadheading from the depot to the end of the first trip
        double charge_level = MAX_CHARGE_LEVEL
                -(trip[curr_rotation[0]-1].deadhead_distance[curr_rotation[1]-1]+trip[curr_rotation[0]-1].distance)
                        *ENERGY_PER_KM;

        // Check if energy level is above the minimum threshold, if so, return false and break
        if (charge_level<MIN_CHARGE_LEVEL)
            return false;

        for (int i = 1; i<curr_rotation.size()-2; ++i) {
            curr_trip = curr_rotation[i];
            next_trip = curr_rotation[i+1];

            end_terminal_curr_trip = trip[curr_trip-1].end_terminal;  // End terminal id of current trip
            start_terminal_next_trip = trip[next_trip-1].start_terminal;  // Start terminal id of next trip

            is_trip_end_charge_terminal = terminal[end_terminal_curr_trip-1].is_charge_station;
            is_next_trip_start_charge_terminal = terminal[start_terminal_next_trip-1].is_charge_station;

            charge_time_window = trip[curr_trip-1].idle_time[next_trip-1];

            char scenario = 'n';  // n: no charging, e: end terminal, s: start terminal
            // If both locations have charging stations, prefer charging at end terminal
            // Else if either one of them have a charging station add it to the charge_terminal
            if (is_trip_end_charge_terminal && is_next_trip_start_charge_terminal)
                scenario = 'e';
            else if (is_trip_end_charge_terminal || is_next_trip_start_charge_terminal)
                scenario = is_trip_end_charge_terminal ? 'e' : 's';

            switch (scenario) {
            case 'e':charge_level = std::min(charge_level+(charge_time_window*MAX_ENERGY_PER_MIN), MAX_CHARGE_LEVEL);
                charge_level -= trip[curr_trip-1].deadhead_distance[next_trip-1]*ENERGY_PER_KM;
                if (charge_level<MIN_CHARGE_LEVEL)
                    return false;
                break;
            case 's':charge_level -= trip[curr_trip-1].deadhead_distance[next_trip-1]*ENERGY_PER_KM;
                if (charge_level<MIN_CHARGE_LEVEL)
                    return false;
                charge_level = std::min(charge_level+charge_time_window*MAX_ENERGY_PER_MIN, MAX_CHARGE_LEVEL);
                break;
            default:  // No charging location is available at either location
                charge_level -= trip[curr_trip-1].deadhead_distance[next_trip-1]*ENERGY_PER_KM;
                if (charge_level<MIN_CHARGE_LEVEL)
                    return false;
            }
            charge_level -= trip[next_trip-1].distance*ENERGY_PER_KM;
            if (charge_level<MIN_CHARGE_LEVEL)
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

    double current_cost = (trip[prev_trip_id-1].deadhead_distance[curr_trip_id-1]
            +trip[curr_trip_id-1].deadhead_distance[next_trip_id-1])*COST_PER_KM;

    double new_cost = (trip[prev_trip_id-1].deadhead_distance[new_trip_id-1]
            +trip[new_trip_id-1].deadhead_distance[next_trip_id-1])*COST_PER_KM;

    return current_cost-new_cost;  // If current cost is higher, the savings are positive and the exchange is beneficial
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

    double current_cost = trip[curr_trip_id-1].deadhead_distance[next_trip_id-1]*COST_PER_KM;

    double new_cost = (trip[curr_trip_id-1].deadhead_distance[new_trip_id-1]
            +trip[new_trip_id-1].deadhead_distance[next_trip_id-1])*COST_PER_KM;

    return current_cost-new_cost;  // If current cost is higher, the savings are positive and the exchange is beneficial
}

// Calculate cost changes due to removal of an existing trip
double evaluation::calculate_trip_removal_cost(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        int source_vehicle_index, int source_vehicle_trip_index)
{
    // TODO: Add bus cost to the savings if the trip becomes empty

    // Trip source_vehicle_trip_index is being removed from vehicle rotation source_vehicle_index
    // The function calculates the changes in the deadheading costs
    int prev_trip_id = vehicle[source_vehicle_index].trip_id[source_vehicle_trip_index-1];
    int curr_trip_id = vehicle[source_vehicle_index].trip_id[source_vehicle_trip_index];
    int next_trip_id = vehicle[source_vehicle_index].trip_id[source_vehicle_trip_index+1];

    double current_cost = trip[prev_trip_id-1].deadhead_distance[curr_trip_id-1]*COST_PER_KM
            +trip[curr_trip_id-1].deadhead_distance[next_trip_id-1]*COST_PER_KM;

    double new_cost = trip[prev_trip_id-1].deadhead_distance[next_trip_id-1]*COST_PER_KM;

    // Check if the size of trips in the source vehicle is 3 (two depots and one trip)
    // If so, add the cost of the bus to the savings
    if (vehicle[source_vehicle_index].trip_id.size()==3)
        new_cost += VEHICLE_COST;

    return current_cost-new_cost;  // If current cost is higher, the savings are positive and the exchange is beneficial
}

// Function to check for improvement in solutions
void evaluation::update_best_solution(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, std::vector<Vehicle>& best_vehicle,
        std::vector<Terminal>& best_terminal, Logger& logger)
{
    best_terminal = terminal;  // Update the best charging station configuration
    best_vehicle = vehicle;  // Update the best rotations

    // Log the rotation data
    logger.log(LogLevel::Debug, "Current rotation data:");
    for (int i = 0; i<vehicle.size(); ++i) {
        logger.log(LogLevel::Debug, "Vehicle "+std::to_string(i+1));
        for (int j = 0; j<vehicle[i].trip_id.size(); ++j) {
            logger.log(LogLevel::Debug, "Trip "+std::to_string(vehicle[i].trip_id[j]));
        }
    }
}