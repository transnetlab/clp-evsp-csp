#include "operators.h"

double operators::exchange_trips(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Exchange& exchange, Logger& logger)
{
    // Find a pair of vehicle rotation as a set from vehicles
    double max_savings = 0.0;  // Stores the maximum savings among all exchanges

    // Store a vector of trip_id vectors after swapping trips
    std::vector<std::vector<int>> swapped_trip_ids;

    for (int u = 0; u<vehicle.size(); u++) {
        for (int v = u+1; v<vehicle.size(); v++) {
            for (int k = 1; k<vehicle[u].num_trips-1; k++) {  //  Exchange trips k and l of vehicles u and v
                for (int l = 1; l<vehicle[v].num_trips-1; l++) {
                    double savings = 0.0;
                    // Check if the exchanges is feasible
                    if (evaluation::is_exchange_compatible(vehicle, trip, u, v, k, l)) {
                        // Check if exchanges are charge feasible
                        swapped_trip_ids.clear();

                        // Push the original trip_ids to swapped_trip_ids
                        swapped_trip_ids.push_back(vehicle[u].trip_id);
                        swapped_trip_ids.push_back(vehicle[v].trip_id);

                        // Exchange trips k and l of vehicles u and v in swapped_trip_ids
                        int temp = swapped_trip_ids[0][k];
                        swapped_trip_ids[0][k] = swapped_trip_ids[1][l];
                        swapped_trip_ids[1][l] = temp;

                        if (evaluation::are_rotations_charge_feasible(trip, swapped_trip_ids, logger)) {
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
    std::vector<std::vector<int>> shifted_trip_ids;

    for (int u = 0; u<vehicle.size(); u++) {
        for (int v = u+1; v<vehicle.size(); v++) {
            for (int k = 1; k<vehicle[u].num_trips-1; k++) {  //  Insert trip l of vehicle v after trip k of vehicle u
                for (int l = 1; l<vehicle[v].num_trips-1; l++) {
                    double savings = 0.0;

                    // Check if the exchanges is feasible
                    if (evaluation::is_shift_compatible(vehicle, trip, u, v, k, l)) {
                        // Check if exchanges are charge feasible
                        shifted_trip_ids.clear();

                        // Insert the original trip_ids to shifted_trip_ids
                        shifted_trip_ids.push_back(vehicle[u].trip_id);
                        shifted_trip_ids.push_back(vehicle[v].trip_id);

                        // Insert trip l of vehicle v after trip k of vehicle u in shifted_trip_ids
                        shifted_trip_ids[0].insert(shifted_trip_ids[0].begin()+k+1, shifted_trip_ids[1][l]);

                        // Remove trip l of vehicle v from shifted_trip_ids
                        shifted_trip_ids[1].erase(shifted_trip_ids[1].begin()+l);

                        // Check if shifted_trip_ids[1] has only two trips. If so, delete it
                        if (shifted_trip_ids[1].size()==2)
                            shifted_trip_ids.erase(shifted_trip_ids.begin()+1);

                        if (evaluation::are_rotations_charge_feasible(trip, shifted_trip_ids, logger)) {
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
void operators::best_improvement(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Logger& logger)
{
    logger.log(LogLevel::Info, "Running best improvement operator...");

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

    // Check if savings are positive
    if (exchange_savings<=0.0 and shift_savings<=0.0) {
        logger.log(LogLevel::Info, "No improvement possible. Exiting...");
        return;
    }

    if (exchange_savings>shift_savings) {
        logger.log(LogLevel::Info, "Exchanges operator is better than shifts operator. Performing exchange...");

        // Log the members of the exchange object
        logger.log(LogLevel::Info, "First vehicle index, Second vehicle index, First trip index, Second trip index");
        logger.log(LogLevel::Info,
                std::to_string(exchange.first_vehicle_index)+", "+std::to_string(exchange.second_vehicle_index)+", "
                        +std::to_string(exchange.first_trip_index)+", "+std::to_string(exchange.second_trip_index));

        // TODO: Add details of charging locations opened
        perform_exchange(vehicle, terminal, exchange);
    }
    else {
        logger.log(LogLevel::Info, "Shifts operator is better than exchanges operator");

        // Log the members of the shift object
        logger.log(LogLevel::Info,
                "Destination vehicle index, Source vehicle index, Destination trip index, Source trip index");
        logger.log(LogLevel::Info,
                std::to_string(shift.dest_vehicle_index)+", "+std::to_string(shift.source_vehicle_index)+", "
                        +std::to_string(shift.dest_trip_index)+", "+std::to_string(shift.source_trip_index));

        // TODO: Add details of charging locations opened
        perform_shift(vehicle, terminal, shift);
    }
}

void evaluation::calculate_objective(std::vector<Trip>& trip, std::vector<Terminal>& terminal,
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
        current_vehicle.update_num_trips();
        current_vehicle.calculate_deadhead_cost(trip);
        // Log the deadheading cost of each trip
        logger.log(LogLevel::Debug, "Deadheading cost of vehicle "+std::to_string(current_vehicle.id)+": "
                +std::to_string(current_vehicle.deadhead_cost));
        deadhead_cost += current_vehicle.deadhead_cost;
    }

    // Calculate the total cost and log the cost components
    double total_cost = location_cost+vehicle_acquisition_cost+deadhead_cost;
    logger.log(LogLevel::Info, "Fixed cost of opening charging stations: "+std::to_string(location_cost));
    logger.log(LogLevel::Info, "Fixed cost of bus acquisition: "+std::to_string(vehicle_acquisition_cost));
    logger.log(LogLevel::Info, "Cost of deadheading: "+std::to_string(deadhead_cost));
    logger.log(LogLevel::Info, "Total cost: "+std::to_string(total_cost));
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
bool evaluation::are_rotations_charge_feasible(std::vector<Trip>& trip, std::vector<std::vector<int>> rotation, Logger& logger) {
    // Iterate across rotations
    for (auto current_rotation : rotation) {
        // Iterate across trips in the rotation
        for (int i = 1; i<current_rotation.size()-1; i++) {
            // Check if the trip is charge feasible
            if (trip[current_rotation[i]-1].is_charge_feasible==false) {
                logger.log(LogLevel::Info, "Rotation is not charge feasible. Exiting...");
                return false;
            }
        }
    }
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
            +trip[curr_trip_id-1].deadhead_distance[next_trip_id-1])*DEADHEAD_COST_FACTOR;

    double new_cost = (trip[prev_trip_id-1].deadhead_distance[new_trip_id-1]
            +trip[new_trip_id-1].deadhead_distance[next_trip_id-1])*DEADHEAD_COST_FACTOR;

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

    double current_cost = trip[curr_trip_id-1].deadhead_distance[next_trip_id-1]*DEADHEAD_COST_FACTOR;

    double new_cost = (trip[curr_trip_id-1].deadhead_distance[new_trip_id-1]
            +trip[new_trip_id-1].deadhead_distance[next_trip_id-1])*DEADHEAD_COST_FACTOR;

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

    double current_cost = trip[prev_trip_id-1].deadhead_distance[curr_trip_id-1]*DEADHEAD_COST_FACTOR
            +trip[curr_trip_id-1].deadhead_distance[next_trip_id-1]*DEADHEAD_COST_FACTOR;

    double new_cost = trip[prev_trip_id-1].deadhead_distance[next_trip_id-1]*DEADHEAD_COST_FACTOR;

    // Check if the size of trips in the source vehicle is 3 (two depots and one trip)
    // If so, add the cost of the bus to the savings
    if (vehicle[source_vehicle_index].num_trips==3)
        new_cost += VEHICLE_COST;

    return current_cost-new_cost;  // If current cost is higher, the savings are positive and the exchange is beneficial
}
