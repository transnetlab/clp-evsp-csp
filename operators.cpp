#include "operators.h"

void exchanges::two_opt(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip, std::vector<Terminal>& terminal, Logger& logger)
{
    logger.log(LogLevel::Info, "Running exchanges operator which swaps two trips between vehicles");

    // Find a pair of vehicle rotation as a set from vehicles
    double savings = 0.0;
    double max_savings = 0.0;  // Stores the maximum savings among all exchanges
    int best_first_vehicle_index = 0;  // Stores the vehicle index of the first vehicle in the exchange
    int best_second_vehicle_index = 0;  // Stores the vehicle index of the second vehicle in the exchange
    int best_first_vehicle_trip_index = 0;  // Stores the trip index of the first vehicle in the exchange
    int best_second_vehicle_trip_index = 0;  // Stores the trip index of the second vehicle in the exchange
    std::vector<int> best_new_charge_stations;  // Stores the best new charging terminals to be opened in the exchange
    std::vector<int> temp_new_charge_stations;  // Stores the temporary new charging terminals to be opened in the exchange

    for (int u = 0; u < vehicle.size(); u++) {
        for (int v = u + 1; v < vehicle.size(); v++) {
            // Check if trip k of vehicle u and trip l of vehicle v can be exchanged
            for (int k = 1; k < vehicle[u].num_trips - 1; k++) {
                for (int l = 1; l < vehicle[v].num_trips - 1; l++) {
                    // Check if the exchanges is feasible
                    if (evaluation::is_exchange_compatible(vehicle, trip, u, v, k, l)) {
                        // Check if exchanges require new charging stations to be opened and the associated costs
                        temp_new_charge_stations.clear();
                        double new_charge_station_cost = evaluation::make_exchange_charge_feasible(vehicle, trip, terminal, temp_new_charge_stations, u, v, k, l);

                        // Calculate savings in deadheading from performing the exchange
                        savings += evaluation::calculate_trip_replacement_cost(vehicle, trip, u, v, k, l);
                        savings += evaluation::calculate_trip_replacement_cost(vehicle, trip, v, u, l, k);
                        savings += new_charge_station_cost;

                        // Check if the exchange is the best so far
                        if (savings > max_savings) {
                            max_savings = savings;
                            best_first_vehicle_index = u;
                            best_second_vehicle_index = v;
                            best_first_vehicle_trip_index = k;
                            best_second_vehicle_trip_index = l;
                            best_new_charge_stations.clear();
                            best_new_charge_stations = temp_new_charge_stations;
                        }
                    }
                }
            }
        }

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
    int prev_trip_id_first_vehicle = vehicle[u].trip_id[k-1];
    int curr_trip_id_first_vehicle = vehicle[u].trip_id[k];
    int next_trip_id_first_vehicle = vehicle[u].trip_id[k+1];

    int prev_trip_id_second_vehicle = vehicle[v].trip_id[l-1];
    int curr_trip_id_second_vehicle = vehicle[v].trip_id[l];
    int next_trip_id_second_vehicle = vehicle[v].trip_id[l+1];

    // Check if the exchanges is feasible using compatibility matrices
    if (trip[prev_trip_id_first_vehicle-1].is_compatible[curr_trip_id_second_vehicle-1] and
        trip[curr_trip_id_second_vehicle-1].is_compatible[next_trip_id_first_vehicle-1] and
        trip[prev_trip_id_second_vehicle-1].is_compatible[curr_trip_id_first_vehicle-1] and
        trip[curr_trip_id_first_vehicle-1].is_compatible[next_trip_id_second_vehicle-1])
        return true;
    else
        return false;
}

// Function to check if new charging stations need to be opened to make the exchange feasible
double evaluation::make_exchange_charge_feasible(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, int u, int v, int k, int l)
{
    // Trip k of vehicle u and trip l of vehicle v can be exchanged
    // The function checks if the exchange requires new charging stations to be opened
    int prev_trip_id_first_vehicle = vehicle[u].trip_id[k-1];
    int curr_trip_id_first_vehicle = vehicle[u].trip_id[k];
    int next_trip_id_first_vehicle = vehicle[u].trip_id[k+1];

    int prev_trip_id_second_vehicle = vehicle[v].trip_id[l-1];
    int curr_trip_id_second_vehicle = vehicle[v].trip_id[l];
    int next_trip_id_second_vehicle = vehicle[v].trip_id[l+1];

    // Check if the exchanges is feasible using compatibility matrices
    if (trip[prev_trip_id_first_vehicle-1].is_compatible[curr_trip_id_second_vehicle-1] and
        trip[curr_trip_id_second_vehicle-1].is_compatible[next_trip_id_first_vehicle-1] and
        trip[prev_trip_id_second_vehicle-1].is_compatible[curr_trip_id_first_vehicle-1] and
        trip[curr_trip_id_first_vehicle-1].is_compatible[next_trip_id_second_vehicle-1])
        return true;
    else
        return false;
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

    double current_cost = trip[prev_trip_id-1].deadhead_distance[curr_trip_id-1] * DEADHEAD_COST_FACTOR
        + trip[curr_trip_id-1].deadhead_distance[next_trip_id-1] * DEADHEAD_COST_FACTOR;

    double new_cost = trip[prev_trip_id-1].deadhead_distance[new_trip_id-1] * DEADHEAD_COST_FACTOR
        + trip[new_trip_id-1].deadhead_distance[next_trip_id-1] * DEADHEAD_COST_FACTOR;

    return new_cost - current_cost;
}