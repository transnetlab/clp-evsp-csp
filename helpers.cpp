#include "helpers.h"

// Function to read relevant GTFS trip data
void preprocessing::read_trip_data(std::string instance, std::vector<Trip>& trip, int& num_trips, Logger& logger)
{
    // Read trip data from file
    logger.log(LogLevel::Info, "Reading trip data from file...");
    std::ifstream input_file("./data/"+instance+"/trip_data.txt");
    if (!input_file.is_open()) {
        logger.log(LogLevel::Error, "Unable to open trip data file");
        exit(1); // Terminate with error
    }

    // Read each line and populate members of the trip class
    num_trips = 0;
    std::string line;
    while (std::getline(input_file, line)) {
        Trip temp_trip;
        ++num_trips;
        std::istringstream line_stream(line);
        line_stream >> temp_trip.id;
        line_stream >> temp_trip.start_stop;
        line_stream >> temp_trip.end_stop;
        line_stream >> temp_trip.start_time;
        line_stream >> temp_trip.end_time;
        line_stream >> temp_trip.distance;

        line_stream.clear();
        trip.push_back(temp_trip);
    }

    input_file.close(); // Close the input_file

    // Log the data read
    logger.log(LogLevel::Info, "Trip data read successfully");
    logger.log(LogLevel::Info, "Number of trips: "+std::to_string(num_trips));
}

// Function that reads data on terminal stops of routes
void preprocessing::read_terminal_data(std::string instance, std::vector<Terminal>& terminal, int& num_terminals,
        Logger& logger)
{
    // Read terminal data from file
    logger.log(LogLevel::Info, "Reading terminal data from file...");
    std::ifstream input_file("./data/"+instance+"/terminal_data.txt");
    if (!input_file.is_open()) {
        logger.log(LogLevel::Error, "Unable to open terminal data file");
        exit(1); // Terminate with error
    }

    // Read each line and populate members of the terminal class
    num_terminals = 0;
    std::string line;
    while (std::getline(input_file, line)) {
        Terminal temp_terminal;
        ++num_terminals;
        std::istringstream line_stream(line);
        line_stream >> temp_terminal.stop_id;
        line_stream >> temp_terminal.is_depot;
        line_stream >> temp_terminal.is_charge_station;

        line_stream.clear();
        terminal.push_back(temp_terminal);
    }

    input_file.close(); // Close the input_file

    // Log the data read
    logger.log(LogLevel::Info, "Terminal data read successfully");
    logger.log(LogLevel::Info, "Number of terminals: "+std::to_string(num_terminals));
}

// Function to augment trips with depot stops
void preprocessing::create_depot_trips(std::vector<Trip>& trip, std::vector<Terminal>& terminal, int& num_trips,
        int& num_terminals, Logger& logger)
{
    // Augment trips with depot stops
    logger.log(LogLevel::Info, "Augmenting trips with depot stops...");

    // Add depot stops to the trip vector and update the terminal members with new trip IDs
    for (auto& current_terminal : terminal) {
        if (current_terminal.is_depot) {
            // Add a new trip to the trip vector
            Trip temp_trip;
            temp_trip.id = num_trips+1;
            temp_trip.start_stop = current_terminal.stop_id;
            temp_trip.end_stop = current_terminal.stop_id;
            temp_trip.start_time = 0;
            temp_trip.end_time = 0;
            temp_trip.distance = 0.0;

            current_terminal.trip_id = temp_trip.id;  // Update the trip_id data of the terminal

            // Add the new trip to the trip vector
            trip.push_back(temp_trip);
            ++num_trips;
        }
    }

    // Log the set of augmented trips
    logger.log(LogLevel::Info, "Number of trips after augmentation: "+std::to_string(num_trips));
    logger.log(LogLevel::Debug, "Printing trip data (Trip ID, Start stop, End stop, Start time, End time, Distance)");
    for (const auto& current_trip : trip) {
        logger.log(LogLevel::Debug, std::to_string(current_trip.id)+" "+std::to_string(current_trip.start_stop)+" "+
                std::to_string(current_trip.end_stop)+" "+std::to_string(current_trip.start_time)+" "+
                std::to_string(current_trip.end_time)+" "+std::to_string(current_trip.distance));
    }

    // Additional debug info that prints members of each terminal
    logger.log(LogLevel::Debug, "Printing terminal data (Stop ID, Trip ID, Is depot?, Is station?)");
    for (const auto& current_terminal : terminal) {
        logger.log(LogLevel::Debug,
                std::to_string(current_terminal.stop_id)+" "+std::to_string(current_terminal.trip_id)+" "+
                        std::to_string(current_terminal.is_depot)+" "
                        +std::to_string(current_terminal.is_charge_station));
    }
}

// Function to read data on trip pairs
void preprocessing::read_trip_pair_data(std::string instance, std::vector<Trip>& trip, int& num_trips, Logger& logger)
{
    // Read trip pair data from file
    logger.log(LogLevel::Info, "Reading trip pair data from file...");

    std::ifstream input_file_compatibility("compatibility_matrix.txt");
    std::ifstream input_file_deadheading("deadhead_distance_matrix.txt");
    std::ifstream input_file_idle_time("idle_time_matrix.txt");

    // Terminate with error if the files cannot be opened
    if (!input_file_compatibility.is_open()) {
        std::cout << "Unable to open compatibility matrix data";
        exit(1); // terminate with error
    }

    if (!input_file_deadheading.is_open()) {
        std::cout << "Unable to open deadheading matrix data";
        exit(1); // terminate with error
    }

    if (!input_file_idle_time.is_open()) {
        std::cout << "Unable to open idle time matrix data";
        exit(1); // terminate with error
    }

    // Read each line and populate members of the trip class
    for (auto& current_trip : trip) {
        current_trip.is_compatible.resize(num_trips, false);
        current_trip.deadhead_distance.resize(num_trips, 0.0);
        current_trip.idle_time.resize(num_trips, 0.0);
    }

    int value;  // Temporary variable to store the value read from the file
    for (auto& current_trip : trip) {
        for (int i = 0; i<num_trips; ++i) {
            input_file_compatibility >> value;
            current_trip.is_compatible[i] = (value==1) ? true : false;
            input_file_deadheading >> current_trip.deadhead_distance[i];
            input_file_idle_time >> current_trip.idle_time[i];
        }
    }
    input_file_compatibility.close(); // Close the files
    input_file_deadheading.close();
    input_file_idle_time.close();

    // Logging trip pair data
    logger.log(LogLevel::Info, "Trip pair data read successfully");
    logger.log(LogLevel::Debug,
            "Printing trip pair data (Current trip, Next trip, Is compatible?, Deadhead distance, Idle time)");
    for (const auto& current_trip : trip) {
        for (int i = 0; i<num_trips; ++i) {
            // Print the current trip and the next trip as an ordered pair
            logger.log(LogLevel::Debug, std::to_string(current_trip.id)+" "+std::to_string(i+1)+" "+
                    std::to_string(current_trip.is_compatible[i])+" "
                    +std::to_string(current_trip.deadhead_distance[i]));
        }
    }
}

// Function to initialize bus rotations from the solution to the concurrent scheduler algorithm
void preprocessing::initialize_vehicle_rotations(std::string instance, std::vector<Vehicle>& vehicle, Logger& logger)
{
    // Initialize bus rotations from the solution to the concurrent scheduler algorithm
    logger.log(LogLevel::Info, "Initializing bus rotations from the concurrent scheduler solution...");
}

// Function to initialize charging stations from the solution to the concurrent scheduler algorith
void preprocessing::initialize_charge_locations(std::string instance, std::vector<Terminal>& terminal,
        int& num_terminals, Logger& logger)
{
    // Initialize charging stations from the solution to the concurrent scheduler algorithm
    logger.log(LogLevel::Info, "Initializing charging stations from the concurrent scheduler solution...");

}

// Function to read inputs to the model including GTFS data and initial rotations
void preprocessing::initialize_inputs(std::string instance, std::vector<Trip>& trip, std::vector<Terminal>& terminal,
        std::vector<Vehicle>& vehicle, int& num_trips, int& num_terminals,
        Logger& logger)
{
    // Read input data on trips and stops and initialize bus rotations
    preprocessing::read_trip_data(instance, trip, num_trips, logger);
    preprocessing::read_terminal_data(instance, terminal, num_terminals, logger);
    preprocessing::create_depot_trips(trip, terminal, num_trips, num_terminals, logger);

    // Populate compatibility, deadheading, and idle time information of trip pairs
    preprocessing::read_trip_pair_data(instance, trip, num_trips, logger);

    // Initialize bus rotation and charging stations from the solution to the concurrent scheduler algorithm
    preprocessing::initialize_vehicle_rotations(instance, vehicle, logger);
    preprocessing::initialize_charge_locations(instance, terminal, num_terminals, logger);
}

void evaluation::calculate_objective(std::vector<Trip>& trip, std::vector<Terminal>& terminal, std::vector<Vehicle>& vehicle,
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
    double vehicle_acquisition_cost = VEHICLE_COST * vehicle.size();

    // Calculate variable cost of deadheading
    double deadhead_cost = 0.0;
    for (auto& current_vehicle : vehicle) {
        current_vehicle.calculate_deadhead_cost(trip);
        deadhead_cost += current_vehicle.deadhead_cost;
    }

    // Calculate the total cost and log the cost components
    double total_cost = location_cost + vehicle_acquisition_cost + deadhead_cost;
    logger.log(LogLevel::Info, "Fixed cost of opening charging stations: "+std::to_string(location_cost));
    logger.log(LogLevel::Info, "Fixed cost of bus acquisition: "+std::to_string(vehicle_acquisition_cost));
    logger.log(LogLevel::Info, "Variable cost of deadheading: "+std::to_string(deadhead_cost));
    logger.log(LogLevel::Info, "Total cost: "+std::to_string(total_cost));
}