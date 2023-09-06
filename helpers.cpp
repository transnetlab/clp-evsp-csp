#include "helpers.h"

// Function to read relevant GTFS trip data
void preprocessing::read_trip_data(std::string instance, std::vector<Trip>& trip, int& num_trips, Logger& logger)
{
    // Read trip data from file
    logger.log(LogLevel::Info, "Reading trip data from file...");
    std::ifstream input_file("../data/"+instance+"/trip_data.txt");
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
    std::ifstream input_file("../data/"+instance+"/terminal_data.txt");
    if (!input_file.is_open()) {
        logger.log(LogLevel::Error, "Unable to open terminal data file");
        exit(1); // Terminate with error
    }

    // Read each line and populate members of the terminal class
    num_terminals = 0;
    std::string line;
    int value;
    while (std::getline(input_file, line)) {
        Terminal temp_terminal;
        ++num_terminals;
        std::istringstream line_stream(line);
        line_stream >> temp_terminal.id;
        line_stream >> temp_terminal.stop_id;
        line_stream >> temp_terminal.trip_id;
        line_stream >> value;
        temp_terminal.is_depot = (value==1) ? true : false;
        line_stream >> value;
        temp_terminal.is_charge_station = (value==1) ? true : false;

        line_stream.clear();
        terminal.push_back(temp_terminal);
    }

    input_file.close(); // Close the input_file

    // Log the data read
    logger.log(LogLevel::Info, "Terminal data read successfully");
    logger.log(LogLevel::Info, "Number of terminals: "+std::to_string(num_terminals));
}

// Function to augment trips with depot stops
void preprocessing::create_depot_trips(std::vector<Trip>& trip, std::vector<Terminal>& terminal, int& num_trips, Logger& logger)
{
    // Augment trips with depot stops
    logger.log(LogLevel::Info, "Augmenting trips with depot stops...");

    // Add depot stops to the trip vector and update the terminal members with new trip IDs
    for (auto& current_terminal : terminal) {
        // Add a new trip to the trip vector
        Trip temp_trip;
        temp_trip.id = num_trips+1;
        temp_trip.start_stop = current_terminal.stop_id;
        temp_trip.end_stop = current_terminal.stop_id;
        temp_trip.start_time = 0;
        temp_trip.end_time = 0;
        temp_trip.distance = 0.0;

        // Add the new trip to the trip vector
        trip.push_back(temp_trip);
        ++num_trips;

        // Double check if trip IDs are consistent
        if (current_terminal.trip_id!=temp_trip.id) {
            logger.log(LogLevel::Error, "Mismatch in trip IDs for depots found");
            exit(1);
        }
    }
    logger.log(LogLevel::Info, "Number of trips after augmentation: "+std::to_string(num_trips));
}

// Function to read data on trip pairs
void preprocessing::read_trip_pair_data(std::string instance, std::vector<Trip>& trip, int& num_trips, Logger& logger)
{
    // Read trip pair data from file
    logger.log(LogLevel::Info, "Reading trip pair data from file...");

    std::ifstream input_file_compatibility("../data/"+instance+"/compatibility_matrix.txt");
    std::ifstream input_file_deadheading("../data/"+instance+"/deadhead_distance_matrix.txt");
    std::ifstream input_file_idle_time("../data/"+instance+"/idle_time_matrix.txt");

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
            // Print the current trip and the next trip as an ordered pair only if the trip pair is compatible
            if (current_trip.is_compatible[i])
                logger.log(LogLevel::Debug, std::to_string(current_trip.id)+" "+std::to_string(i+1)+" "
                        +std::to_string(current_trip.is_compatible[i])+" "+std::to_string(current_trip.deadhead_distance[i])
                        +" "+std::to_string(current_trip.idle_time[i]));
        }
    }
}

// Function to initialize bus rotations from the solution to the concurrent scheduler algorithm
void preprocessing::initialize_vehicle_rotations(std::string instance, std::vector<Vehicle>& vehicle, Logger& logger)
{
    // Initialize bus rotations from the solution to the concurrent scheduler algorithm
    logger.log(LogLevel::Info, "Initializing vehicle rotations from the concurrent scheduler solution...");

    // Read the initial vehicle rotations from a file
    std::ifstream input_file("../data/"+instance+"/initial_vehicle_rotations.txt");
    if (!input_file.is_open()) {
        std::cout << "Unable to open vehicle rotations file";
        exit(1); // terminate with error
    }

    // Save the data to the vehicle vector
    std::string line;
    int count = 0;
    int temp_trip_id;  // Variable used to populate trip IDs one at a time
    while (std::getline(input_file, line)) {
        Vehicle temp_vehicle;
        ++count;
        std::istringstream line_stream(line);
        line_stream >> temp_vehicle.id;
        // If count does not match the temp_vehicle id issue a warning
        if (count!=temp_vehicle.id)
            logger.log(LogLevel::Warning, "Found continuity ID issue in initial vehicle rotations");

        // Populate other trip ID elements of the row
        while (line_stream >> temp_trip_id)
            temp_vehicle.trip_id.push_back(temp_trip_id);

        // Add the data to vehicle vector
        vehicle.push_back(temp_vehicle);
    }

    // Close the input file
    input_file.close();

    logger.log(LogLevel::Info, "Vehicle rotations read successfully");
    logger.log(LogLevel::Info, "Number of vehicles: "+std::to_string(count));
}

void preprocessing::log_input_data(std::vector<Trip>& trip, std::vector<Terminal>& terminal,
        std::vector<Vehicle>& vehicle, Logger& logger)
{
    // Log the set of augmented trips
    logger.log(LogLevel::Debug, "Printing trip data (Trip ID, Start stop, End stop, Start time, End time, Distance)");
    for (const auto& current_trip : trip) {
        logger.log(LogLevel::Debug, std::to_string(current_trip.id)+" "+std::to_string(current_trip.start_stop)+" "+
                std::to_string(current_trip.end_stop)+" "+std::to_string(current_trip.start_time)+" "+
                std::to_string(current_trip.end_time)+" "+std::to_string(current_trip.distance));
    }

    // Additional debug info that prints members of each terminal
    logger.log(LogLevel::Debug, "Printing terminal data (Stop ID, Trip ID, Is depot?, Is charging station?)");
    for (const auto& current_terminal : terminal) {
        logger.log(LogLevel::Debug, std::to_string(current_terminal.id)+" "+std::to_string(current_terminal.stop_id)+" "
                +std::to_string(current_terminal.trip_id)+" "+
                std::to_string(current_terminal.is_depot)+" "
                +std::to_string(current_terminal.is_charge_station));
    }

    // Debug info for vehicle rotations
    logger.log(LogLevel::Debug, "Printing vehicle rotations (Vehicle ID, Trip IDs)");
    std::string trip_list;
    for (const auto& current_vehicle : vehicle) {
        trip_list = "";
        for (const auto& current_trip : current_vehicle.trip_id)
            trip_list += std::to_string(current_trip)+" ";
        logger.log(LogLevel::Debug, std::to_string(current_vehicle.id)+" "+trip_list);
    }
}

// Function to read inputs to the model including GTFS data and initial rotations
void preprocessing::initialize_inputs(std::string instance, std::vector<Trip>& trip, std::vector<Terminal>& terminal,
        std::vector<Vehicle>& vehicle, int& num_trips, int& num_terminals,
        Logger& logger)
{
    // Read input data on trips and stops and initialize bus rotations
    preprocessing::read_trip_data(instance, trip, num_trips, logger);
    preprocessing::read_terminal_data(instance, terminal, num_terminals, logger);
    preprocessing::create_depot_trips(trip, terminal, num_trips, logger);

    // Populate compatibility, deadheading, and idle time information of trip pairs
    preprocessing::read_trip_pair_data(instance, trip, num_trips, logger);

    // Initialize bus rotation and charging stations from the solution to the concurrent scheduler algorithm
    preprocessing::initialize_vehicle_rotations(instance, vehicle, logger);

    // Log the input data
    preprocessing::log_input_data(trip, terminal, vehicle, logger);
}
