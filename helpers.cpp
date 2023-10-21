#include "helpers.h"

// Function to read relevant GTFS trip data
void preprocessing::read_trip_data(std::string instance, std::vector<Trip>& trip, int& num_trips, Logger& logger)
{
    // Read trip data from file
    logger.log(LogLevel::Info, "Reading trip data from file...");
    std::ifstream input_file("../data/"+instance+"/trip_data.txt");
    if (!input_file.is_open()) {
        logger.log(LogLevel::Error, "Unable to open trip data file");
        postprocessing::write_output_data("Error: Unable to open trip data file", logger);
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
        line_stream >> temp_trip.start_terminal;
        line_stream >> temp_trip.end_terminal;
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

    // Check if trip IDs are continuous. If not, throw an error and exit
    for (int i = 0; i<num_trips; ++i) {
        if (trip[i].id!=i+1) {
            logger.log(LogLevel::Error, "Trip IDs are not continuous");
            postprocessing::write_output_data("Error: Trip IDs are not continuous", logger);
            exit(1);
        }
    }
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
        postprocessing::write_output_data("Error: Unable to open terminal data file", logger);
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
void preprocessing::create_depot_trips(std::vector<Trip>& trip, std::vector<Terminal>& terminal, int& num_trips,
        int& num_augmented_trips,
        Logger& logger)
{
    // Augment trips with depot stops
    logger.log(LogLevel::Info, "Augmenting trips with depot stops...");

    // Add depot stops to the trip vector and update the terminal members with new trip IDs
    num_augmented_trips = num_trips;
    for (auto& curr_terminal : terminal) {
        // Add a new trip to the trip vector
        Trip temp_trip;
        temp_trip.id = num_augmented_trips+1;
        temp_trip.start_terminal = curr_terminal.id;
        temp_trip.end_terminal = curr_terminal.id;
        temp_trip.start_time = 0;
        temp_trip.end_time = 0;
        temp_trip.distance = 0.0;

        // Add the new trip to the trip vector
        trip.push_back(temp_trip);
        ++num_augmented_trips;

        // Double check if trip IDs are consistent
        if (curr_terminal.trip_id!=temp_trip.id) {
            logger.log(LogLevel::Error, "Mismatch in trip IDs for depots found");
            postprocessing::write_output_data("Error: Mismatch in trip IDs for depots found", logger);
            exit(1);
        }
    }
    logger.log(LogLevel::Info, "Number of trips after augmentation: "+std::to_string(num_augmented_trips));
}

// Function to read data on trip pairs
void preprocessing::read_trip_pair_data(std::string instance, std::vector<Trip>& trip, int& num_augmented_trips,
        Logger& logger)
{
    // Read trip pair data from file
    logger.log(LogLevel::Info, "Reading trip pair data from file...");

    std::ifstream input_file_compatibility("../data/"+instance+"/compatibility_matrix.txt");
    std::ifstream input_file_deadheading("../data/"+instance+"/deadhead_distance_matrix.txt");
    std::ifstream input_file_idle_time("../data/"+instance+"/idle_time_matrix.txt");

    // Terminate with error if the files cannot be opened
    if (!input_file_compatibility.is_open()) {
        logger.log(LogLevel::Error, "Unable to open compatibility matrix data");
        postprocessing::write_output_data("Error: Unable to open compatibility matrix data", logger);
        exit(1); // terminate with error
    }

    if (!input_file_deadheading.is_open()) {
        logger.log(LogLevel::Error, "Unable to open deadheading matrix data");
        postprocessing::write_output_data("Error: Unable to open deadheading matrix data", logger);
        exit(1); // terminate with error
    }

    if (!input_file_idle_time.is_open()) {
        logger.log(LogLevel::Error, "Unable to open idle time matrix data");
        postprocessing::write_output_data("Error: Unable to open idle time matrix data", logger);
        exit(1); // terminate with error
    }

    // Read each line and populate members of the trip class
    for (auto& curr_trip : trip) {
        curr_trip.is_compatible.resize(num_augmented_trips, false);
        curr_trip.deadhead_distance.resize(num_augmented_trips, 0.0);
        curr_trip.idle_time.resize(num_augmented_trips, 0.0);
    }

    int value;  // Temporary variable to store the value read from the file
    for (auto& curr_trip : trip) {
        for (int i = 0; i<num_augmented_trips; ++i) {
            input_file_compatibility >> value;
            curr_trip.is_compatible[i] = (value==1) ? true : false;
            input_file_deadheading >> curr_trip.deadhead_distance[i];
            input_file_idle_time >> curr_trip.idle_time[i];
        }
    }
    input_file_compatibility.close(); // Close the files
    input_file_deadheading.close();
    input_file_idle_time.close();

    // Logging trip pair data
    logger.log(LogLevel::Info, "Trip pair data read successfully");
    logger.log(LogLevel::Verbose,
            "Printing compatible trip pair data (Current trip, Next trip, Deadhead distance, Idle time)");
    for (const auto& curr_trip : trip) {
        for (int i = 0; i<num_augmented_trips; ++i) {
            // Print the current trip and the next trip as an ordered pair only if the trip pair is compatible
            if (curr_trip.is_compatible[i])
                logger.log(LogLevel::Verbose, std::to_string(curr_trip.id)+" "+std::to_string(i+1)+" "
                        +std::to_string(curr_trip.deadhead_distance[i])+" "
                        +std::to_string(curr_trip.idle_time[i]));
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
        logger.log(LogLevel::Error, "Unable to open vehicle rotations file");
        postprocessing::write_output_data("Error: Unable to open vehicle rotations file", logger);
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
            logger.log(LogLevel::Warning, "Found continuity issue in IDs of initial vehicle rotations");

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

// Function to log input data
void preprocessing::log_input_data(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal,
        Logger& logger)
{
    // Log the set of trips including the augmented depot trips
    logger.log(LogLevel::Debug,
            "Printing trip data (Trip ID, Start terminal, End terminal, Start time, End time, Distance)");
    for (const auto& curr_trip : trip) {
        logger.log(LogLevel::Debug,
                std::to_string(curr_trip.id)+" "+std::to_string(curr_trip.start_terminal)+" "+
                        std::to_string(curr_trip.end_terminal)+" "+std::to_string(curr_trip.start_time)+" "+
                        std::to_string(curr_trip.end_time)+" "+std::to_string(curr_trip.distance));
    }

    // Additional debug info that prints members of each terminal
    logger.log(LogLevel::Debug, "Printing terminal data (Stop ID, Trip ID, Is depot?, Is charging station?)");
    for (const auto& curr_terminal : terminal) {
        logger.log(LogLevel::Debug,
                std::to_string(curr_terminal.id)+" "+curr_terminal.stop_id+" "
                        +std::to_string(curr_terminal.trip_id)+" "+
                        std::to_string(curr_terminal.is_depot)+" "
                        +std::to_string(curr_terminal.is_charge_station));
    }

    // Debug info for vehicle rotations
    logger.log(LogLevel::Debug, "Printing vehicle rotations (Vehicle ID, Trip IDs)");
    std::string trip_list;
    for (const auto& curr_vehicle : vehicle)
        logger.log(LogLevel::Debug, std::to_string(curr_vehicle.id)+" "+vector_to_string(curr_vehicle.trip_id));
}

// Function to read inputs to the model including GTFS data and initial rotations
void preprocessing::initialize_inputs(std::string instance, std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, int& num_trips, int& num_augmented_trips, int& num_terminals, Logger& logger)
{
    logger.log(LogLevel::Info, "Initializing inputs to the model...");

    // Read input data on trips and stops and initialize bus rotations
    preprocessing::read_trip_data(instance, trip, num_trips, logger);
    preprocessing::read_terminal_data(instance, terminal, num_terminals, logger);
    preprocessing::create_depot_trips(trip, terminal, num_trips, num_augmented_trips, logger);

    // Populate compatibility, deadheading, and idle time information of trip pairs
    preprocessing::read_trip_pair_data(instance, trip, num_augmented_trips, logger);

    // Initialize bus rotation and charging stations from the solution to the concurrent scheduler algorithm
    preprocessing::initialize_vehicle_rotations(instance, vehicle, logger);

    // Log the input data
    preprocessing::log_input_data(vehicle, trip, terminal, logger);

    logger.log(LogLevel::Info, "Inputs to the model initialized successfully");
}

// Sanity check for solutions: (1) Single bus for each trip (2) All trips are included (3) Charge feasibility
void postprocessing::check_solution(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, int num_trips, Logger& logger)
{
    logger.log(LogLevel::Info, "Performing sanity checks on the solution...");

    // Check if each trip is assigned to exactly one vehicle
    logger.log(LogLevel::Info, "Checking if each trip is assigned to exactly one vehicle...");
    std::vector<bool> is_trip_assigned(num_trips, false);
    int curr_trip;
    for (const auto& curr_vehicle : vehicle) {
        for (int i = 1; i<curr_vehicle.trip_id.size()-1; ++i) {
            curr_trip = curr_vehicle.trip_id[i];
            if (is_trip_assigned[curr_trip-1]) {
                logger.log(LogLevel::Error,
                        "Trip ID"+std::to_string(curr_trip)+" is in more than one vehicle. Sanity check status: Fail");
                postprocessing::write_output_data(
                        "Error: Trip ID"+std::to_string(curr_trip)+" is in more than one vehicle", logger);
                exit(1);
            }
            is_trip_assigned[curr_trip-1] = true;
        }
    }
    logger.log(LogLevel::Info, "Each trip is assigned to exactly one vehicle");

    // Check if all trips are assigned to a vehicle
    logger.log(LogLevel::Info, "Checking if all trips are assigned to a vehicle...");
    for (int i = 0; i<num_trips; ++i) {
        if (!is_trip_assigned[i]) {
            logger.log(LogLevel::Error,
                    "Trip "+std::to_string(i+1)+" is not assigned to any vehicle. Sanity check status: Fail");
            postprocessing::write_output_data(
                    "Error: Trip "+std::to_string(i+1)+" is not assigned to any vehicle in the final solution", logger);
            exit(1);
        }
    }
    logger.log(LogLevel::Info, "All trips are assigned to a vehicle");

    // Check if the rotations are charge feasible
    logger.log(LogLevel::Info, "Checking if the solution is charge feasible...");
    std::vector<std::vector<int>> rotations;
    for (const auto& curr_vehicle : vehicle)
        rotations.push_back(curr_vehicle.trip_id);
    if (!evaluation::are_rotations_charge_feasible(trip, terminal, rotations)) {
        logger.log(LogLevel::Error, "Solution is not charge feasible. Sanity check status: Fail");
        postprocessing::write_output_data("Error: Final solution is not charge feasible", logger);
        exit(1);
    }
    logger.log(LogLevel::Info, "Solution is charge feasible");
    logger.log(LogLevel::Info, "Sanity check status: Pass");
}

// Function that saves an error message in the summary file if the code does not run to completion
void postprocessing::write_output_data(std::string message, Logger& logger)
{
    // Write the error message
    std::ofstream summary_file("../output/Summary.txt", std::ios_base::app);
    if (!summary_file.is_open()) {
        logger.log(LogLevel::Error, "Unable to open summary file");
        exit(1); // Terminate with error
    }

    summary_file << message << std::endl;
    summary_file.close();
}

// Function that saves the instance name
void postprocessing::write_output_data(std::string instance, std::time_t curr_time, Logger& logger)
{
    // Write the error message
    std::ofstream summary_file("../output/Summary.txt", std::ios_base::app);
    if (!summary_file.is_open()) {
        logger.log(LogLevel::Error, "Unable to open summary file");
        exit(1); // Terminate with error
    }

    summary_file << instance << ", " << std::put_time(std::localtime(&curr_time), "%b %d %H:%M:%S") << ", ";
    summary_file.close();
}

// Function to write the summary outputs to a file
void postprocessing::write_output_data(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, int num_trips, int num_terminals, double runtime, Logger& logger)
{
    // Write the output data to a file
    logger.log(LogLevel::Info, "Writing output summary to file...");
    std::ofstream summary_file("../output/Summary.txt", std::ios_base::app);
    if (!summary_file.is_open()) {
        logger.log(LogLevel::Error, "Unable to open summary file");
        exit(1); // Terminate with error
    }

    // Find the number of charging stations
    int num_charging_stations = 0;
    for (const auto& curr_terminal : terminal) {
        if (curr_terminal.is_charge_station)
            ++num_charging_stations;
    }

    // Write the cost of the solution and the problem settings
    double cost = evaluation::calculate_objective(vehicle, trip, terminal, logger);
    summary_file << num_trips << ", " << num_terminals << ", " << vehicle.size() << ", " << num_charging_stations
                 << ", " << std::fixed << std::setprecision(2) << cost << ", " << runtime;

    // Write the constants used in the model
    /*summary_file << " (Settings: Vehicle cost: " << VEHICLE_COST << ", " << "Charge station cost: " << CHARGE_LOC_COST
                 << ", " << "Cost per km: " << COST_PER_KM << ", " << "Max charge level: " << MAX_CHARGE_LEVEL << ", "
                 << "Min charge level: " << MIN_CHARGE_LEVEL << ", " << "Charge rate: " << CHARGE_RATE << ", "
                 << "Max energy per min: " << MAX_ENERGY_PER_MIN << ", " << "Energy per km: " << ENERGY_PER_KM << ", "
                 << "Num price intervals: " << NUM_PRICE_INTERVALS << ", " << "Energy left interval: "
                 << array_to_string(ENERGY_LEFT_INTERVAL) << ", " << "Energy price: " << array_to_string(ENERGY_PRICE)
                 << ", " << "Power capacity price: " << POWER_CAPACITY_PRICE << ", " << "Idle time threshold: "
                 << IDLE_TIME_THRESHOLD << ", " << "Perform three exchanges: " << PERFORM_THREE_EXCHANGES << ", "
                 << "Perform two shifts: " << PERFORM_TWO_SHIFTS << ", " << "Swap charge stations: "
                 << SWAP_CHARGE_STATIONS << ", " << "Shift all trips threshold: " << SHIFT_ALL_TRIPS_THRESHOLD << ")" << std::endl;*/

    summary_file.close();
}

