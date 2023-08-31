#include "helpers.h"

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

    // Additional debug info that prints members of each trip
    logger.log(LogLevel::Debug, "Printing trip data (Trip ID, Start stop, End stop, Start time, End time, Distance)");
    for (int i = 0; i<num_trips; ++i) {
        logger.log(LogLevel::Debug, std::to_string(trip[i].id)+" "+
                std::to_string(trip[i].start_stop)+" "+std::to_string(trip[i].end_stop)+" "+
                std::to_string(trip[i].start_time)+" "+std::to_string(trip[i].end_time)+" "+
                std::to_string(trip[i].distance));
    }
}

void preprocessing::read_input_data(std::string instance, std::vector<Trip>& trip, std::vector<Terminal>& terminal,
        std::vector<Vehicle>& vehicle, int& num_trips, int& num_terminals, Logger& logger)
{
    // Read input data on trips and stops and initialize bus rotations
    preprocessing::read_trip_data(instance, trip, num_trips, logger);
    // read_terminal_data(instance, trip, num_terminals);

    /* initialize_bus_rotations(instance, );
    initalize_charge_stations(); */
}

void test::testing() {
    std::cout << "Testing" << std::endl;
}