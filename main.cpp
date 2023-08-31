#include "constants.h"
#include "logger.h"
#include "vehicle.h"
#include "helpers.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <sstream>
#include <chrono>
#include <algorithm>
#include <iomanip>

int main() {
    std::string instance = "Ann_Arbor";
    Logger logger("./output/" + instance + "_log.txt", true);
    logger.set_log_level_threshold(LogLevel::Info);

    // Initialize variables
    int num_trips, num_terminals;  // Number of trips and terminals in the network
    std::vector<Trip> trip;  // Vector of trips
    std::vector<Terminal> terminal;  // Vector of terminals
    std::vector<Vehicle> vehicle;  // Vector of vehicles

    // Read input data on trips and stops and initialize bus rotations
    preprocessing::read_input_data(instance, trip, terminal, vehicle, num_trips, num_terminals, logger);

    // VNS algorithm
    test::testing();

    return 0;
}
