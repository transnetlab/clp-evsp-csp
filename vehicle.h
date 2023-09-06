#ifndef EBUS_VNS_VEHICLE_H
#define EBUS_VNS_VEHICLE_H

#include "constants.h"
#include <string>
#include <vector>
#include <iostream>

//Create a trip class which contains relevant GTFS details
class Trip {
public:
    int id;  // Trip ID
    int start_stop;  // GTFS stop ID
    int end_stop;  // GTFS stop ID
    int start_time;  // Minutes since midnight
    int end_time;  // Minutes since midnight
    double distance;  // Distance in km
    std::vector<bool> is_compatible;  // Booleans to check if a trip is compatible with another (includes depot 'trips')
    std::vector<double> deadhead_distance;  // Vector of deadhead distances (includes depot 'trips')
    std::vector<double> idle_time;  // Vector of idle times (includes depot 'trips')

    // Constructor
    Trip() {
        // Nothing to do here
    }

    Trip(int id, int start_stop, int end_stop, int start_time, int end_time, double distance) {
        this->id = id;
        this->start_stop = start_stop;
        this->end_stop = end_stop;
        this->start_time = start_time;
        this->end_time = end_time;
        this->distance = distance;
    }

    //Destructor
    ~Trip() {
        is_compatible.clear();
        deadhead_distance.clear();
        idle_time.clear();
    }
};

//Create a stops class with potential charging locations
class Terminal {
public:
    int id;  // Terminal ID
    int stop_id;  // GTFS stop ID
    int trip_id;  // Augmented trip ID for populating rotations
    bool is_depot;  // True if the stop is a depot
    bool is_charge_station;  // True if the stop is a charging station

    // Constructor
    Terminal() {
        // Nothing to do here
    };

    Terminal(int stop_id, int trip_id, bool is_depot, bool is_station) {
        this->stop_id = stop_id;
        this->trip_id = trip_id;
        this->is_depot = is_depot;
        this->is_charge_station = is_station;
    }

    // Destructor
    ~Terminal() {
        // Nothing to do here
    }
};

//Create a vehicle class which stores bus rotation details
class Vehicle {
public:
    int id;  // Vehicle ID
    int num_trips;  // Number of trips in the rotation
    std::vector<int> trip_id;  // First and last trips are aliases for depots
    double deadhead_cost;  // Cost of deadheading in the rotation

    // Constructor
    Vehicle() {
        // Nothing to do here
    };

    Vehicle(int id) {
        this->id = id;
        num_trips = 0;
        deadhead_cost = 0.0;
    }

    // Desctructor
    ~Vehicle() {
        trip_id.clear();
    }

    // Function to calculate the deadheading costs in the rotation
    void calculate_deadhead_cost(const std::vector<Trip> &trip) {
        // Calculate the deadhead cost of the rotation
        deadhead_cost = 0.0;
        int curr_trip_index, next_trip_index;
        for (int i = 0; i < trip_id.size() - 1; ++i) {
            curr_trip_index = trip_id[i] - 1;
            next_trip_index = trip_id[i + 1] - 1;
            deadhead_cost += trip[curr_trip_index].deadhead_distance[next_trip_index] * DEADHEAD_COST_FACTOR;
        }
    }

    // Private function to update num_trips
    void update_num_trips() {
        num_trips = static_cast<int>(trip_id.size());
    }

    // Print members of the class
    void print_members() {
        std::cout << "Vehicle ID: " << id << std::endl;
        std::cout << "Number of trips: " << trip_id.size() << std::endl;
        std::cout << "Trip IDs: ";
        for (int i = 0; i < trip_id.size(); ++i) {
            std::cout << trip_id[i] << " ";
        }
        std::cout << std::endl;
        std::cout << "Deadhead cost: " << deadhead_cost << std::endl;
    }
};

#endif //EBUS_VNS_VEHICLE_H
