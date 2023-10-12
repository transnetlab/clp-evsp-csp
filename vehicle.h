#ifndef EBUS_VNS_VEHICLE_H
#define EBUS_VNS_VEHICLE_H

#include "constants.h"
#include "logger.h"
#include <string>
#include <vector>
#include <iostream>
#include <sstream>

template<typename T>
std::string vector_to_string(const std::vector<T>& input_vector)
{
    std::stringstream ss;
    for (const T& element : input_vector) {
        ss << element << " ";
    }
    return ss.str();
}

//Create a stops class with potential charging locations
class Terminal {
public:
    int id;  // Terminal ID
    std::string stop_id;  // GTFS stop ID
    int trip_id;  // Augmented trip ID for populating rotations
    bool is_depot;  // True if the stop is a depot
    bool is_charge_station;  // True if the stop is a charging station

    int current_idle_time;  // Total idle time across all rotations. This measures the utilization of the terminal.
    int potential_idle_time; // Total idle time across all rotations if this were to be a charge station
    std::vector<int> passing_rotation;  // indices passing through the terminal in the current rotation
    std::vector<int> passing_rotation_idle_time;  // Idle time of each trip in the current rotation

    // Constructor
    Terminal()
    {
        // Nothing to do here
    };

    Terminal(int stop_id, int trip_id, bool is_depot, bool is_station)
    {
        this->stop_id = stop_id;
        this->trip_id = trip_id;
        this->is_depot = is_depot;
        this->is_charge_station = is_station;
    }

    // Destructor
    ~Terminal()
    {
        // Nothing to do here
    }
};

//Create a trip class which contains relevant GTFS details
class Trip {
public:
    int id;  // Trip ID
    int start_terminal;  // Terminal ID of first stop
    int end_terminal;  // Terminal ID of last stop
    int start_time;  // Minutes since midnight
    int end_time;  // Minutes since midnight
    double distance;  // Distance in km
    std::vector<bool> is_compatible;  // Booleans to check if a trip is compatible with another (includes depot 'trips')
    std::vector<double> deadhead_distance;  // Vector of deadhead distances (includes depot 'trips')
    std::vector<double> idle_time;  // Vector of idle times (includes depot 'trips')

    // Constructor
    Trip()
    {
        // Nothing to do here
    }

    Trip(int id, int start_terminal, int end_terminal, int start_time, int end_time, double distance)
    {
        this->id = id;
        this->start_terminal = start_terminal;
        this->end_terminal = end_terminal;
        this->start_time = start_time;
        this->end_time = end_time;
        this->distance = distance;
    }

    //Destructor
    ~Trip()
    {
        is_compatible.clear();
        deadhead_distance.clear();
        idle_time.clear();
    }
};

//Create a vehicle class which stores bus rotation details
class Vehicle {
public:
    int id;  // Vehicle IDs. Rotations are added and removed and hence these need not be continuous
    std::vector<int> trip_id;  // First and last trips are aliases for depots
    double deadhead_cost;  // Cost of deadheading in the rotation

    // Constructor
    Vehicle()
    {
        // Nothing to do here
    };

    Vehicle(int id)
    {
        this->id = id;
        deadhead_cost = 0.0;
    }

    // Desctructor
    ~Vehicle()
    {
        trip_id.clear();
    }

    // Function to calculate the deadheading costs in the rotation
    void calculate_deadhead_cost(const std::vector<Trip>& trip)
    {
        // Calculate the deadhead cost of the rotation
        deadhead_cost = 0.0;
        int curr_trip_index, next_trip_index;
        for (int i = 0; i<trip_id.size()-1; ++i) {
            curr_trip_index = trip_id[i]-1;
            next_trip_index = trip_id[i+1]-1;
            deadhead_cost += trip[curr_trip_index].deadhead_distance[next_trip_index]*COST_PER_KM;
        }
    }

    // Print members of the class
    void log_member_data(Logger& logger)
    {
        logger.log(LogLevel::Info, "Vehicle ID, No. of Trips, Deadhead cost, Trip IDs: "+std::to_string(id)+" "
                +std::to_string(trip_id.size())+" "+std::to_string(deadhead_cost)+" "+vector_to_string(trip_id));
    }
};

#endif //EBUS_VNS_VEHICLE_H
