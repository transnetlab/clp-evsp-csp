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

    Terminal(std::string stop_id, int trip_id, bool is_depot, bool is_station)
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
    std::vector<int> idle_time;  // Vector of idle times (includes depot 'trips')

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

    // Variables for CSP
    bool is_charging_required = false;  // Updated to true if charging is required in the rotation
    std::vector<int> charge_terminal;  // Vector of charging station terminal IDs at different opportunities (repeats are allowed)
    std::vector<int> start_charge_time;  // Vector of start charge times at different opportunities
    std::vector<int> end_charge_time;  // Vector of end charge times at different opportunities
    std::vector<double> energy_till_charge_terminal;  // Total energy required to reach the charging terminal. This has an extra element to account for reaching the depot.

    // Constructor
    Vehicle()
    {

    };

    Vehicle(int id)
    {
        this->id = id;
        deadhead_cost = 0.0;
    }

    // Destructor
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

    // Clear CSP variables
    void clear_csp_variables()
    {
        is_charging_required = false;
        charge_terminal.clear();
        start_charge_time.clear();  //TODO: Check if this starts at 0, i.e., are these indices?
        end_charge_time.clear();
        energy_till_charge_terminal.clear();
    }

    // Print members of the class
    void log_member_data(Logger& logger) const
    {
        logger.log(LogLevel::Info, "Vehicle ID, No. of Trips, Deadhead cost, Trip IDs: "+std::to_string(id)+" "
                +std::to_string(trip_id.size())+" "+std::to_string(deadhead_cost)+" "+vector_to_string(trip_id));
    }

    // Populate CSP related variables under the charge and go policy
    void populate_csp_variables_cag(const std::vector<Trip>& trip, const std::vector<Terminal>& terminal) {
        // If trip is empty, throw an error and exit
        if (trip_id.empty()) {
            std::cerr << "Error: Trip ID is empty for vehicle while populating CSP data " << id << std::endl;
            exit(1);
        }

        int curr_trip, next_trip;  // Current trip and next trip IDs
        int end_terminal_curr_trip, start_terminal_next_trip; // End terminal of current trip and start terminal of the next trip
        bool is_curr_trip_end_charge_terminal, is_next_trip_start_charge_terminal;
        int charge_time_window;  // Idle time during which charging is allowed

        int curr_trip_end_time, next_trip_start_time;  // Current trip end time and next trip start time
        double cumulative_energy; // Cumulative energy required from the start depot

        // Calculate the energy required for deadheading from the depot and the first trip
        cumulative_energy =
                (trip[trip_id[0]-1].deadhead_distance[trip_id[1]-1]+trip[trip_id[1]-1].distance)*ENERGY_PER_KM;

        // Iterate across trips and find the charging station terminals
        // Depots, start terminal of the first trip and end terminal of last trip are excluded
        for (int i = 1; i<trip_id.size()-2; ++i) {
            curr_trip = trip_id[i];
            next_trip = trip_id[i+1];

            end_terminal_curr_trip = trip[curr_trip-1].end_terminal;  // End terminal id of current trip
            start_terminal_next_trip = trip[next_trip-1].start_terminal;  // Start terminal id of next trip

            is_curr_trip_end_charge_terminal = terminal[end_terminal_curr_trip-1].is_charge_station;
            is_next_trip_start_charge_terminal = terminal[start_terminal_next_trip-1].is_charge_station;

            curr_trip_end_time = trip[curr_trip-1].end_time;
            next_trip_start_time = trip[next_trip-1].start_time;

            charge_time_window = trip[curr_trip-1].idle_time[next_trip-1];

            // n: no charging, e: end terminal, s: start terminal; Charging is preferred at the end terminal
            char scenario = 'n';
            if (is_curr_trip_end_charge_terminal)
                scenario = 'e';
            else if (is_next_trip_start_charge_terminal)
                scenario = 's';

            switch (scenario) {
            case 'e':
                if (charge_time_window > 0) {
                    charge_terminal.push_back(end_terminal_curr_trip);
                    start_charge_time.push_back(curr_trip_end_time);
                    end_charge_time.push_back(curr_trip_end_time+charge_time_window);
                    energy_till_charge_terminal.push_back(cumulative_energy);
                }
                cumulative_energy += trip[curr_trip-1].deadhead_distance[next_trip-1]*ENERGY_PER_KM;
                break;
            case 's':
                cumulative_energy += trip[curr_trip-1].deadhead_distance[next_trip-1]*ENERGY_PER_KM;
                if (charge_time_window > 0) {
                    charge_terminal.push_back(start_terminal_next_trip);
                    start_charge_time.push_back(next_trip_start_time-charge_time_window);
                    end_charge_time.push_back(next_trip_start_time);
                    energy_till_charge_terminal.push_back(cumulative_energy);
                }
                break;
            default:  // No charging location is available at either location
                cumulative_energy += trip[curr_trip-1].deadhead_distance[next_trip-1]*ENERGY_PER_KM;
            }
            cumulative_energy += trip[next_trip-1].distance*ENERGY_PER_KM;
        }

        // Add distance from the last trip to the depot to cumulative_energy
        int penultimate_trip = trip_id[trip_id.size()-2];
        int last_trip = trip_id[trip_id.size()-1];
        cumulative_energy += trip[penultimate_trip-1].deadhead_distance[last_trip-1]*ENERGY_PER_KM;
        energy_till_charge_terminal.push_back(cumulative_energy);

        // If cumulative energy is less than the maximum charge level, then charging is not required
        is_charging_required = (cumulative_energy > MAX_CHARGE_LEVEL);
    }
};

#endif //EBUS_VNS_VEHICLE_H
