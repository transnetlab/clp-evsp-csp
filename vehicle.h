#ifndef EBUS_VNS_VEHICLE_H
#define EBUS_VNS_VEHICLE_H

#include <string>
#include <vector>

//Create a trip class which contains relevant GTFS details
class Trip {
public:
    int id;
    int start_stop;
    int end_stop;
    int start_time;
    int end_time;
    double distance;
    std::vector<bool> is_compatible;
    std::vector<double> deadhead_distance;
    std::vector<double> idle_time;

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
    int stop_id;  // GTFS stop ID
    int trip_id;  // Augmented trip ID for populating rotations
    bool is_depot;
    bool is_station;

    // Constructor
    Terminal() {
        // Nothing to do here
    };

    Terminal(int stop_id, int trip_id, bool is_depot, bool is_station) {
        this->stop_id = stop_id;
        this->trip_id = trip_id;
        this->is_depot = is_depot;
        this->is_station = is_station;
    }

    // Destructor
    ~Terminal() {
        // Nothing to do here
    }
};

//Create a vehicle class which stores bus rotation details
class Vehicle {
public:
    int id;
    int num_trips;
    std::vector<int> trip_id;
    double deadhead_cost;

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

private:
    // Private function to update num_trips
    void update_num_trips() {
        num_trips = static_cast<int>(trip_id.size());
    }
};

#endif //EBUS_VNS_VEHICLE_H
