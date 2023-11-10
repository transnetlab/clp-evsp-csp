#ifndef EBUS_VNS_CONSTANTS_H
#define EBUS_VNS_CONSTANTS_H

#include "logger.h"
#include <array>
#include <string>

constexpr double VEHICLE_COST = 350000.0; // Acquisition cost of a bus
constexpr double CHARGE_LOC_COST = 200000.0;  // Cost of opening one charging location
constexpr double COST_PER_KM = 0.44 * 365 * 12;  // Per km travel cost

constexpr double MAX_CHARGE_LEVEL = 300.0;  // Maximum allowed charge level in kWh
constexpr double MIN_CHARGE_LEVEL = 45.0;  // Minimum charge level to be maintained in kWh
constexpr double CHARGE_RATE = 1.67;  // Rate at which charging happens km/min
constexpr double MAX_ENERGY_PER_MIN = 2.505;  // Maximum energy IN kWh that can be charged in a minute
constexpr double ENERGY_PER_KM = 1.5;  // Energy consumed in kWh/km

constexpr int NUM_PRICE_INTERVALS = 7;  // Number of energy price points
constexpr std::array<int, NUM_PRICE_INTERVALS + 1> ENERGY_LEFT_INTERVAL = {0, 540, 840, 960, 1260, 1440, 1980, 2280};
constexpr std::array<double, NUM_PRICE_INTERVALS> ENERGY_PRICE = {400.0, 300.0, 400.0, 1120.0, 400.0, 400.0, 300.0};

constexpr double POWER_CAPACITY_PRICE = 600;  // in Euros/kW

constexpr double INF = 1e12; // Large number to represent infinity
constexpr double EPSILON = 1e-12; // Small number to compare doubles
constexpr double SMALL_EPSILON = 1e-15; // Small number to compare doubles

constexpr double IDLE_TIME_THRESHOLD = 0.0;  // Threshold for idle time in minutes used in opening and closing stations
constexpr bool SWAP_CHARGE_STATIONS = false; // Flag to turn on/off charge station swap operators
constexpr int SHIFT_ALL_TRIPS_THRESHOLD = 6; // Threshold for number of trips in a rotation to perform shift all trips (includes depots)

//constexpr bool SOLVE_CSP_JOINTLY = false;  // Flag to solve the CSP jointly or separately
enum class SolutionType : int {
  Split,
  Uniform
};
constexpr SolutionType CSP_SOLUTION_TYPE = SolutionType::Uniform; // Solution type for the CSP



#endif //EBUS_VNS_CONSTANTS_H
