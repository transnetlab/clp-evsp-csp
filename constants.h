#ifndef EBUS_VNS_CONSTANTS_H
#define EBUS_VNS_CONSTANTS_H

#include <array>

constexpr double VEHICLE_COST = 350000.0; // Acquisition cost of a bus
constexpr double CHARGE_LOC_COST = 200000.0;  // Cost of opening one charging location
constexpr double COST_PER_KM = 0.44 * 365 * 12;  // Per km travel cost

constexpr double MAX_CHARGE_LEVEL = 300.0;  // Maximum allowed charge level in kWh
constexpr double MIN_CHARGE_LEVEL = 45.0;  // Minimum charge level to be maintained in kWh
constexpr double CHARGE_RATE = 1.67;  // Rate at which charging happens km/min
constexpr double MAX_ENERGY_PER_MIN = 2.505;  // Maximum energy IN kWh that can be charged in a minute
constexpr double ENERGY_PER_KM = 1.5;  // Energy consumed in kWh/km

constexpr int NUM_PRICE_INTERVALS = 5;  // Number of energy price points
constexpr std::array<int, NUM_PRICE_INTERVALS + 1> ENERGY_LEFT_INTERVAL = {0, 540, 840, 960, 1260, 1440};
constexpr std::array<double, NUM_PRICE_INTERVALS> ENERGY_PRICE = {400.0, 300.0, 400.0, 1120.0, 400.0};

constexpr double POWER_CAPACITY_PRICE = 600;  // in Euros/kW

constexpr int MAX_ITERATIONS = 1;  // Maximum number of iterations for the VNS algorithm

constexpr double INF = 1e12; // Large number to represent infinity
constexpr double EPSILON = 1e-6; // Small number to compare doubles

constexpr bool PERFORM_DEPOT_EXCHANGES = true;  // Flag to turn on/off depot exchange operators

#endif //EBUS_VNS_CONSTANTS_H
