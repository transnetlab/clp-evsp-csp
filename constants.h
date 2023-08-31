#ifndef EBUS_VNS_CONSTANTS_H
#define EBUS_VNS_CONSTANTS_H

constexpr double BUS_COST = 350000.0; // Acquisition cost of a bus
constexpr double CHARGE_LOC_COST = 200000.0;  // Cost of opening one charging location
constexpr double DEADHEAD_COST = 0.44 * 365 * 12;  // Per km travel cost

constexpr double MAX_CHARGE_LEVEL = 100.0;  // Maximum allowed charge level
constexpr double MIN_CHARGE_LEVEL = 20.0;  // Minimum charge level to be maintained
constexpr double CHARGE_RATE = 1.67;  // Rate at which charging happens

#endif //EBUS_VNS_CONSTANTS_H
