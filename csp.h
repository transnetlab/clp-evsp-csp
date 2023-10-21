#ifndef EBUS_VNS_CSP_H
#define EBUS_VNS_CSP_H

#include "logger.h"
#include "constants.h"
#include "vehicle.h"

class ChargeInterval {
public:
    std::vector<int> period_index;  // Index of the price period in which the interval lies
    std::vector<int> within_period_duration;  // Duration of the interval. Each window lies exclusively in a single price period
    std::vector<int> start_time;
    std::vector<int> end_time;
};

namespace initialization {
void update_vehicles(std::vector<Trip>&, std::vector<Terminal>&, std::vector<Vehicle>&, Logger&);
}

namespace lp {
void solve_csp(const std::vector<Trip>&, const std::vector<Terminal>&, const std::vector<Vehicle>&, Logger&);
}

#endif  //EBUS_VNS_CSP_H
