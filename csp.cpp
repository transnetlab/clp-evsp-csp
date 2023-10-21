// CPLEX code for charge scheduling problem
#include "csp.h"

void initialization::update_vehicles(std::vector<Trip>& trip, std::vector<Terminal>& terminal,
        std::vector<Vehicle>& vehicle, Logger& logger)
{
    logger.log(LogLevel::Info, "Updating CSP variables for all vehicles");

    // Clear old CSP variables (if any) and update them
    for (auto& curr_vehicle : vehicle) {
        curr_vehicle.clear_csp_variables();
        curr_vehicle.populate_csp_variables_cag(trip, terminal);
    }

    // Log the results from the initialization step
    logger.log(LogLevel::Info, "CSP variables updated for all vehicles");
    logger.log(LogLevel::Info, "Number of vehicles: "+std::to_string(vehicle.size()));
    logger.log(LogLevel::Info,
            "Number of vehicles requiring charging: "+std::to_string(std::count_if(vehicle.begin(), vehicle.end(),
                    [](Vehicle& bus) { return bus.is_charging_required; })));
    for (const auto& curr_vehicle : vehicle) {
        logger.log(LogLevel::Debug, "Vehicle ID: "+std::to_string(curr_vehicle.id));
        logger.log(LogLevel::Debug, "Is charging required: "+std::to_string(curr_vehicle.is_charging_required));
        logger.log(LogLevel::Debug, "Charge opportunity terminals: "+vector_to_string(curr_vehicle.charge_terminal));
        logger.log(LogLevel::Debug, "Charge opportunity start times: "+vector_to_string(curr_vehicle.start_charge_time));
        logger.log(LogLevel::Debug, "Charge opportunity end times: "+vector_to_string(curr_vehicle.end_charge_time));
        logger.log(LogLevel::Debug,
                "Vehicle energy till charge terminals: "+vector_to_string(curr_vehicle.energy_till_charge_terminal));
    }
}

void lp::solve_csp(const std::vector<Trip>& trip, const std::vector<Terminal>& terminal,
        const std::vector<Vehicle>& vehicle, Logger& logger)
{


}