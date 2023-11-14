// CPLEX code for charge scheduling problem
#include "csp.h"

// This function updates the CSP members of the vehicle class for all vehicles
void initialization::update_vehicles(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal)
{
    //logger.log(LogLevel::Debug, "Updating CSP variables for all vehicles");

    // Clear old CSP variables (if any) and update them
    for (int v = 0; v<vehicle.size(); ++v) {
        vehicle[v].clear_csp_parameters();
        vehicle[v].populate_csp_parameters(trip, terminal);
    }

    // Log the results from the initialization step
    /*logger.log(LogLevel::Info, "CSP variables updated for all vehicles");
    logger.log(LogLevel::Info, "Number of vehicles: "+std::to_string(vehicle.size()));
    logger.log(LogLevel::Info,
            "Number of vehicles requiring charging: "+std::to_string(std::count_if(vehicle.begin(), vehicle.end(),
                    [](Vehicle& bus) { return bus.is_charging_required; })));
    for (const auto& curr_vehicle : vehicle) {
        logger.log(LogLevel::Info, "Vehicle ID: "+std::to_string(curr_vehicle.id));
        logger.log(LogLevel::Info, "Is charging required: "+std::to_string(curr_vehicle.is_charging_required));
        logger.log(LogLevel::Info, "Charge opportunity terminals: "+vector_to_string(curr_vehicle.charge_terminal));
        logger.log(LogLevel::Info,
                "Charge opportunity start times: "+vector_to_string(curr_vehicle.start_charge_time));
        logger.log(LogLevel::Info, "Charge opportunity end times: "+vector_to_string(curr_vehicle.end_charge_time));
        logger.log(LogLevel::Info,
                "Vehicle energy till charge terminals: "+vector_to_string(curr_vehicle.energy_till_charge_terminal));
    }*/
}

// Function to update the CSP members of the vehicle class for a subset of vehicles
void initialization::update_vehicles(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Data& data, std::vector<int>& update_vehicle_indices)
{
    //logger.log(LogLevel::Info, "Updating CSP variables for all vehicles");

    // Clear old CSP variables (if any) and update them
    for (int index = 0; index<update_vehicle_indices.size(); ++index) {
        vehicle[update_vehicle_indices[index]].clear_csp_parameters();
        vehicle[update_vehicle_indices[index]].populate_csp_parameters(trip, terminal);
    }

    // Log the results from the initialization step
    /*logger.log(LogLevel::Info, "CSP variables updated for all vehicles");
    logger.log(LogLevel::Info, "Number of vehicles: "+std::to_string(vehicle.size()));
    logger.log(LogLevel::Info,
            "Number of vehicles requiring charging: "+std::to_string(std::count_if(vehicle.begin(), vehicle.end(),
                    [](Vehicle& bus) { return bus.is_charging_required; })));
    for (const auto& curr_vehicle : vehicle) {
        logger.log(LogLevel::Info, "Vehicle ID: "+std::to_string(curr_vehicle.id));
        logger.log(LogLevel::Info, "Is charging required: "+std::to_string(curr_vehicle.is_charging_required));
        logger.log(LogLevel::Info, "Charge opportunity terminals: "+vector_to_string(curr_vehicle.charge_terminal));
        logger.log(LogLevel::Info,
                "Charge opportunity start times: "+vector_to_string(curr_vehicle.start_charge_time));
        logger.log(LogLevel::Info, "Charge opportunity end times: "+vector_to_string(curr_vehicle.end_charge_time));
        logger.log(LogLevel::Info,
                "Vehicle energy till charge terminals: "+vector_to_string(curr_vehicle.energy_till_charge_terminal));
    }*/
}

void initialization::create_sets(std::vector<Vehicle>& vehicle, std::vector<Terminal>& terminal,
        std::vector<int>& vehicles_requiring_charging, std::vector<int>& terminals_with_charging_station,
        std::vector<int>& terminal_to_index)
{
    logger.log(LogLevel::Debug, "Preprocessing the data for the CSP problem");
    // Determine the indices of vehicles which require charging
    std::vector<int> vehicle_to_index(vehicle.size(), -1);
    for (int v = 0; v<vehicle.size(); ++v) {
        if (vehicle[v].is_charging_required) {
            vehicles_requiring_charging.push_back(v);
            vehicle_to_index[v] = vehicles_requiring_charging.size()-1;
        }  // TODO: Is the vehicle to index required?
    }

    // Determine the terminal indices with charging stations
    // TODO: This can go outside the loop in exchanges and shifts since it does not change
    // Initialize a terminal to index vector with -1s and length equal to the number of terminals
    terminal_to_index.resize(terminal.size(), -1);
    int index = 0;
    for (const auto& curr_terminal : terminal) {
        if (curr_terminal.is_charge_station) {
            terminals_with_charging_station.push_back(curr_terminal.id-1);
            terminal_to_index[curr_terminal.id-1] = index;
            ++index;
        }
    }

    // Log the indices of the vehicles that require charging and the terminals that have charging stations
    /*logger.log(LogLevel::Debug, "Vehicles requiring charging: "+vector_to_string(vehicles_requiring_charging));
    logger.log(LogLevel::Debug, "Terminals with charging: "+vector_to_string(terminals_with_charging_station));*/
}

void initialization::create_sets(std::vector<Vehicle>& vehicle, std::vector<Terminal>& terminal,
        std::vector<int>& vehicles_requiring_charging, std::vector<int>& terminals_with_charging_station)
{
    logger.log(LogLevel::Debug, "Preprocessing the data for the CSP problem");
    // Determine the indices of vehicles which require charging
    std::vector<int> vehicle_to_index(vehicle.size(), -1);
    for (int v = 0; v<vehicle.size(); ++v) {
        if (vehicle[v].is_charging_required) {
            vehicles_requiring_charging.push_back(v);
            vehicle_to_index[v] = vehicles_requiring_charging.size()-1;
        }  // TODO: Is the vehicle to index required?
    }

    // Determine the terminal indices with charging stations
    // TODO: This can go outside the loop in exchanges and shifts since it does not change
    // Initialize a terminal to index vector with -1s and length equal to the number of terminals
    int index = 0;
    for (const auto& curr_terminal : terminal) {
        if (curr_terminal.is_charge_station) {
            terminals_with_charging_station.push_back(curr_terminal.id-1);
            ++index;
        }
    }

    // Log the indices of the vehicles that require charging and the terminals that have charging stations
    /*logger.log(LogLevel::Debug, "Vehicles requiring charging: "+vector_to_string(vehicles_requiring_charging));
    logger.log(LogLevel::Debug, "Terminals with charging: "+vector_to_string(terminals_with_charging_station));*/
}

// This function is used to create three types of variables -- charge levels after different opportunities, energy
// input during different opportunities, and power capacity at charging stations.
void csp::create_variables_uniform_model(IloEnv& env, const std::vector<Vehicle>& vehicle,
        UniformModelVariable& variable, const std::vector<int>& vehicles_requiring_charging,
        const std::vector<int>& terminals_with_charging_station)
{
    // Charging level of vehicle with index v and after charging at opportunity k
    // CPLEX index is b since some of the vehicles do not require charging
    int v;
    std::string var_name;
    for (int b = 0; b<vehicles_requiring_charging.size(); ++b) {
        v = vehicles_requiring_charging[b];  // Vehicle index
        variable.charge_level[b].resize(
                vehicle[v].charge_terminal.size());  // No. of terminal IDs at charge opportunities
        for (int k = 0; k<vehicle[v].charge_terminal.size(); ++k) {  // Iterate over charging opportunities
            variable.charge_level[b][k] = IloNumVar(env, MIN_CHARGE_LEVEL, MAX_CHARGE_LEVEL, IloNumVar::Float);
            //var_name = "l("+std::to_string(v)+","+std::to_string(k)+")";  // Names are wrt original indices
            //variable.charge_level[b][k].setName(var_name.c_str());
        }
    }

    // Energy input variables for vehicle v during opportunity k
    double charge_time_window;
    for (int b = 0; b<vehicles_requiring_charging.size(); ++b) {
        v = vehicles_requiring_charging[b];
        variable.energy_input[b].resize(vehicle[v].charge_terminal.size());
        for (int k = 0; k<vehicle[v].charge_terminal.size(); ++k) {
            charge_time_window = vehicle[v].end_charge_time[k]-vehicle[v].start_charge_time[k];
            variable.energy_input[b][k] = IloNumVar(env, 0.00, MAX_ENERGY_PER_MIN*charge_time_window, IloNumVar::Float);
            //var_name = "w("+std::to_string(v)+","+std::to_string(k)+")";  // Names are wrt original indices
            //variable.energy_input[b][k].setName(var_name.c_str());
        }
    }

    // Power input variables for each charging terminal. Names are wrt original indices
    for (int s = 0; s<terminals_with_charging_station.size(); ++s) {
        variable.terminal_charge_capacity[s] = IloNumVar(env, 0.00, IloInfinity, IloNumVar::Float);
        //var_name = "z("+std::to_string(terminals_with_charging_station[s])+")";
        //variable.terminal_charge_capacity[s].setName(var_name.c_str());
    }
}

void csp::create_constraints_uniform_model(IloEnv& env, IloModel& model, const std::vector<Vehicle>& vehicle,
        const std::vector<Terminal> terminal, UniformModelVariable& variable,
        const std::vector<int>& vehicles_requiring_charging, const std::vector<int>& terminals_with_charging_station,
        const std::vector<int>& terminal_to_index)
{
    int v;
    std::string constraint_name;

    // Constraint 1: Energy balance constraint
    for (int b = 0; b<vehicles_requiring_charging.size(); ++b) {
        v = vehicles_requiring_charging[b];

        // First opportunity
        // constraint_name = "energy_balance("+std::to_string(v)+","+std::to_string(0)+")";
        //model.add(variable.charge_level[b][0]-variable.energy_input[b][0]==
        //        MAX_CHARGE_LEVEL-vehicle[v].energy_till_charge_terminal[0]).setName(constraint_name.c_str());

        model.add(variable.charge_level[b][0]-variable.energy_input[b][0]
                ==MAX_CHARGE_LEVEL-vehicle[v].energy_till_charge_terminal[0]);

        // Not the first opportunity
        for (int k = 0; k<vehicle[v].charge_terminal.size()-1; ++k) {
            // constraint_name = "energy_balance("+std::to_string(v)+","+std::to_string(k+1)+")";
            //model.add(variable.charge_level[b][k+1]-variable.charge_level[b][k]-variable.energy_input[b][k+1]==
            //vehicle[v].energy_till_charge_terminal[k]-vehicle[v].energy_till_charge_terminal[k+1]).setName(
            //        constraint_name.c_str());
            model.add(variable.charge_level[b][k+1]-variable.charge_level[b][k]-variable.energy_input[b][k+1]==
                    vehicle[v].energy_till_charge_terminal[k]-vehicle[v].energy_till_charge_terminal[k+1]);
        }
    }

    // Constraint 2: Minimum charge level constraint
    for (int b = 0; b<vehicles_requiring_charging.size(); ++b) {
        v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].charge_terminal.size(); ++k) { // Includes trip to the depot, hence +1
            //constraint_name = "min_charge_level("+std::to_string(v)+","+std::to_string(k)+")";
            //model.add(variable.charge_level[b][k]>=MIN_CHARGE_LEVEL+vehicle[v].energy_till_charge_terminal[k+1]
            //        -vehicle[v].energy_till_charge_terminal[k]).setName(constraint_name.c_str());
            model.add(variable.charge_level[b][k]>=MIN_CHARGE_LEVEL+vehicle[v].energy_till_charge_terminal[k+1]
                    -vehicle[v].energy_till_charge_terminal[k]);
        }
    }

    // Constraint 3: Determine the max power required at each charging location
    // Save information on the rotation opportunity pairs for different terminals that have charging stations
    std::vector<std::vector<std::pair<int, int>>> terminal_rotation_opportunity(
            terminals_with_charging_station.size());

    for (int b = 0; b < vehicles_requiring_charging.size(); ++b) {
        int v = vehicles_requiring_charging[b];
        for (int k = 0; k < vehicle[v].charge_terminal.size(); ++k) {
            int s = terminal_to_index[vehicle[v].charge_terminal[k] - 1];  // Terminal index
            terminal_rotation_opportunity[s].push_back(std::make_pair(b, k));
        }
    }

    // Loop through all terminals with charging stations
    for (int s = 0; s<terminals_with_charging_station.size(); ++s) {
        // Create a graph where nodes are rotation-opportunity pairs and edges connect them if they overlap
        typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;
        Graph g;

        // Loop through all trip rotations and charge opportunities for two nested loops and check if they overlap
        for (int i = 0; i<terminal_rotation_opportunity[s].size(); ++i) {
            int u = vehicles_requiring_charging[terminal_rotation_opportunity[s][i].first];  // This is original vehicle index
            int k = terminal_rotation_opportunity[s][i].second;
            boost::add_edge(i, i, g);
            for (int j = i+1; j<terminal_rotation_opportunity[s].size(); ++j) {
                v = vehicles_requiring_charging[terminal_rotation_opportunity[s][j].first];
                int l = terminal_rotation_opportunity[s][j].second;
                if (u==v) // Check if v is different from u. If yes, check if the charge opportunities overlap
                    continue;

                // If they overlap add an edge between the (rotation, charge opportunity) pair in graph g
                if (vehicle[u].start_charge_time[k]<vehicle[v].end_charge_time[l]
                        && vehicle[v].start_charge_time[l]<vehicle[u].end_charge_time[k])
                    boost::add_edge(i, j, g);
            }
        }

        std::vector<Clique> cliques;
        struct {
          Cliques& target;
          void clique(std::deque<V> clique, Graph const&) const { target.push_back(std::move(clique)); }
        } collect{cliques};
        bron_kerbosch_all_cliques(g, collect, 1);

        /*std::vector<Clique> cliques;
        bron_kerbosch_all_cliques(g, Collector{cliques}, 1);*/

        // Iterate across the cliques and populate max power constraints
        for (int c = 0; c<cliques.size(); ++c) {
            IloExpr max_power(env);
            // Select the rotation and charge opportunity pair from the clique elements
            for (int pair = 0; pair<cliques[c].size(); ++pair) {
                int b = terminal_rotation_opportunity[s][cliques[c][pair]].first;
                int k = terminal_rotation_opportunity[s][cliques[c][pair]].second;
                v = vehicles_requiring_charging[b];

                // Populate the max power constraint. Check if denominator is zero, if so throw an error and exit
                double denominator = double(vehicle[v].end_charge_time[k]-vehicle[v].start_charge_time[k])/60.0;
                if (denominator<0.0) {
                    std::cerr << "Error: Check charge time windows of vehicle index " << v << std::endl;
                    exit(1);
                }
                max_power += variable.energy_input[b][k]/denominator;
            }

            // Add the max power constraint to the model
            //constraint_name = "max_power("+std::to_string(terminals_with_charging_station[s])+","+std::to_string(c)+")";
            //model.add(max_power<=variable.terminal_charge_capacity[s]).setName(constraint_name.c_str());
            model.add(max_power<=variable.terminal_charge_capacity[s]);
            max_power.end();
        }
    }
}

void csp::create_objective_uniform_model(IloExpr& objective, const std::vector<Vehicle>& vehicle,
        UniformModelVariable& variable, const std::vector<int>& vehicles_requiring_charging,
        const std::vector<int>& terminals_with_charging_station)
{
    int v;
    for (int b = 0; b<vehicles_requiring_charging.size(); ++b) {
        v = vehicles_requiring_charging[b];

        // Find the last end charging time across all vehicles and all opportunities
        //int last_end_charge_time = vehicle[v].end_charge_time[vehicle[v].charge_terminal.size()-1];

        // If last charge time is greater than 1440, throw an error and exit
        /*if (last_end_charge_time>1440) {
            std::cerr << "Error: Operations extend beyond a day to " << last_end_charge_time;
            std::cerr << "Adjust pricing in the constants file..." << std::endl;
            exit(1);
        }*/

        /*int k = 0;
        int left_marker = vehicle[v].start_charge_time[k];
        std::vector<ChargeInterval> charge_interval(vehicle[v].charge_terminal.size());
        // Code to populate sub-intervals of the charging opportunities where prices are the same
        for (int p = 0; p<NUM_PRICE_INTERVALS-1; ++p) {
            if (left_marker<ENERGY_LEFT_INTERVAL[p+1]) {  // charging opportunity k starts in [p, p+1]
                charge_interval[k].period_index.push_back(p);
                charge_interval[k].start_time.push_back(left_marker);
                // Check if charging finishes in this time period or continues in the next one.
                // Charging opportunity k ends in [p, p+1]
                if (vehicle[v].end_charge_time[k]<=ENERGY_LEFT_INTERVAL[p+1]) {
                    charge_interval[k].end_time.push_back(vehicle[v].end_charge_time[k]);
                    charge_interval[k].within_period_duration.push_back(
                            vehicle[v].end_charge_time[k]-left_marker);

                    ++k; // Proceed to the next charging opportunity
                    if (k>vehicle[v].charge_terminal.size()-1) // If k is the last charging opportunity
                        break;

                    left_marker = vehicle[v].start_charge_time[k]; // Update left end point
                    --p; // Checks if the next charging opportunity starts in the same time window
                } // Check till the last possible k. Charging opportunity k continues.
                else {
                    charge_interval[k].end_time.push_back(ENERGY_LEFT_INTERVAL[p+1]);
                    charge_interval[k].within_period_duration.push_back(ENERGY_LEFT_INTERVAL[p+1]-left_marker);

                    left_marker = ENERGY_LEFT_INTERVAL[p+1];
                }
            }
        }*/

        std::vector<ChargeInterval> charge_interval(vehicle[v].charge_terminal.size());
        // Code to populate sub-intervals of the charging opportunities where prices are the same
        for (int k = 0; k<vehicle[v].charge_terminal.size(); ++k) {
            int left_marker = vehicle[v].start_charge_time[k];
            for (int p = 0; p<NUM_PRICE_INTERVALS-1; ++p) {
                if (left_marker<ENERGY_LEFT_INTERVAL[p+1]) {  // charging opportunity k starts in [p, p+1]
                    charge_interval[k].period_index.push_back(p);
                    charge_interval[k].start_time.push_back(left_marker);

                    // Check if charging finishes in this time period or continues in the next one.
                    // Charging opportunity k ends in [p, p+1]
                    if (vehicle[v].end_charge_time[k]<=ENERGY_LEFT_INTERVAL[p+1]) {
                        charge_interval[k].end_time.push_back(vehicle[v].end_charge_time[k]);
                        charge_interval[k].within_period_duration.push_back(
                                vehicle[v].end_charge_time[k]-left_marker);
                        break;
                    } // Charging opportunity k continues in another price period
                    else {
                        charge_interval[k].end_time.push_back(ENERGY_LEFT_INTERVAL[p+1]);
                        charge_interval[k].within_period_duration.push_back(ENERGY_LEFT_INTERVAL[p+1]-left_marker);

                        left_marker = ENERGY_LEFT_INTERVAL[p+1];
                    }
                }
            }
        }

        // Log vector members of charge opportunities
        logger.log(LogLevel::Verbose, "Charging opportunities for vehicle index "+std::to_string(v));
        for (int k = 0; k<charge_interval.size(); ++k) {
            logger.log(LogLevel::Verbose, "Charging opportunity "+std::to_string(k));
            charge_interval[k].log_member_data(logger);
        }

        // Add objective expressions for dynamic energy prices
        for (int k = 0; k<charge_interval.size(); ++k) {
            // Charging opportunity k lies in a single price period
            if (charge_interval[k].period_index.size()==1)
                objective += variable.energy_input[b][k]*ENERGY_PRICE[charge_interval[k].period_index[0]];
            else {  // Charging opportunity k lies in multiple price periods
                // Find the fraction of time the vehicle is charging in each price period
                double denominator_inverse =
                        1.0/double(vehicle[v].end_charge_time[k]-vehicle[v].start_charge_time[k]);
                for (int p = 0; p<charge_interval[k].period_index.size(); ++p) {
                    double time_fraction = double(charge_interval[k].within_period_duration[p])*denominator_inverse;
                    objective += variable.energy_input[b][k]*(time_fraction)
                            *(ENERGY_PRICE[charge_interval[k].period_index[p]]);
                }
            }
        }
        charge_interval.clear();  // Clear charge_interval variable
    }

    // Add objective expressions for static capacity costs
    for (int s = 0; s<terminals_with_charging_station.size(); ++s)
        objective += variable.terminal_charge_capacity[s]*POWER_CAPACITY_PRICE;
}

void csp::log_solution_uniform_model(IloCplex& cplex, const std::vector<Vehicle>& vehicle,
        const UniformModelVariable& variable, const std::vector<int>& vehicles_requiring_charging,
        const std::vector<int>& terminals_with_charging_station)
{
    int v;
    // Log the charging level solution
    logger.log(LogLevel::Debug, "Charging level solution");
    for (int b = 0; b<vehicles_requiring_charging.size(); ++b) {
        v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].charge_terminal.size(); ++k) {
            logger.log(LogLevel::Debug, "Charge level for vehicle "+std::to_string(v+1)+" at opportunity "
                    +std::to_string(k)+" = "+std::to_string(cplex.getValue(variable.charge_level[b][k])));
        }
    }

    // Log the energy input solution
    logger.log(LogLevel::Debug, "Energy level solution");
    for (int b = 0; b<vehicles_requiring_charging.size(); ++b) {
        v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].charge_terminal.size(); ++k) {
            logger.log(LogLevel::Debug, "Energy input for vehicle "+std::to_string(v+1)+" at opportunity "
                    +std::to_string(k)+" = "+std::to_string(cplex.getValue(variable.energy_input[b][k])));
        }
    }

    // Log the power limit solutions
    logger.log(LogLevel::Debug, "Power limit solution");
    for (int s = 0; s<terminals_with_charging_station.size(); ++s) {
        logger.log(LogLevel::Debug, "Power limit for terminal "+std::to_string(terminals_with_charging_station[s])
                +" = "+std::to_string(cplex.getValue(variable.terminal_charge_capacity[s])));
    }
}

double csp::solve_uniform_model(std::vector<Vehicle>& vehicle, std::vector<Terminal>& terminal, Data& data)
{
    logger.log(LogLevel::Debug, "Solving the uniform charging CSP problem");
    IloEnv env; // Create the CPLEX environment
    double optimal_objective;

    try {
        // Create a CPLEX model
        IloModel model(env);
        IloCplex cplex(model);

        // Turn off CPLEX output
        cplex.setOut(env.getNullStream());
        cplex.setWarning(env.getNullStream());

        std::vector<int> vehicles_requiring_charging;
        std::vector<int> terminals_with_charging_station;
        std::vector<int> terminal_to_index;
        initialization::create_sets(vehicle, terminal, vehicles_requiring_charging, terminals_with_charging_station, terminal_to_index);

        // Create decision variables
        logger.log(LogLevel::Debug, "Creating decision variables");
        UniformModelVariable variable(vehicles_requiring_charging.size(), terminals_with_charging_station.size());
        csp::create_variables_uniform_model(env, vehicle, variable, vehicles_requiring_charging,
                terminals_with_charging_station);

        // Add constraints
        logger.log(LogLevel::Debug, "Adding constraints");
        csp::create_constraints_uniform_model(env, model, vehicle, terminal, variable, vehicles_requiring_charging,
                terminals_with_charging_station, terminal_to_index);

        // Add objective
        logger.log(LogLevel::Debug, "Adding objective function");
        IloExpr objective(env);
        csp::create_objective_uniform_model(objective, vehicle, variable, vehicles_requiring_charging,
                terminals_with_charging_station);

        // Solve the linear program
        IloObjective obj = IloMinimize(env, objective);
        model.add(obj);
        objective.end();

        logger.log(LogLevel::Debug, "Solving the LP");
        if (!cplex.solve()) {
            cplex.exportModel("../output/csp_uniform.lp");  // Write the LP to a .lp file
            logger.log(LogLevel::Error, "Failed to optimize LP");

            // Log terminal data
            logger.log(LogLevel::Info, "Terminal data and utilization");
            for (const auto& curr_terminal : terminal)
                curr_terminal.log_member_data();

            // Log the vehicle rotations
            logger.log(LogLevel::Info, "Vehicle rotations");
            for (const auto& curr_vehicle : vehicle)
                curr_vehicle.log_member_data();
            logger.log(LogLevel::Info, "Vehicle CSP member data");
            for (const auto& curr_vehicle : vehicle)
                curr_vehicle.log_csp_member_data();
            throw (-1);
        }

        if (data.log_csp_solution) {  // Write the LP to a .lp file and save the results in a .sol file
            cplex.exportModel(("../output/"+data.instance+"_csp_uniform.lp").c_str());
            cplex.writeSolution(("../output/"+data.instance+"_csp_uniform.sol").c_str());
        }

        //Display results and log the solution
        optimal_objective = cplex.getObjValue();
        //logger.log(LogLevel::Debug, "Solution status = "+std::to_string(cplex.getStatus()));
        //logger.log(LogLevel::Debug, "Solution value = "+std::to_string(cplex.getObjValue()));
        //csp::log_solution_uniform_model(cplex, vehicle, variable, vehicles_requiring_charging, terminals_with_charging_station);
    }
        // Catch exceptions thrown by CPLEX
    catch (IloException& exception) {
        std::cerr << "Error: " << exception << std::endl;
    }
    env.end();  // Clean up
    return optimal_objective;
}

/* SPLIT MODEL
 * In this model, charging decisions can be made at one-minute intervals.
 * The charging levels are still tracked at the end of each charging opportunity.
 * Terminal charge capacity decisions are evaluated at each time step. */

// Function that creates the variables required for the split model. They include charge levels at the end of each
// opportunity, energy input at each time step, and terminal charge capacity
void csp::create_variables_split_model(IloEnv& env, const std::vector<Vehicle>& vehicle, SplitModelVariable& variable,
        const std::vector<int>& vehicles_requiring_charging, const std::vector<int>& terminals_with_charging_station)
{
    // Charging level of vehicle with index v and after charging at opportunity k
    // CPLEX index is b since some of the vehicles do not require charging
    int v;
    //std::string var_name;
    for (int b = 0; b<vehicles_requiring_charging.size(); ++b) {
        v = vehicles_requiring_charging[b];  // Vehicle index
        variable.charge_level[b].resize(
                vehicle[v].charge_terminal.size());  // No. of terminal IDs at charge opportunities
        for (int k = 0; k<vehicle[v].charge_terminal.size(); ++k) {  // Iterate over charging opportunities
            variable.charge_level[b][k] = IloNumVar(env, MIN_CHARGE_LEVEL, MAX_CHARGE_LEVEL, IloNumVar::Float);
            //var_name = "l("+std::to_string(v)+","+std::to_string(k)+")";  // Names are wrt original indices
            //variable.charge_level[b][k].setName(var_name.c_str());
        }
    }

    // Energy input variables for vehicle v during opportunity k
    int charge_time_window;
    for (int b = 0; b<vehicles_requiring_charging.size(); ++b) {
        v = vehicles_requiring_charging[b];
        variable.energy_input[b].resize(
                vehicle[v].charge_terminal.size());  // No. of terminal IDs at charge opportunities
        for (int k = 0; k<vehicle[v].charge_terminal.size(); ++k) {  // Iterate over charging opportunities
            charge_time_window = vehicle[v].end_charge_time[k]-vehicle[v].start_charge_time[k];
            variable.energy_input[b][k].resize(charge_time_window);
            for (int t = 0; t<charge_time_window; ++t) {
                variable.energy_input[b][k][t] = IloNumVar(env, 0.00, MAX_ENERGY_PER_MIN, IloNumVar::Float);
                //var_name = "w("+std::to_string(v)+","+std::to_string(vehicle[v].charge_terminal[k]-1)+","
                //        +std::to_string(vehicle[v].start_charge_time[k]+t)+")";
                //variable.energy_input[b][k][t].setName(var_name.c_str());
            }
        }
    }

    // Power input variables for each charging terminal. Names are wrt original indices
    for (int s = 0; s<terminals_with_charging_station.size(); ++s) {
        variable.terminal_charge_capacity[s] = IloNumVar(env, 0.00, IloInfinity, IloNumVar::Float);
        //var_name = "z("+std::to_string(terminals_with_charging_station[s])+")";
        //variable.terminal_charge_capacity[s].setName(var_name.c_str());
    }
}

// Function that creates the constraints required for the split model. They include energy balance, minimum charge level,
// and max power constraints
void csp::create_constraints_split_model(IloEnv& env, IloModel& model, const std::vector<Vehicle>& vehicle,
        const Data& data,
        SplitModelVariable& variable, const std::vector<int>& vehicles_requiring_charging,
        const std::vector<int>& terminals_with_charging_station, const std::vector<int>& terminal_to_index)
{
    int v, s;
    //std::string constraint_name;

    // Constraint 1: Energy balance constraint
    int charge_time_window;
    for (int b = 0; b<vehicles_requiring_charging.size(); ++b) {
        v = vehicles_requiring_charging[b];

        // First opportunity -- run a loop from start charge time to end charge time at this opportunity
        IloExpr lhs(env);
        lhs += variable.charge_level[b][0];
        charge_time_window = vehicle[v].end_charge_time[0]-vehicle[v].start_charge_time[0];
        for (int t = 0; t<charge_time_window; ++t)
            lhs -= variable.energy_input[b][0][t];

        //constraint_name = "energy_balance("+std::to_string(v)+","+std::to_string(0)+")";
        //model.add(lhs==MAX_CHARGE_LEVEL-vehicle[v].energy_till_charge_terminal[0]).setName(
        //        constraint_name.c_str());
        model.add(lhs==MAX_CHARGE_LEVEL-vehicle[v].energy_till_charge_terminal[0]);
        lhs.end();

        // Not the first opportunity -- run a loop from start charge time to end charge time at this opportunity
        for (int k = 0; k<vehicle[v].charge_terminal.size()-1; ++k) {
            s = terminal_to_index[vehicle[v].charge_terminal[k]-1];  // Terminal index in the subset of indices
            IloExpr lhs(env);
            lhs += variable.charge_level[b][k+1];
            lhs -= variable.charge_level[b][k];
            charge_time_window = vehicle[v].end_charge_time[k+1]-vehicle[v].start_charge_time[k+1];
            for (int t = 0; t<charge_time_window; ++t)
                lhs -= variable.energy_input[b][k+1][t];

            //constraint_name = "energy_balance("+std::to_string(v)+","+std::to_string(k+1)+")";
            //model.add(lhs==vehicle[v].energy_till_charge_terminal[k]-
            //        vehicle[v].energy_till_charge_terminal[k+1]).setName(constraint_name.c_str());
            model.add(lhs==vehicle[v].energy_till_charge_terminal[k]-vehicle[v].energy_till_charge_terminal[k+1]);
            lhs.end();
        }
    }

    // Constraint 2: Minimum charge level constraint
    for (int b = 0; b<vehicles_requiring_charging.size(); ++b) {
        v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].charge_terminal.size(); ++k) { // Includes trip to the depot, hence +1
            //constraint_name = "min_charge_level("+std::to_string(v)+","+std::to_string(k)+")";
            //model.add(variable.charge_level[b][k]>=MIN_CHARGE_LEVEL+vehicle[v].energy_till_charge_terminal[k+1]
            //        -vehicle[v].energy_till_charge_terminal[k]).setName(constraint_name.c_str());
            model.add(variable.charge_level[b][k]>=MIN_CHARGE_LEVEL+vehicle[v].energy_till_charge_terminal[k+1]
                    -vehicle[v].energy_till_charge_terminal[k]);
        }
    }

    // Constraint 3: Determine the max power required at each charging location
    IloArray<IloExprArray> total_energy_input(env, terminals_with_charging_station.size());
    for (s = 0; s<terminals_with_charging_station.size(); ++s) {
        total_energy_input[s] = IloExprArray(env, data.time_steps_length);
        for (int t = 0; t<data.time_steps_length; ++t)
            total_energy_input[s][t] = IloExpr(env);
    }

    std::vector<std::vector<bool>> include_lhs(vehicles_requiring_charging.size(),
            std::vector<bool>(data.time_steps_length, false));

    for (int b = 0; b<vehicles_requiring_charging.size(); ++b) {
        v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].charge_terminal.size(); ++k) {
            s = terminal_to_index[vehicle[v].charge_terminal[k]-1];  // Terminal index in the subset of indices
            charge_time_window = vehicle[v].end_charge_time[k]-vehicle[v].start_charge_time[k];
            for (int t = 0; t<charge_time_window; ++t) {
                total_energy_input[s][vehicle[v].start_charge_time[k]+t
                        -data.first_trip_start_time] += variable.energy_input[b][k][t];
                include_lhs[s][vehicle[v].start_charge_time[k]+t-data.first_trip_start_time] = true;
            }
        }
    }

    for (s = 0; s<terminals_with_charging_station.size(); ++s) {
        for (int t = 0; t<data.time_steps_length; ++t) {
            if (include_lhs[s][t]) {
                //constraint_name =
                //        "max_power("+std::to_string(terminals_with_charging_station[s])+","
                //                +std::to_string(t+data.first_trip_start_time)+")";
                //model.add(total_energy_input[s][t]<=variable.terminal_charge_capacity[s]/60.0).setName(
                //        constraint_name.c_str());
                model.add(total_energy_input[s][t]<=variable.terminal_charge_capacity[s]/60.0);
            }
        }
    }

    // Clear the expression variables
    for (s = 0; s<terminals_with_charging_station.size(); ++s) {
        total_energy_input[s].endElements();  // End individual expressions in each row
        total_energy_input[s].end();  // End the row
    }
}

// Function that creates the objective function for the split model. It includes energy input for each rotation and
// fixed costs due to terminal charge capacity
void csp::create_objective_split_model(IloExpr& objective, const std::vector<Vehicle>& vehicle, Data& data,
        SplitModelVariable& variable, const std::vector<int>& vehicles_requiring_charging,
        const std::vector<int>& terminals_with_charging_station)
{
    int v;
    for (int b = 0; b<vehicles_requiring_charging.size(); ++b) {
        v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].charge_terminal.size(); ++k) {
            int charge_time_window = vehicle[v].end_charge_time[k]-vehicle[v].start_charge_time[k];
            for (int t = 0; t<charge_time_window; ++t)
                objective +=
                        variable.energy_input[b][k][t]*data.energy_price_per_min[vehicle[v].start_charge_time[k]+t];
        }
    }

    // Add objective expressions for static capacity costs
    for (int s = 0; s<terminals_with_charging_station.size(); ++s)
        objective += variable.terminal_charge_capacity[s]*POWER_CAPACITY_PRICE;
}

// Function that logs the solution of the split model for debugging
void csp::log_solution_split_model(IloCplex& cplex, const std::vector<Vehicle>& vehicle,
        SplitModelVariable& variable, const std::vector<int>& vehicles_requiring_charging,
        const std::vector<int>& terminals_with_charging_station)
{
    int v;
    // Log the charging level solution
    logger.log(LogLevel::Debug, "Charging level solution");
    for (int b = 0; b<vehicles_requiring_charging.size(); ++b) {
        v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].charge_terminal.size(); ++k) {
            logger.log(LogLevel::Debug, "Charge level for vehicle index "+std::to_string(v)+" at opportunity "
                    +std::to_string(k)+" = "+std::to_string(cplex.getValue(variable.charge_level[b][k])));
        }
    }

    // Log the energy input solution
    int charge_time_window;
    for (int b = 0; b<vehicles_requiring_charging.size(); ++b) {
        v = vehicles_requiring_charging[b];
        for (int k = 0; k<vehicle[v].charge_terminal.size(); ++k) {  // Iterate over charging opportunities
            charge_time_window = vehicle[v].end_charge_time[k]-vehicle[v].start_charge_time[k];
            for (int t = 0; t<charge_time_window; ++t) {
                logger.log(LogLevel::Debug, "Energy input for vehicle index "+std::to_string(v)+" at terminal index "
                        +std::to_string(vehicle[v].charge_terminal[k]-1)+" at time "
                        +std::to_string(vehicle[v].start_charge_time[k]+t)+" = "
                        +std::to_string(cplex.getValue(variable.energy_input[b][k][t])));
            }
        }
    }

    // Log the power limit solutions
    logger.log(LogLevel::Debug, "Power limit solution");
    for (int s = 0; s<terminals_with_charging_station.size(); ++s) {
        logger.log(LogLevel::Debug, "Power limit for terminal index"+std::to_string(terminals_with_charging_station[s])
                +" = "+std::to_string(cplex.getValue(variable.terminal_charge_capacity[s])));
    }
}

// Function that solves the split model
double csp::solve_split_model(std::vector<Vehicle>& vehicle, std::vector<Terminal>& terminal, Data& data)
{
    logger.log(LogLevel::Debug, "Solving the split charging CSP problem");
    IloEnv env; // Create the CPLEX environment
    double optimal_objective;

    try {
        // Create a CPLEX model
        IloModel model(env);
        IloCplex cplex(model);

        // Turn off CPLEX output
        cplex.setOut(env.getNullStream());
        cplex.setWarning(env.getNullStream());

        std::vector<int> vehicles_requiring_charging;
        std::vector<int> terminals_with_charging_station;
        initialization::create_sets(vehicle, terminal, vehicles_requiring_charging, terminals_with_charging_station);

        // Create decision variables
        logger.log(LogLevel::Debug, "Creating decision variables");
        SplitModelVariable variable(vehicles_requiring_charging.size(), terminals_with_charging_station.size());

        csp::create_variables_split_model(env, vehicle, variable, vehicles_requiring_charging,
                terminals_with_charging_station);

        // Add constraints
        logger.log(LogLevel::Debug, "Adding constraints");
        std::vector<int> terminal_to_index(terminal.size(), -1);
        int index = 0;
        for (const auto& curr_terminal : terminal) {
            if (curr_terminal.is_charge_station) {
                terminal_to_index[curr_terminal.id-1] = index;
                ++index;
            }
        }
        csp::create_constraints_split_model(env, model, vehicle, data, variable, vehicles_requiring_charging,
                terminals_with_charging_station, terminal_to_index);

        // Add objective
        logger.log(LogLevel::Debug, "Adding objective function");
        IloExpr objective(env);
        csp::create_objective_split_model(objective, vehicle, data, variable, vehicles_requiring_charging,
                terminals_with_charging_station);

        // Solve the linear program
        IloObjective obj = IloMinimize(env, objective);
        model.add(obj);
        objective.end();

        logger.log(LogLevel::Debug, "Solving the LP");
        if (!cplex.solve()) {
            cplex.exportModel("../output/csp_split.lp");  // Write the LP to a .lp file
            logger.log(LogLevel::Error, "Failed to optimize LP");

            // Log terminal data
            logger.log(LogLevel::Info, "Terminal data and utilization");
            for (const auto& curr_terminal : terminal)
                curr_terminal.log_member_data();

            // Log the vehicle rotations
            logger.log(LogLevel::Info, "Vehicle rotations");
            for (const auto& curr_vehicle : vehicle)
                curr_vehicle.log_member_data();
            logger.log(LogLevel::Info, "Vehicle CSP member data");
            for (const auto& curr_vehicle : vehicle)
                curr_vehicle.log_csp_member_data();
            throw (-1);
        }

        if (data.log_csp_solution) {  // Write the LP to a .lp file and save the results in a .sol file
            cplex.exportModel(("../output/"+data.instance+"_csp_split.lp").c_str());
            cplex.writeSolution(("../output/"+data.instance+"_csp_split.sol").c_str());
        }

        //Display results and log the solution
        optimal_objective = cplex.getObjValue();
        /*logger.log(LogLevel::Debug, "Solution status = "+std::to_string(cplex.getStatus()));
        logger.log(LogLevel::Debug, "Solution value = "+std::to_string(cplex.getObjValue()));*/
        //csp::log_solution_split_model(cplex, vehicle, variable, vehicles_requiring_charging,
        //        terminals_with_charging_station);
    }
        // Catch exceptions thrown by CPLEX
    catch (IloException& exception) {
        std::cerr << "Error: " << exception << std::endl;
    }
    env.end();  // Clean up
    return optimal_objective;
}

// Function that updates all vehicle rotations with CSP data and selects the CSP model based on user parameters
double csp::select_optimization_model(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Data& data)
{
    initialization::update_vehicles(vehicle, trip, terminal);

    if (CSP_SOLUTION_TYPE==SolutionType::Uniform) {
        return csp::solve_uniform_model(vehicle, terminal, data);
    }
    if (CSP_SOLUTION_TYPE==SolutionType::Split)
        return csp::solve_split_model(vehicle, terminal, data);
    else {
        std::cerr << "Error: Invalid charging model" << std::endl;
        exit(1);
    }
}

// Function that updates a subset of vehicle rotations with CSP data and selects the CSP model based on user parameters
double csp::select_optimization_model(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Data& data, std::vector<int>& update_vehicle_indices)
{
    initialization::update_vehicles(vehicle, trip, terminal, data, update_vehicle_indices);

    if (CSP_SOLUTION_TYPE==SolutionType::Uniform) {
        return csp::solve_uniform_model(vehicle, terminal, data);
    }
    if (CSP_SOLUTION_TYPE==SolutionType::Split)
        return csp::solve_split_model(vehicle, terminal, data);
    else {
        std::cerr << "Error: Invalid charging model" << std::endl;
        exit(1);
    }
}

// Function that updates all vehicle rotations with CSP data and selects the CSP model based on user parameters
double csp::select_optimization_model(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Data& data, std::string type)
{
    initialization::update_vehicles(vehicle, trip, terminal);

    if (type=="Uniform") {
        return csp::solve_uniform_model(vehicle, terminal, data);
    }
    if (type=="Split")
        return csp::solve_split_model(vehicle, terminal, data);
    else {
        std::cerr << "Error: Invalid charging model" << std::endl;
        exit(1);
    }
}

// TODO: Write this function for only split model since there are many if statements throughout the code
// Function that updates a subset of vehicle rotations with CSP data and selects the CSP model based on user parameters
double csp::select_optimization_model(std::vector<Vehicle>& vehicle, std::vector<Trip>& trip,
        std::vector<Terminal>& terminal, Data& data, std::vector<int>& update_vehicle_indices, std::string type)
{
    initialization::update_vehicles(vehicle, trip, terminal, data, update_vehicle_indices);

    if (type=="Uniform") {
        return csp::solve_uniform_model(vehicle, terminal, data);
    }
    if (type=="Split")
        return csp::solve_split_model(vehicle, terminal, data);
    else {
        std::cerr << "Error: Invalid charging model" << std::endl;
        exit(1);
    }
}


