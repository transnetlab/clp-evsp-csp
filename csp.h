#ifndef EBUS_VNS_CSP_H
#define EBUS_VNS_CSP_H

#include "logger.h"
#include "constants.h"
#include "vehicle.h"
#include "helpers.h"
#include <sstream>
#include <ilcplex/ilocplex.h>
#include <vector>
#include <map>
#include <set>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/bron_kerbosch_all_cliques.hpp>

class SplitModelVariable {
public:
    std::vector<std::vector<IloNumVar>> charge_level_var;
    std::vector<std::vector<std::vector<IloNumVar>>> energy_input_var;
    std::vector<IloNumVar> charge_terminal_capacity_var;

    SplitModelVariable(int num_vehicles, int num_terminals)
            :charge_level_var(num_vehicles),
             energy_input_var(num_vehicles),
             charge_terminal_capacity_var(num_terminals)
    {

    }
};

class Visitor {
public:
    template<typename Clique, typename Graph>
    void clique(const Clique& c, const Graph& g)
    {
        // Write the clique details to a file with vertices of each clique in a new line
        std::ofstream clique_file;
        clique_file.open("../output/cliques.txt", std::ios_base::app);
        for (auto it = c.begin(); it!=c.end(); ++it)
            clique_file << *it << " ";
        clique_file << std::endl;
        clique_file.close();

        // Display the cliques
        /*std::cout << "Clique: ";
        for (auto vertex : c)
            std::cout << vertex << " ";
        std::cout << std::endl;*/
    }
};

// Function object to handle found cliques
class CliqueCollector {
public:
    std::vector<std::vector<int>> cliques;

    template<typename Clique, typename Graph>
    void operator()(const Clique& c, Graph& /* g */)
    {
        std::vector<int> temp;
        for (auto iter = c.begin(); iter!=c.end(); ++iter) {
            temp.push_back(static_cast<int>(*iter));
        }
        cliques.push_back(temp);
    }
};

// This class splits the charge opportunity into sub-intervals where price is constant
class ChargeInterval {
public:
    std::vector<int> period_index;  // Index of the price period in which the interval lies
    std::vector<int> within_period_duration;  // Duration of the interval. Each window lies exclusively in a single price period
    std::vector<int> start_time;
    std::vector<int> end_time;

    // Print members of the class
    void log_member_data(Logger& logger) const
    {
        // Print this only if charging opportunities has at least two elements
        if (period_index.size()>1) {
            logger.log(LogLevel::Verbose, "Overlap period index: "+vector_to_string(period_index));
            logger.log(LogLevel::Verbose, "Overlap within_period_duration: "+vector_to_string(within_period_duration));
            logger.log(LogLevel::Verbose, "Overlap start time: "+vector_to_string(start_time));
            logger.log(LogLevel::Verbose, "Overlap end time: "+vector_to_string(end_time));
        }
    }
};

namespace initialization {
void update_vehicles(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&);
void update_vehicles(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data& data,
        std::vector<int>&);
void create_sets(std::vector<Vehicle>&, std::vector<Terminal>&, std::vector<int>&, std::vector<int>&, Logger&);
}

namespace csp {
void create_variables_uniform_model(IloEnv&,
        const std::vector<Vehicle>&,
        std::vector<std::vector<IloNumVar>>&,
        std::vector<std::vector<IloNumVar>>&,
        std::vector<IloNumVar>&,
        const std::vector<int>&,
        const std::vector<int>&);
void create_constraints_uniform_model(IloEnv&,
        IloModel&,
        const std::vector<Vehicle>&,
        const std::vector<std::vector<IloNumVar>>&,
        const std::vector<std::vector<IloNumVar>>&,
        const std::vector<IloNumVar>&,
        const std::vector<int>&,
        const std::vector<int>&);
void create_objective_uniform_model(IloExpr&,
        const std::vector<Vehicle>&,
        const std::vector<std::vector<IloNumVar>>&,
        const std::vector<IloNumVar>&,
        const std::vector<int>&,
        const std::vector<int>&,
        Logger&);
void log_solution_uniform_model(IloCplex&,
        const std::vector<Vehicle>&,
        const std::vector<std::vector<IloNumVar>>&,
        const std::vector<std::vector<IloNumVar>>&,
        const std::vector<IloNumVar>&,
        const std::vector<int>&,
        const std::vector<int>&,
        Logger&);
double solve_uniform_model(std::vector<Vehicle>&, std::vector<Terminal>&, Data& data);

void create_variables_split_model(IloEnv&, const std::vector<Vehicle>&, SplitModelVariable&, const std::vector<int>&,
        const std::vector<int>&);
void create_constraints_split_model(IloEnv&, IloModel&, const std::vector<Vehicle>&, const Data&, SplitModelVariable&,
        const std::vector<int>&, const std::vector<int>&, const std::vector<int>&);
void create_objective_split_model(IloExpr&, const std::vector<Vehicle>&, Data&, SplitModelVariable&,
        const std::vector<int>&, const std::vector<int>&);
void log_solution_split_model(IloCplex&, const std::vector<Vehicle>&, SplitModelVariable&, const std::vector<int>&,
        const std::vector<int>&);
double solve_split_model(std::vector<Vehicle>&, std::vector<Terminal>&, Data& data);

double select_optimization_model(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data&);
double select_optimization_model(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data&,
        std::vector<int>&);
}

#endif  //EBUS_VNS_CSP_H
