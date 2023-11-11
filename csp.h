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
#include <deque>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/bron_kerbosch_all_cliques.hpp>

class SplitModelVariable {
public:
    std::vector<std::vector<IloNumVar>> charge_level;
    std::vector<std::vector<std::vector<IloNumVar>>> energy_input;
    std::vector<IloNumVar> terminal_charge_capacity;

    SplitModelVariable(int num_vehicles, int num_terminals)
            :charge_level(num_vehicles),
             energy_input(num_vehicles),
             terminal_charge_capacity(num_terminals)
    {

    }
};

class UniformModelVariable {
public:
    std::vector<std::vector<IloNumVar>> charge_level;
    std::vector<std::vector<IloNumVar>> energy_input;
    std::vector<IloNumVar> terminal_charge_capacity;

    UniformModelVariable(int num_vehicles, int num_terminals)
            :charge_level(num_vehicles),
             energy_input(num_vehicles),
             terminal_charge_capacity(num_terminals)
    {

    }
};

/*using Graph   = boost::adjacency_matrix<boost::undirectedS>;
using V       = Graph::vertex_descriptor;
using Clique  = std::deque<V>;
using Cliques = std::vector<Clique>;*/

using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS>;
using V = Graph::vertex_descriptor;
using Clique = std::vector<V>;
using Cliques = std::vector<Clique>;

struct Collector {
  Cliques& target;

  void clique(auto const& clique, Graph const&) const
  {
      for (auto& t = target.emplace_back(); Graph::vertex_descriptor v : clique)
          t.push_back(v);
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
        //if (period_index.size()>1) {
            logger.log(LogLevel::Verbose, "Overlap period index: "+vector_to_string(period_index));
            logger.log(LogLevel::Verbose, "Overlap within_period_duration: "+vector_to_string(within_period_duration));
            logger.log(LogLevel::Verbose, "Overlap start time: "+vector_to_string(start_time));
            logger.log(LogLevel::Verbose, "Overlap end time: "+vector_to_string(end_time));
        //}
    }
};

namespace initialization {
void update_vehicles(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&);
void update_vehicles(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data& data,
        std::vector<int>&);
void create_sets(std::vector<Vehicle>&, std::vector<Terminal>&, std::vector<int>&, std::vector<int>&);
void update_rotation_opportunities(std::vector<Vehicle>&, std::vector<Terminal>&);
}

namespace csp {
void create_variables_uniform_model(IloEnv&, const std::vector<Vehicle>&, UniformModelVariable&,
        const std::vector<int>&, const std::vector<int>&);
void create_constraints_uniform_model(IloEnv&, IloModel&, const std::vector<Vehicle>&,
        const std::vector<Terminal> terminal, UniformModelVariable&, const std::vector<int>&, const std::vector<int>&);
void create_objective_uniform_model(IloExpr&, const std::vector<Vehicle>&, UniformModelVariable&,
        const std::vector<int>&, const std::vector<int>&);
void log_solution_uniform_model(IloCplex&, const std::vector<Vehicle>&, const UniformModelVariable&,
        const std::vector<int>&, const std::vector<int>&);
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
double select_optimization_model(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data&, std::string);
double select_optimization_model(std::vector<Vehicle>&, std::vector<Trip>&, std::vector<Terminal>&, Data&,
        std::vector<int>&, std::string);
}

#endif  //EBUS_VNS_CSP_H
