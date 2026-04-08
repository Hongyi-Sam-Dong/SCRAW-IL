#pragma once
#include "planner/pibt.h"
#include "LNS/Parallel/GlobalManager.h"

namespace Planner {

class WPPLSolver {
public:
    int num_of_agents;
    int window_size_for_PATH;
    int window_size_for_EXEC;
    
    std::shared_ptr<LNS::Parallel::GlobalManager> lns;
    std::shared_ptr<HeuristicTable> heuristic_table;
    std::shared_ptr<std::vector<float> > map_weights;
    std::shared_ptr<SharedEnvironment> env;
    std::shared_ptr<LNS::Instance> instance;
    std::shared_ptr<std::vector<LNS::AgentInfo> > agent_infos;

    std::shared_ptr<PIBTSolver> pibt_solver;

    bool use_lns; // if not, just single-step pibt

    WPPLSolver(
        int rows,
        int cols,
        std::vector<int> & map,
        std::vector<float> & _map_weights,
        int num_of_agents,
        int window_size_for_PATH,
        int window_size_for_EXEC,
        int num_threads,
        int max_iterations,
        bool verbose=false
    );

    std::vector<int> solve(
        std::vector<int> & start_locations,
        std::vector<int> & goal_locations,
        double time_limit
    );

    int get_sum_of_cost() const;

}; 
    

} // namespace Planner
