#pragma once
#include "common.h"
#include "DiscreteSimulator.h"
#include "planner/wppl.h"

// return total reached goals and trajectories of agents
std::tuple<int, std::vector<std::vector<int>>> simulate_discrete(
    std::shared_ptr<Planner::WPPLSolver> planner_ptr,
    std::vector<int> & start_positions,
    std::vector<int> & goal_positions,
    int sim_steps,
    double time_limit
);
