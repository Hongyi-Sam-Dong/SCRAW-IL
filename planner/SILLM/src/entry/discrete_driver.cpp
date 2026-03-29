#include "common.h"
#include "DiscreteSimulator.h"
#include "Grid.h"
#include "planner/wppl.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>

namespace po = boost::program_options;

int main(int argc, char** argv) {
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        // ("inputFolder", po::value<std::string>()->default_value("."), "input folder")
        ("map_fp,m", po::value<std::string>()->required(), "map file path")
        ("num_agents,n", po::value<int>()->required(), "number of agents")
        ("random_seed,r", po::value<size_t>()->default_value(0), "random seed")
        ("sim_steps,s", po::value<int>()->default_value(100), "number of steps to simulate")
        ("plan_time_limit", po::value<double>()->default_value(1.0), "time limit for each planning call")
        ("lns_plan_window", po::value<int>()->default_value(15), "planning window size for LNS")
        ("lns_exec_window", po::value<int>()->default_value(1), "execution window size for LNS")
        ("lns_num_threads,t", po::value<int>()->default_value(1), "number of threads for LNS parallelization")
        ("lns_max_iters", po::value<int>()->default_value(1000), "maximum number of LNS iterations")
    ;
        
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);

    auto grid_ptr = std::make_shared<Grid>(vm["map_fp"].as<std::string>());
    int num_agents = vm["num_agents"].as<int>();

    // TODO: make map weights configurable
    auto map_weights_ptr = std::make_shared<std::vector<float> >(grid_ptr->map.size()*5, 1.0f);

    bool use_lns = vm["lns_num_threads"].as<int>() >0 && vm["lns_max_iters"].as<int>() > 0;

    // TODO: make planner configurable
    auto planner_ptr = std::make_shared<Planner::WPPLSolver>(
        grid_ptr->rows,
        grid_ptr->cols,
        grid_ptr->map,
        *map_weights_ptr,
        num_agents,
        vm["lns_plan_window"].as<int>(), // window_size_for_PATH
        vm["lns_exec_window"].as<int>(), // window_size_for_EXEC
        vm["lns_num_threads"].as<int>(), // num_threads
        vm["lns_max_iters"].as<int>(), // max_iterations
        false // verbose
    );

    auto simulator_ptr = std::make_shared<Simulator::DiscreteSimulator>(
        vm["random_seed"].as<size_t>(),
        num_agents,
        grid_ptr->rows,
        grid_ptr->cols,
        grid_ptr->map
    );
    
    for (int step=0; step < vm["sim_steps"].as<int>(); step++) {
        std::vector<int> actions = planner_ptr->solve(
            simulator_ptr->positions,
            simulator_ptr->goals,
            use_lns, // use_lns
            vm["plan_time_limit"].as<double>() // time_limit
        );
        simulator_ptr->step(actions);
    }

}