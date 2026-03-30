/* Copyright (C) Jiaoyang Li
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Confidential
 * Written by Jiaoyang Li <jiaoyanl@usc.edu>, May 2020
 */

/*driver.cpp
 * Solve a MAPF instance on 2D grids.
 */
#include <rpc/client.h>
#include <rpc/rpc_error.h>

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>

#include "Graph.h"
#include "Instance.h"
// #include "PBS.h"
#include "common.h"

#include "Grid.h"
#include "planner/wppl.h"
#include "DiscreteSimulator.h"

vector<vector<tuple<int, int, double, int>>> gen_mapf_plan(
    Instance & instance,
    std::shared_ptr<Planner::WPPLSolver> & planner_ptr,
    std::shared_ptr<Simulator::DiscreteSimulator> & simulator_ptr,
    int sim_steps,
    double plan_time_limit
) {

}

/* Main function */
int main(int argc, char** argv) {
    std::cout << "Starting SILLM Driver..." << std::endl;
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "produce help message")
        // ("inputFolder", po::value<std::string>()->default_value("."), "input folder")
        ("ip", po::value<std::string>()->default_value("127.0.0.1"), "IP address")
        ("port", po::value<int>()->required(), "port number")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
        ("map_fp,m", po::value<std::string>()->required(), "map file path")
        ("num_agents,n", po::value<int>()->required(), "number of agents")
        ("random_seed,r", po::value<size_t>()->default_value(0), "random seed")
        ("sim_steps", po::value<int>()->default_value(100), "number of steps to simulate")
        ("plan_time_limit", po::value<double>()->default_value(1.0), "time limit for each planning call")
        ("lns_plan_window", po::value<int>()->default_value(15), "planning window size for LNS")
        ("lns_exec_window", po::value<int>()->default_value(1), "execution window size for LNS. just ignore it for now.")
        ("lns_num_threads,t", po::value<int>()->default_value(1), "number of threads for LNS parallelization")
        ("lns_max_iters", po::value<int>()->default_value(1000), "maximum number of LNS iterations. If lns_max_iters is set, plan_time_limit will be ignored.")
    ;
        
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        cout << desc << endl;
        return 1;
    }

    po::notify(vm);

    int seed = vm["random_seed"].as<int>();
    srand(seed);  // Set the random seed for reproducibility

    // Set up logger
    auto console_logger = spdlog::default_logger()->clone("Planner");
    spdlog::set_default_logger(console_logger);

    // build the simulator and planner
    int num_agents = vm["num_agents"].as<int>();
    std::string map_fp = vm["map_fp"].as<std::string>();
    auto grid_ptr = std::make_shared<Grid>(map_fp);

    // TODO: make map weights configurable
    auto map_weights_ptr = std::make_shared<std::vector<float> >(grid_ptr->map.size()*5, 1.0f);

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

    // one shot to generate windowed plan
    const bool one_shot = true;
    auto simulator_ptr = std::make_shared<Simulator::DiscreteSimulator>(
        vm["random_seed"].as<size_t>(),
        num_agents,
        grid_ptr->rows,
        grid_ptr->cols,
        grid_ptr->map,
        one_shot
    );

    ///////////////////////////////////////////////////////////////////////////
    // int prev_last_task_id = 0;  // last task id from the previous iteration
    // vector<Task> prev_goal_locs;
    set<int> finished_tasks_id;
    int screen = vm["screen"].as<int>();

    // Create a graph, heuristic will be computed only once in the graph
    auto graph = make_shared<Graph>(map_fp, screen);

    // Stats
    int n_mapf_calls = 0;        // number of MAPF calls
    int n_rule_based_calls = 0;  // number of rule-based calls

    // We must have enough empty locations for agents to "wait in queue".
    // Since we do not consider window in this planner, it is not okay for
    // agents to have the goal. Therefore when agent j tries to go to a
    // location which is the goal of agent i, it must go to an empty space
    // first.
    if (graph->nEmptyLocations() < num_agents) {
        spdlog::error("Not enough empty locations for agents to wait in queue. "
                      "Please increase the number of empty locations.");
        exit(-1);
    }

    // We assume the server is already running at this point.
    rpc::client client("127.0.0.1", vm["portNum"].as<int>());
    // client.set_timeout(5000);  // in ms

    // Wait for the server to initialize
    int trial = 0;
    while (!client.call("is_initialized").as<bool>()) {
        if (screen > 0) {
            // printf("%d Waiting for server to initialize...\n", trial);
            trial++;
        }
    }
    
    while (true) {
        // Get the current simulation tick
        bool invoke_planner = client.call("invoke_planner").as<bool>();

        // Skip planning until the simulation step is a multiple of the
        // simulation window
        if (!invoke_planner) {
            continue;
        }

        string result_message = client.call("get_location").as<string>();

        auto result_json = json::parse(result_message);
        if (!result_json["initialized"].get<bool>()) {
            printf("Planner not initialized! Retrying\n");
            sleep(1);
            continue;
        }

        // Obtain the MAPF instance
        if (!result_json.contains("mapf_instance")) {
            spdlog::error("PBS Driver: mapf_instance not found in the JSON "
                          "from server. Exit...");
            exit(1);
        }

        if (!result_json["mapf_instance"].contains("starts") ||
            !result_json["mapf_instance"].contains("goals")) {
            spdlog::error("PBS Driver: starts or goals not found in the "
                          "mapf_instance from server. Exit...");
            exit(1);
        }
        json mapf_instance = result_json["mapf_instance"];

        Instance instance(graph, screen);
        // instance.loadAgents(commit_cut, new_finished_tasks_id);
        instance.loadAgents(mapf_instance);

        instance.printAgents();

        // run
        n_mapf_calls += 1;

        // TODO: write a function like gen_mapf_plan to generate a windowed plan, and submit the plan to the server.
        // replace the following code with WPPL planner to generate a windowed plan submitted to the server.
        // please refer to the discrete_driver.cpp for an example of how to simulate,
        // and the following PBS code for what everythin means and how to submit a plan to the server.
        auto new_mapf_plan = gen_mapf_plan(instance, planner_ptr, simulator_ptr, vm["sim_steps"].as<int>(), vm["plan_time_limit"].as<double>());

        // Send new plan
        json stats = {{"n_mapf_calls", n_mapf_calls},
                      {"n_rule_based_calls", n_rule_based_calls}};
        json new_plan_json = {
            {"success", true},
            {"plan", new_mapf_plan},
            {"congested", congested(new_mapf_plan)},
            {"stats", stats.dump()},
        };
        client.call("add_plan", new_plan_json.dump());
    }

    cout << "Planner finished!" << endl;
    return 0;
}