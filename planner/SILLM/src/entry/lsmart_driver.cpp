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
    int plan_window_size,
    double plan_time_limit
) {
    vector<int> start_locations = instance.getStartLocations();
    vector<int> goal_locations = instance.getGoalLocations();
    vector<Task> goal_tasks = instance.getGoalTasks();
    const int num_agents = start_locations.size();

    std::cout << "SILLM: gen_mapf_plan before reset_one_shot" << std::endl;
    simulator_ptr->reset_one_shot(start_locations, goal_locations);
    std::cout << "SILLM: gen_mapf_plan after reset_one_shot" << std::endl;

    vector<vector<tuple<int, int, double, int>>> new_mapf_plan(num_agents);
    vector<bool> task_finished(num_agents, false);

    for (int agent_id = 0; agent_id < num_agents; agent_id++) {
        const int start_loc = simulator_ptr->positions[agent_id];
        new_mapf_plan[agent_id].emplace_back(
            instance.graph->getRowCoordinate(start_loc),
            instance.graph->getColCoordinate(start_loc),
            0.0,
            -1
        );
    }

    for (int step = 0; step < plan_window_size; step++) {
        std::cout << "SILLM: gen_mapf_plan before solve step " << step
                  << std::endl;
        std::vector<int> actions = planner_ptr->solve(
            simulator_ptr->positions,
            simulator_ptr->goals,
            plan_time_limit
        );
        std::cout << "SILLM: gen_mapf_plan after solve step " << step
                  << std::endl;

        std::cout << "SILLM: gen_mapf_plan before simulator step " << step
                  << std::endl;
        simulator_ptr->step(actions);
        std::cout << "SILLM: gen_mapf_plan after simulator step " << step
                  << std::endl;

        for (int agent_id = 0; agent_id < num_agents; agent_id++) {
            const int curr_loc = simulator_ptr->positions[agent_id];
            int task_id = -1;
            if (!task_finished[agent_id] &&
                curr_loc == goal_tasks[agent_id].loc) {
                task_id = goal_tasks[agent_id].id;
                task_finished[agent_id] = true;
            }

            new_mapf_plan[agent_id].emplace_back(
                instance.graph->getRowCoordinate(curr_loc),
                instance.graph->getColCoordinate(curr_loc),
                static_cast<double>(step + 1),
                task_id
            );
        }
        std::cout << "SILLM: gen_mapf_plan finished bookkeeping step " << step
                  << std::endl;
    }

    return new_mapf_plan;
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
        ("port", po::value<int>(), "port number")
        ("portNum", po::value<int>(), "alias of --port")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
        ("map_fp,m", po::value<std::string>(), "map file path")
        ("map", po::value<std::string>(), "alias of --map_fp")
        ("num_agents,n", po::value<int>(), "number of agents")
        ("agentNum", po::value<int>(), "alias of --num_agents")
        ("random_seed,r", po::value<size_t>()->default_value(0), "random seed")
        ("plan_window_size", po::value<int>()->default_value(10), "window size of the plan to submit to the server")
        ("plan_time_limit", po::value<double>(), "time limit for each planning call in seconds")
        ("cutoffTime", po::value<double>(), "alias of --plan_time_limit")
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

    auto get_int_option = [&](const char* primary, const char* alias,
                              int default_value) {
        if (vm.count(primary)) {
            return vm[primary].as<int>();
        }
        if (vm.count(alias)) {
            return vm[alias].as<int>();
        }
        return default_value;
    };

    auto get_double_option = [&](const char* primary, const char* alias,
                                 double default_value) {
        if (vm.count(primary)) {
            return vm[primary].as<double>();
        }
        if (vm.count(alias)) {
            return vm[alias].as<double>();
        }
        return default_value;
    };

    auto get_string_option = [&](const char* primary, const char* alias) {
        if (vm.count(primary)) {
            return vm[primary].as<std::string>();
        }
        if (vm.count(alias)) {
            return vm[alias].as<std::string>();
        }
        return std::string();
    };

    const int port = get_int_option("port", "portNum", -1);
    const std::string map_fp = get_string_option("map_fp", "map");
    const int num_agents = get_int_option("num_agents", "agentNum", -1);
    const double plan_time_limit =
        get_double_option("plan_time_limit", "cutoffTime", 1.0);

    if (port < 0) {
        throw po::error("missing required option: --port or --portNum");
    }
    if (map_fp.empty()) {
        throw po::error("missing required option: --map_fp or --map");
    }
    if (num_agents < 0) {
        throw po::error("missing required option: --num_agents or --agentNum");
    }

    size_t seed = vm["random_seed"].as<size_t>();
    srand(seed);  // Set the random seed for reproducibility

    // Set up logger
    auto console_logger = spdlog::default_logger()->clone("Planner");
    spdlog::set_default_logger(console_logger);

    std::string grid_map_fp = map_fp;
    if (grid_map_fp.size() >= 5 &&
        grid_map_fp.substr(grid_map_fp.size() - 5) == ".json") {
        const auto slash_pos = grid_map_fp.find_last_of("/\\");
        const std::string dir = slash_pos == std::string::npos
                                    ? ""
                                    : grid_map_fp.substr(0, slash_pos + 1);
        const std::string basename = slash_pos == std::string::npos
                                         ? grid_map_fp
                                         : grid_map_fp.substr(slash_pos + 1);
        grid_map_fp = dir + "_sillm_auto_" +
                      basename.substr(0, basename.size() - 5) + ".map";
    }

    // build the simulator and planner
    auto grid_ptr = std::make_shared<Grid>(grid_map_fp);

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
    rpc::client client("127.0.0.1", port);
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
        try {
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
                spdlog::error("SILLM Driver: mapf_instance not found in the JSON "
                              "from server. Exit...");
                exit(1);
            }

            if (!result_json["mapf_instance"].contains("starts") ||
                !result_json["mapf_instance"].contains("goals")) {
                spdlog::error("SILLM Driver: starts or goals not found in the "
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
            std::cout << "SILLM: before gen_mapf_plan" << std::endl;
            auto new_mapf_plan = gen_mapf_plan(
                instance, 
                planner_ptr, 
                simulator_ptr, 
                vm["plan_window_size"].as<int>(), 
                plan_time_limit
            );
            std::cout << "SILLM: after gen_mapf_plan" << std::endl;

            // Send new plan
            json stats = {{"n_mapf_calls", n_mapf_calls},
                          {"n_rule_based_calls", n_rule_based_calls},
                          {"sum_of_cost", planner_ptr->get_sum_of_cost()}};
            json new_plan_json = {
                {"success", true},
                {"plan", new_mapf_plan},
                {"congested", congested(new_mapf_plan)},
                {"stats", stats.dump()},
            };
            std::cout << "SILLM: before add_plan" << std::endl;
            client.call("add_plan", new_plan_json.dump());
        } catch (const std::exception& e) {
            std::cerr << "SILLM planner exception: " << e.what() << std::endl;
            throw;
        }
    }

    cout << "Planner finished!" << endl;
    return 0;
}
