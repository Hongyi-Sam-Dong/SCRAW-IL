#include "planner/wppl.h"

namespace Planner {

WPPLSolver::WPPLSolver(
    int rows,
    int cols,
    std::vector<int> & map,
    std::vector<float> & _map_weights,
    int num_of_agents,
    int window_size_for_PATH,
    int window_size_for_EXEC,
    int num_threads,
    int max_iterations,
    bool verbose
): 
    num_of_agents(num_of_agents),
    window_size_for_PATH(window_size_for_PATH),
    window_size_for_EXEC(window_size_for_EXEC) {

    env = std::make_shared<SharedEnvironment>();

    if (window_size_for_EXEC!=1) {
        std::cerr<<"only window_size_for_EXEC=1 is supported for now"<<std::endl;
        exit(-1);
    }

    if (window_size_for_EXEC>window_size_for_PATH) {
        std::cerr<<"window_size_for_EXEC should be <= window_size_for_PATH"<<std::endl;
        exit(-1);
    }

    // auto grid=std::make_shared<Grid>(map_path);
    env=std::make_shared<SharedEnvironment>();
    env->map=map;
    env->rows=rows;
    env->cols=cols;  
    env->num_of_agents=num_of_agents;

    // build fake start states and goals for now
    env->curr_states.resize(num_of_agents);
    env->goal_locations.resize(num_of_agents,vector<pair<int,int>>(1));

    map_weights=std::make_shared<std::vector<float> >(_map_weights);
    if (map_weights->size()!=rows*cols*5) {
        std::cerr<<"map weights' size doesn't match rows*cols*5: "<<map_weights->size()<<" "<<rows<<" "<<cols<<std::endl;
        exit(1);
    }

#ifdef NO_ROT
        bool consider_rotation=false;
#else
        bool consider_rotation=true;
        throw std::runtime_error("NO_ROT is not supported now, mainly because the pibt we used here!");
#endif

    // TODO: use env.get is dangerous! legacy code!
    heuristic_table=std::make_shared<HeuristicTable>(env.get(), map_weights, consider_rotation);
    heuristic_table->compute_weighted_heuristics();

    pibt_solver=std::make_shared<PIBTSolver>();
    instance=std::make_shared<LNS::Instance>(*env);
    agent_infos=std::make_shared<std::vector<LNS::AgentInfo> >();

    // TODO: agent_infos also maintains goal_location, etc.
    // a bad design! refactor it! the only thing we want is the disabled vector.
    for (int i=0; i<num_of_agents; ++i) {
        agent_infos->emplace_back();
        auto & agent_info=agent_infos->back();
        agent_info.id=i;
        // agent_info.disabled=false;
    }


    // TODO: make configurable
    bool async=true;
    // agent_infos
    int neighbor_size=8;
    // destroy_strategy is useless if ALNS is true.
    LNS::Parallel::destroy_heuristic destroy_strategy=LNS::Parallel::destroy_heuristic::RANDOMWALK;
    bool ALNS=true;
    double decay_factor=0.01;
    double reaction_factor=0.01;
    // TODO: LaCAM2 here is actually PIBT, legacy naming issue.
    std::string init_algo_name="LaCAM2";
    std::string replan_algo_name="PP";
    bool sipp=false;
    int window_size_for_CT=window_size_for_PATH;
    int window_size_for_CAT=window_size_for_PATH;
    int execution_window=window_size_for_EXEC;
    // TODO: support disabled agents
    bool has_disabled_agents=false;
    bool fix_ng_bug=true;
    int screen=0;

    // TODO: agent_infos

    lns=std::make_shared<LNS::Parallel::GlobalManager>(
        async,
        *instance, heuristic_table, map_weights, agent_infos,
        neighbor_size, destroy_strategy,
        ALNS, decay_factor, reaction_factor,
        init_algo_name, replan_algo_name, sipp,
        window_size_for_CT, window_size_for_CAT, window_size_for_PATH, execution_window,
        has_disabled_agents,
        fix_ng_bug,
        screen,
        verbose,
        num_threads,
        max_iterations
    );
}

std::vector<int> WPPLSolver::solve(
    std::vector<int> & start_locations,
    std::vector<int> & goal_locations,
    bool use_lns,
    double time_limit
) {
    int planning_window;

    if (use_lns) {
        planning_window=window_size_for_PATH;
    } else {
        planning_window=1;
    }

    // call pibt multiple times
    std::vector<int> action_choices={0,1,1,0,0,-1,-1,0,0,0};
    std::vector<int> map_size={env->rows,env->cols};
    std::vector<float> priorities;
    std::vector<int> locations;

    std::vector<std::vector<int> > paths(env->num_of_agents);

    // TODO: disable agents as in the original WPPL paper.
    for (int agent_idx=0;agent_idx<env->num_of_agents;++agent_idx) {
        // use EPIBT's priority function.
#ifdef NO_ROT
        float dist=heuristic_table->get(
            start_locations[agent_idx], 
            goal_locations[agent_idx]
        );
#else
        throw std::runtime_error("NO_ROT is not supported now");
#endif
        float priority = -dist;
        priorities.push_back(priority);
        int location=start_locations[agent_idx];
        locations.push_back(location/env->cols);
        locations.push_back(location%env->cols);
        paths[agent_idx].push_back(location);
    }

    for (int step=0;step<planning_window;++step) {
        std::vector<int> _actions=pibt_solver->solve(
            *heuristic_table,
            priorities,
            locations,
            action_choices,
            map_size,
            false
        );

        // take actions
        for (int agent_idx=0;agent_idx<env->num_of_agents;++agent_idx) {
            int y=locations[agent_idx*2];
            int x=locations[agent_idx*2+1];
            int _action=_actions[agent_idx];
            int dy=action_choices[_action*2];
            int dx=action_choices[_action*2+1];
            int ny=y+dy;
            int nx=x+dx;
            // update locations
            locations[agent_idx*2]=ny;
            locations[agent_idx*2+1]=nx;
            int next_location=ny*env->cols+nx;
            paths[agent_idx].push_back(next_location);
        }   
    }

    if (use_lns) {
        lns->reset();

        // copy pibt paths to lns paths
        for (int agent_idx=0;agent_idx<env->num_of_agents;++agent_idx) {
            auto & lns_path=lns->agents[agent_idx].path;
            auto & pibt_path=paths[agent_idx];
            lns_path.clear();
            for (int j=0;j<pibt_path.size();++j) {
                lns_path.nodes.emplace_back(pibt_path[j],-1);
            }
            lns_path.path_cost=lns->agents[agent_idx].getEstimatedPathLength(lns_path,goal_locations[agent_idx],heuristic_table);
        } 

        // TODO(rivers): should subtract other parts
        double _time_limit=time_limit;
        TimeLimiter time_limiter(_time_limit);
        bool succ=lns->run(time_limiter);

        if (!succ) {
            throw std::runtime_error("LNS failed to find a solution within the time limit");
        }

        // let's copy data back
        // TODO: when execution_window=1, these paths may have different lengths<=planned window size.
        // It is related to a bad implementation in the original WPPL. 
        // Namely, agents are considered disappeared once they reach the goal within the window.
        // We should fix this in the future.
        for (int agent_idx=0;agent_idx<env->num_of_agents;++agent_idx) {
            auto & lns_path=lns->agents[agent_idx].path;
            auto & path=paths[agent_idx];
            path.clear();
            for (int j=0;j<lns_path.size();++j) {
                path.push_back(lns_path[j].location);
            }
        }
    }

    // get actions 
    std::vector<int> actions;
    for (int agent_idx=0;agent_idx<env->num_of_agents;++agent_idx) {
        auto & path=paths[agent_idx];//lns->agents[agent_idx].path.nodes;
        int location = path[0];//.location;
        int next_location = path[1];//.location;
        int y=location/env->cols;
        int x=location%env->cols;
        int ny=next_location/env->cols;
        int nx=next_location%env->cols;
        int dy=ny-y;
        int dx=nx-x;
        int j=0;
        for (;j<action_choices.size()/2;++j) {
            if (dy==action_choices[j*2] && dx==action_choices[j*2+1]) {
                actions.push_back(Action(j));
                break;
            }
        }
        if (j>=action_choices.size()/2) {
            std::cerr<<"MAPFPlanner: a bug exists in action selection"<<std::endl;
            exit(-1);
        }
    }

    return actions;
}

};