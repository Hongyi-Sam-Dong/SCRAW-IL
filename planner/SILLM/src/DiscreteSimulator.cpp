#include "DiscreteSimulator.h"

namespace Simulator {

DiscreteSimulator::DiscreteSimulator(
    size_t _seed,
    int _n_agents,
    int _rows,
    int _cols,
    std::vector<int> & _map
): 
    seed(_seed),
    rng(_seed),
    n_agents(_n_agents),
    rows(_rows),
    cols(_cols),
    map(_map) {

    // sanity check
    if (map.size()!=rows*cols) {    
        throw std::runtime_error("the size of map is wrong: "+std::to_string(rows)+" "+std::to_string(cols)+" "+std::to_string(map.size()));
    }
    
    positions.resize(n_agents, -1);
    goals.resize(n_agents, -1);

    all_agent_idxs.resize(n_agents);
    std::iota(all_agent_idxs.begin(), all_agent_idxs.end(), 0);

    for (int r=0; r<rows; r++) {
        for (int c=0; c<cols; c++) {
            int pos = r*cols + c;
            if (map[pos]==0) {
                empty_locations.emplace_back(pos);
            }
        }
    }
}

void DiscreteSimulator::set_seed(size_t _seed) {
    seed=_seed;
    rng.seed(_seed);
}

void DiscreteSimulator::reset() {
    timestep=0;
    total_reached=0;

    _sample_initial_states();
    _sample_goals(all_agent_idxs);

}

void DiscreteSimulator::step(std::vector<int> & actions) {
    if (actions.size()!=n_agents) {
        throw std::runtime_error("wrong actions size "+std::to_string(actions.size())+" "+std::to_string(n_agents));
    }
    
    ++timestep;

    std::vector<int> next_positions(n_agents, -1);
    std::vector<int> reached_aids;
    for (int aid=0; aid<n_agents; aid++) {
        int curr_position = positions[aid];
        int movement_idx = actions[aid];
        next_positions[aid] = _move(curr_position, movement_idx);
    }

    // validate
    _validate(positions, next_positions);
    // update positions to next_positions
    std::swap(positions, next_positions);

    // check if agents reach goals
    for (int aid=0; aid<n_agents; aid++) {
        if (positions[aid]==goals[aid]) {
            ++total_reached;
            reached_aids.push_back(aid);
        }
    }
    _sample_goals(reached_aids);
}

int DiscreteSimulator::_move(
    int curr_position,
    int movement_idx
) {
    int y = curr_position / cols;
    int x = curr_position % cols;
    int ny = y + movements[movement_idx].first;
    int nx = x + movements[movement_idx].second;

    if (ny<0 || ny>=rows || nx<0 || nx>=cols || map[ny*cols+nx]>0) {
        throw std::runtime_error("error movement "+std::to_string(curr_position)+" "+std::to_string(movement_idx));
    }

    return ny*cols + nx;
}

bool DiscreteSimulator::_validate(
    std::vector<int> & curr_positions,
    std::vector<int> & next_positions
) {
    // check vertex collision
    std::unordered_map<int,int> next_positions2agent_idx;
    for (int agent_idx=0; agent_idx<n_agents; agent_idx++) {
        int pos = next_positions[agent_idx];
        if (next_positions2agent_idx.count(pos)>0) {
            return false;
        }
        next_positions2agent_idx[pos]=agent_idx;
    }

    // check edge collision
    for (int agent_idx=1; agent_idx<n_agents; agent_idx++) {
        int curr_position = curr_positions[agent_idx];
        int next_position = next_positions[agent_idx];

        auto iter = next_positions2agent_idx.find(curr_position);
        if (iter!=next_positions2agent_idx.end()) {
            // another agent wants to move to the current position of the agent
            int another_agent_idx = iter->second;
            // if another agent's current position is the same as the next position of the agent
            if (curr_positions[another_agent_idx]==next_position) {
                // cannot swap positions with another agent
                return false;
            }
        }
    }

    return true;
}

void DiscreteSimulator::_sample_initial_states() {
    std::vector<int> _empty_locations = empty_locations;
    std::shuffle(_empty_locations.begin(), _empty_locations.end(), rng);
    for (int aid=0; aid<n_agents; aid++) {
        positions[aid] = _empty_locations[aid];
    }
}

void DiscreteSimulator::_sample_goals(std::vector<int> & agent_idxs) {
    std::uniform_int_distribution<int> dist(0, empty_locations.size()-1);
    for (int aid: agent_idxs) {
        int idx = dist(rng);
        goals[aid] = empty_locations[idx];
    }
}

};