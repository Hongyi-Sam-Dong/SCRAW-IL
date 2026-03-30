#pragma once
#include "common.h"

namespace Simulator {

class DiscreteSimulator {
public:
    DiscreteSimulator(
        size_t _seed,
        int _n_agents,
        int _rows,
        int _cols,
        std::vector<int> & _map,
        bool _one_shot
    );

    size_t seed;
    std::mt19937 rng;

    int n_agents;
    std::vector<int> all_agent_idxs;

    int rows;
    int cols;
    std::vector<int> map;
    std::vector<int> empty_locations;

    // simulator state except rng
    int timestep=0;
    std::vector<int> positions;
    std::vector<int> goals;
    int total_reached=0;

    bool one_shot=false;

    // TODO: there are multiple definitions for movements
    // we should unify them in common.h for example.
    const std::vector<std::pair<int,int> > movements = {
        {0,1}, // R
        {1,0}, // D
        {0,-1}, // L
        {-1,0}, // U
        {0,0} // W
    };

    void set_seed(size_t _seed);

    void reset();
    void reset_one_shot(std::vector<int> & start_positions, std::vector<int> & goal_positions);
    void step(std::vector<int> & actions);

    int _move(int curr_position, int movement_idx);
    bool _validate(
        std::vector<int> & curr_positions,
        std::vector<int> & next_positions
    );

    void _sample_initial_states();
    void _sample_goals(std::vector<int> & agent_idxs);

};

};