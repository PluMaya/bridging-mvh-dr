//
// Created by crl on 13/12/2024.
//

#ifndef NAMOA_DR_H
#define NAMOA_DR_H
#include "abstract_solver.h"
#include <memory_resource>


class NAMOAdr: public AbstractSolver{

public:
    std::pmr::unsynchronized_pool_resource node_pool;
    size_t num_local_dominance_check_dr = 0;
    size_t num_global_dominance_check = 0;
    float avg_closed_size = 0.0f;
    size_t num_active_states = 0;

    std::string get_solver_name() override { return "NAMOA_DR"; }

    explicit NAMOAdr(const AdjacencyMatrix &adj_matrix);

    void operator()(const size_t &source, const size_t &target,
                    const Heuristic &heuristic, SolutionSet &solutions,
                    unsigned int time_limit) override {
        (*this)(source, target, heuristic, solutions, time_limit, "", "");
    }

    void operator()(const size_t &source, const size_t &target,
                    const Heuristic &heuristic, SolutionSet &solutions,
                    unsigned int time_limit,
                    const std::string& solutions_file,
                    const std::string& stats_file);
};




#endif //NAMOA_DR_H
