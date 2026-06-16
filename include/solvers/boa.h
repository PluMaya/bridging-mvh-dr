//
// Created by crl on 14/07/2024.
//

#ifndef BOA_H
#define BOA_H
#include "abstract_solver.h"
#include <memory_resource>


class BOAStar : public AbstractSolver {
public:
    std::string get_solver_name() override { return "BOA*"; }
    std::pmr::unsynchronized_pool_resource node_pool;
    size_t num_local_dominance_check_dr = 0;
    size_t num_global_dominance_check = 0;

    explicit BOAStar(const AdjacencyMatrix& adj_matrix);

    void operator()(const size_t& source, const size_t& target,
                    const Heuristic& heuristic, SolutionSet& solutions,
                    unsigned int time_limit,
                    const std::string& solutions_file,
                    const std::string& stats_file);

    void operator()(const size_t& source, const size_t& target, const Heuristic& heuristic, SolutionSet& solutions,
                    unsigned time_limit) override {
    };
};

#endif // BOA_H
