//
// Created by crl on 14/07/2024.
//

#ifndef ABSTRACT_SOLVER_H
#define ABSTRACT_SOLVER_H
#include <algorithm>
#include <ctime>
#include <utility>

#include "data_structures/adjacency_matrix.h"
#include "data_structures/node.h"
#include "definitions.h"

using SolutionSet = std::vector<NodePtr>;

class AbstractSolver {
public:
    const AdjacencyMatrix& adj_matrix;
    EPS eps;

    std::clock_t start_time = std::clock(); // TODO: see that it is changed later
    size_t num_expansion = 0;
    size_t num_generation = 0;
    float runtime = 0.0f;
    bool time_limit_reached = false;

    virtual void init_search() {
        num_expansion = 0;
        num_generation = 0;
        time_limit_reached = false;
        start_time = std::clock();
    }

    AbstractSolver(const AdjacencyMatrix& adj_matrix, EPS eps)
        : adj_matrix(adj_matrix), eps(std::move(eps)) {
    }

    virtual ~AbstractSolver() = default;

    virtual void operator()(const size_t& source, const size_t& target,
                            const Heuristic& heuristic, SolutionSet& solutions,
                            unsigned int time_limit) = 0;

    virtual std::string get_solver_name() = 0;
};

#endif // ABSTRACT_SOLVER_H
