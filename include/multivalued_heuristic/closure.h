//
// Created by crl on 14/02/2026.
//

#ifndef CLOSURE_H
#define CLOSURE_H

#include <ctime>

#include "data_structures/adjacency_matrix.h"
#include "data_structures/apex_path_pair.h"
#include "data_structures/map_queue.h"
#include "data_structures/node.h"
#include "definitions.h"

#include <unordered_map>

class Closure {
public:
    const AdjacencyMatrix& adj_matrix;

    std::clock_t start_time = std::clock();
    size_t num_expansion = 0;
    size_t num_generation = 0;
    float runtime = 0.0f;

    explicit Closure(const AdjacencyMatrix& adj_matrix);
    MultiValuedHeuristic operator()(const MultiValuedHeuristic& heuristic, const std::string& output_file);
    virtual ~Closure() = default;
};


#endif //CLOSURE_H
