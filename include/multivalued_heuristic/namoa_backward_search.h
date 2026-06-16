//
// Created by crl on 27/02/2026.
//

#ifndef NAMOA_BACKWARD_SEARCH_H
#define NAMOA_BACKWARD_SEARCH_H

#include <ctime>
#include <list>

#include "data_structures/adjacency_matrix.h"
#include "data_structures/node.h"
#include "definitions.h"

class NAMOABackwardSearch {
public:
    const AdjacencyMatrix& adj_matrix;

    std::vector<std::vector<std::vector<size_t>>> truncated_non_dominated_g;

    std::clock_t start_time = std::clock();
    float runtime = 0.0f;
    size_t num_expansion = 0;
    size_t num_generation = 0;

    explicit NAMOABackwardSearch(const AdjacencyMatrix& adj_matrix);
    virtual ~NAMOABackwardSearch() = default;

    MultiValuedHeuristic operator()(const size_t& source,
                                    const std::string& output_file);
};

#endif // NAMOA_BACKWARD_SEARCH_H
