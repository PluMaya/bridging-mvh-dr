//
// Created by crl on 22/07/2024.
//

#ifndef APEX_MVH_H
#define APEX_MVH_H

#include <ctime>

#include "data_structures/adjacency_matrix.h"
#include "data_structures/apex_path_pair.h"
#include "data_structures/map_queue.h"
#include "data_structures/node.h"
#include "definitions.h"
#include "solvers/apex.h"

#include <unordered_map>

using APEX_MVHSolutionSet = std::unordered_map<size_t, ApexSolutionSet>;

class APEX_MVH {
public:
    const AdjacencyMatrix& adj_matrix;
    EPS eps;

    std::clock_t start_time = std::clock();
    size_t num_expansion = 0;
    size_t num_generation = 0;
    float runtime = 0.0f;

    std::vector<std::vector<std::vector<size_t>>> truncated_g_vecto;

    ApexPathPairPtr last_solution = nullptr;

    MultiValuedHeuristic operator()(const size_t& target, const std::string& output_file);
    virtual ~APEX_MVH() = default;

    virtual void insert(ApexPathPairPtr& ap, MapQueue& queue);

    APEX_MVH(const AdjacencyMatrix& adj_matrix, EPS eps);


};


#endif // APEX_MVH_H
