//
// Created by crl on 13/07/2024.
//

#ifndef SHORTEST_PATH_HEURISTIC_COMPUTER_H
#define SHORTEST_PATH_HEURISTIC_COMPUTER_H
#include "data_structures/adjacency_matrix.h"
#include "data_structures/node.h"
#include "definitions.h"

class ShortestPathHeuristic {
private:
    size_t                  source;
    std::vector<NodePtr>    all_nodes;

    void compute(size_t cost_idx, const AdjacencyMatrix& adj_matrix);
public:
    ShortestPathHeuristic(size_t source, size_t graph_size, const AdjacencyMatrix &adj_matrix);
    std::vector<size_t> operator()(size_t node_id);
    void set_all_to_zero(){
        for (auto n: all_nodes){
            n->h = {0, 0};
        }
    }
};

#endif // SHORTEST_PATH_HEURISTIC_COMPUTER_H
