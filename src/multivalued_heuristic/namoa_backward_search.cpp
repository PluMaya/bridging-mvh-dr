//
// Created by crl on 27/02/2026.
//

#include "multivalued_heuristic/namoa_backward_search.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <list>
#include <queue>

// ── Dominance helpers (DR mechanism, matching namoa_dr.cpp — uses f-values) ──

// Returns true if `value1` is weakly dominated by `value2`
static bool is_dominated_dr(const std::vector<size_t>& value1, const std::vector<size_t>& value2) {
    for (size_t i = 1; i < value1.size(); ++i) {
        if (value1[i] < value2[i])
            return false;
    }
    return true;
}

bool local_dominance_check(const NodePtr& node_ptr, std::vector<std::vector<size_t>> &truncated_non_dominated_g) {
    for (const auto& truncated_g : truncated_non_dominated_g) {
        if (is_dominated_dr(node_ptr->g, truncated_g)) {
            return true;
        }
    }
    return false;
}

bool global_dominance_check(const NodePtr& node_ptr, std::vector<std::vector<size_t>> &truncated_target_g) {
    // Check if node is dominated by comparing g values with truncated vectors
    for (const auto& truncated_g : truncated_target_g) {
        if (is_dominated_dr(node_ptr->f, truncated_g)) {
            return true;
        }
    }
    return false;
}

// ── Constructor ───────────────────────────────────────────────────────────────

NAMOABackwardSearch::NAMOABackwardSearch(const AdjacencyMatrix& adj_matrix)
    : adj_matrix(adj_matrix) {
    truncated_non_dominated_g.resize(adj_matrix.size() + 1);
}

// ── Main search ───────────────────────────────────────────────────────────────

MultiValuedHeuristic
NAMOABackwardSearch::operator()(const size_t& source,
                                const std::string& output_file) {
    start_time = std::clock();

    const size_t V = adj_matrix.size() + 1;
    const size_t K = adj_matrix.num_of_objectives;

    // Per-node Pareto fronts (replaces BOA's scalar min_g2).
    std::vector<std::list<NodePtr>> closed(V);

    std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNodeByFValue> open;

    // The source is the goal node: g = 0, h from SVH (should be ~0 at goal).
    auto source_node = std::make_shared<Node>(
        source, std::vector<size_t>(K, 0), std::vector<size_t>(K, 0));
    open.push(source_node);

    while (!open.empty()) {
        auto node = open.top();
        open.pop();
        ++num_generation;

        if (local_dominance_check(node, truncated_non_dominated_g[node->id]))
            continue;

        ++num_expansion;

        // Update truncated_non_dominated_g list
        // Remove dominated truncated vectors and insert the new g value
        auto& truncated_list = truncated_non_dominated_g[node->id];

        // Remove truncated vectors dominated by node->g
        truncated_list.erase(
            std::remove_if(truncated_list.begin(), truncated_list.end(),
                           [&node](const std::vector<size_t>& truncated_g) {
                               for (size_t i = 1; i < node->g.size(); i++) {
                                   if (truncated_g[i] <node->g[i]) {
                                       return false;
                                   }
                               }
                               return true;
                           }),
            truncated_list.end());


        // Insert node->g if not dominated by existing truncated vectors
        auto insert_pos = std::lower_bound(truncated_list.begin(), truncated_list.end(),
                                           node->g);
        truncated_list.insert(insert_pos, node->g);

        closed[node->id].push_back(node);

        for (const auto& edge : adj_matrix[node->id]) {
            std::vector<size_t> next_g(K);
            for (size_t i = 0; i < K; ++i)
                next_g[i] = node->g[i] + edge.cost[i];

            auto successor = std::make_shared<Node>(
                edge.target, next_g, std::vector<size_t>(K, 0));

            if (local_dominance_check(successor, truncated_non_dominated_g[edge.target]))
                continue;

            open.push(successor);
        }
    }

    MultiValuedHeuristic mvh(V);
    for (size_t i = 0; i < V; ++i) {
        mvh[i].reserve(closed[i].size());
        for (const auto& n : closed[i])
            mvh[i].push_back(n->g);
    }

    runtime = static_cast<float>(std::clock() - start_time);

    // Write output file: one line per (node, heuristic-vector) pair.
    std::ofstream out(output_file);
    for (size_t i = 0; i < V; ++i) {
        for (const auto& h_vec : mvh[i]) {
            out << i;
            for (size_t v : h_vec) out << '\t' << v;
            out << '\n';
        }
    }

    std::cout << "finished writing results to file " << output_file << std::endl;

    return mvh;
}
