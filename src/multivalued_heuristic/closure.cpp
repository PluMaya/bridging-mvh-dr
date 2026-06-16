//
// Created by crl on 14/02/2026.
//

#include "multivalued_heuristic/closure.h"

#include <fstream>
#include <iostream>
#include <queue>


struct ClosureNode {
    size_t id;
    std::vector<size_t> h;
};

struct CompareClosureNode {
    bool operator()(const ClosureNode& a, const ClosureNode& b) const {
        return a.h > b.h;  // lexicographic min-heap
    }
};


Closure::Closure(const AdjacencyMatrix& adj_matrix) : adj_matrix(adj_matrix) {}

MultiValuedHeuristic Closure::operator()(const MultiValuedHeuristic& heuristic,
    const std::string& output_file) {
    start_time = std::clock();
    const size_t num_obj = adj_matrix.num_of_objectives;
    MultiValuedHeuristic mvh_results(adj_matrix.size() + 1);

    // Pre-count total entries so we can reserve and heapify in O(N)
    size_t total = 0;
    for (const auto& h_node : heuristic) total += h_node.size();

    std::vector<ClosureNode> heuristic_values;
    heuristic_values.reserve(total);
    for (size_t i = 0; i < heuristic.size(); i++) {
        for (size_t j = 0; j < heuristic[i].size(); j++) {
            heuristic_values.push_back({i, std::vector<size_t>(
                heuristic[i][j].begin(),
                heuristic[i][j].begin() + num_obj)});
        }
    }

    // O(N) heap construction
    std::priority_queue<ClosureNode, std::vector<ClosureNode>, CompareClosureNode> open(
        CompareClosureNode{}, std::move(heuristic_values));

    // Per-node: track min of last objective as quick filter
    std::vector<size_t> min_last(adj_matrix.size() + 1, MAX_COST);

    // Maintain truncated results for faster dominance checks
    MultiValuedHeuristic truncated_mvh_results(adj_matrix.size() + 1);

    while (!open.empty()) {
        ClosureNode node = open.top();
        open.pop();
        num_generation += 1;

        // Quick filter: if last objective >= min seen, might still be non-dominated
        // for 3+ objectives, but for 2 objectives this is exact
        if (num_obj == 2) {
            if (min_last[node.id] <= node.h[1]) {
                continue;
            }
        } else {
            // For N objectives: check if dominated by any existing vector in truncated set
            bool dominated = false;
            for (const auto& existing : truncated_mvh_results[node.id]) {
                bool all_leq = true;
                for (size_t k = 1; k < num_obj; k++) {
                    if (existing[k] > node.h[k]) { all_leq = false; break; }
                }
                if (all_leq) { dominated = true; break; }
            }
            if (dominated) continue;
        }
        
        // NOTE relevant to only 2 objectives
        mvh_results[node.id].push_back(node.h);

                // Remove dominated truncated vectors and insert the new g value
        auto& truncated_list = truncated_mvh_results[node.id];

        // Remove truncated vectors dominated by node->g
        truncated_list.erase(
            std::remove_if(truncated_list.begin(), truncated_list.end(),
                           [&node](const std::vector<size_t>& existing) {
                               for (size_t k = 1; k < existing.size(); k++) {
                                   if (existing[k] < node.h[k]) {
                                       return false;
                                   }
                               }
                               return true;
                           }),
            truncated_list.end());

        // Insert node->g if not dominated by existing truncated vectors
        auto insert_pos = std::lower_bound(truncated_list.begin(), truncated_list.end(),
                                           node.h);
        truncated_list.insert(insert_pos, node.h);


        num_expansion += 1;
        if (node.h[num_obj - 1] < min_last[node.id]) {
            min_last[node.id] = node.h[num_obj - 1];
        }

        for (const auto& outgoing_edge : adj_matrix[node.id]) {
            std::vector<size_t> new_h(num_obj);
            for (size_t k = 0; k < num_obj; k++) {
                new_h[k] = node.h[k] + outgoing_edge.cost[k];
            }
            if (num_obj == 2) {
                if (min_last[outgoing_edge.target] <= new_h[num_obj - 1]) {
                    continue;
                }
            }
            bool dominated = false;
            for (const auto& existing : truncated_mvh_results[outgoing_edge.target]) {
                bool all_leq = true;
                for (size_t k = 1; k < num_obj; k++) {
                    if (existing[k] > new_h[k]) { all_leq = false; break; }
                }
                if (all_leq) { dominated = true; break; }
            }
            if (dominated) continue;
            open.push({outgoing_edge.target, std::move(new_h)});
        }
    }

    std::ofstream PlotOutput(output_file);

    for (size_t i = 0; i < adj_matrix.size() + 1; i++) {
        for (const auto& solution : mvh_results[i]) {
            PlotOutput << i;
            for (size_t k = 0; k < num_obj; k++) {
                PlotOutput << "\t" << solution[k];
            }
            PlotOutput << "\n";
        }
    }

    std::cout << "finished writing results to file " << output_file << std::endl;

    PlotOutput.close();
    runtime = static_cast<float>(std::clock() - start_time);

    return mvh_results;
}
