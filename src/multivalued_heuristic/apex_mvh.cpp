//
// Created by crl on 22/07/2024.
//

#include <cmath>
#include <fstream>
#include <iostream>
#include <multivalued_heuristic/apex_mvh.h>

#include <utility>

void APEX_MVH::insert(ApexPathPairPtr& ap, MapQueue& queue) {
    std::vector<ApexPathPairPtr>& relevant_aps = queue.get_open(ap->id);
    for (size_t i = 0; i < relevant_aps.size(); ) {
        if (!relevant_aps[i]->is_active) {
            relevant_aps[i] = std::move(relevant_aps.back());
            relevant_aps.pop_back();
            continue;
        }
        if (ap->update_nodes_by_merge_if_bounded(relevant_aps[i], eps)) {
            if ((ap->apex != relevant_aps[i]->apex) || (ap->path_node != relevant_aps[i]->path_node)) {
                relevant_aps[i]->is_active = false;
                relevant_aps[i] = std::move(relevant_aps.back());
                relevant_aps.pop_back();
                queue.insert(ap);
            }
            return;
        }
        ++i;
    }
    queue.insert(ap);
}


APEX_MVH::APEX_MVH(const AdjacencyMatrix& adj_matrix, EPS eps)
    : adj_matrix(adj_matrix), eps(std::move(eps)) {
    truncated_g_vecto.resize(adj_matrix.size() + 1);
}

// Returns true if `value1` is weakly dominated by `value2`
static bool is_dominated_dr(const std::vector<size_t>& value1, const std::vector<size_t>& value2) {
    for (size_t i = 1; i < value1.size(); ++i) {
        if (value1[i] < value2[i])
            return false;
    }
    return true;
}

bool local_dominance_check(const std::vector<size_t>& g,
                           const std::vector<std::vector<size_t>>& truncated_non_dominated_g) {
    for (const auto& truncated_g : truncated_non_dominated_g) {
        if (is_dominated_dr(g, truncated_g)) {
            return true;
        }
    }
    return false;
}

MultiValuedHeuristic
APEX_MVH::operator()(const size_t& target, const std::string& output_file) {
    start_time = std::clock();

    APEX_MVHSolutionSet frontiers;

    MapQueue open(adj_matrix.size() + 1);
    size_t K = adj_matrix.num_of_objectives;
    NodePtr source_node = std::make_shared<Node>(target, std::vector<size_t>(adj_matrix.num_of_objectives, 0),
                                                 std::vector<size_t>(K, 0));
    ApexPathPairPtr ap = std::make_shared<ApexPathPair>(
        source_node, source_node, std::vector<size_t>(K, 0));
    open.insert(ap);

    while (!open.empty()) {
        ap = open.pop();
        num_generation += 1;

        if (ap->is_active == false) {
            continue;
        }
        if (local_dominance_check(ap->apex->g, truncated_g_vecto[ap->id])) {
            continue;
        }

        // Update truncated_non_dominated_g list
        // Remove dominated truncated vectors and insert the new g value
        auto& truncated_list = truncated_g_vecto[ap->id];

        // Remove truncated vectors dominated by node->g
        truncated_list.erase(
            std::remove_if(truncated_list.begin(), truncated_list.end(),
                           [&ap](const std::vector<size_t>& truncated_g) {
                               for (size_t i = 1; i < ap->apex->g.size(); i++) {
                                   if (truncated_g[i] < ap->apex->g[i]) {
                                       return false;
                                   }
                               }
                               return true;
                           }),
            truncated_list.end());


        // Insert node->g if not dominated by existing truncated vectors
        auto insert_pos = std::lower_bound(truncated_list.begin(), truncated_list.end(),
                                           ap->apex->g);
        truncated_list.insert(insert_pos, ap->apex->g);

        num_expansion += 1;

        if (frontiers[ap->id].empty()) {
            frontiers[ap->id].push_back(ap);
        }
        else {
            bool dominated = false;
            for (auto& relevant_ap : frontiers[ap->id]) {
                auto before_value = relevant_ap->apex->g;
                if (relevant_ap->update_nodes_by_merge_if_bounded(ap, eps)) {
                    dominated = true;
                    break;
                }
            }
            if (!dominated) {
                frontiers[ap->id].push_back(ap);
            }
        }

        const std::vector<Edge>& outgoing_edges = adj_matrix[ap->id];
        for (const auto& outgoing_edge : outgoing_edges) {
            std::vector<size_t> new_apex_g(ap->apex->g);
            for (size_t i = 0; i < new_apex_g.size(); i++) new_apex_g[i] += outgoing_edge.cost[i];

            if (local_dominance_check(new_apex_g, truncated_g_vecto[outgoing_edge.target])) {
                continue;
            }
            ApexPathPairPtr next_ap = std::make_shared<ApexPathPair>(
                ap, outgoing_edge, std::vector<size_t>(K, 0));
            insert(next_ap, open);
        }
    }

    runtime = static_cast<float>(std::clock() - start_time);

    MultiValuedHeuristic mvh(adj_matrix.size() + 1);
    /***
    for (const auto& [node_id, apex_list] : frontiers) {
        if (apex_list.size() == 1) {
            mvh[node_id].push_back(apex_list[0]->apex->g);
            continue;
        }
        for (size_t i = 0; i < apex_list.size() - 1; ++i) {
            std::vector<size_t> g1 = apex_list[i]->apex->g;
            std::vector<size_t> g2 = apex_list[i + 1]->apex->g;
            std::vector<size_t> g = std::vector<size_t>(K, 0);
            for (int j = 0; j < g.size(); j++) {
                g[j] = std::min(g1[j], g2[j]);
            }
            mvh[node_id].push_back(g);
        }
    }
    ***/
    for (const auto& [node_id, apex_list] : frontiers) {
        for (const auto& ap: apex_list) {
            mvh[node_id].push_back(ap->apex->g);
        }
    }

    if (!output_file.empty()) {
        std::ofstream out(output_file);
        for (size_t i = 0; i < mvh.size(); i++) {
            for (const auto& g : mvh[i]) {
                out << i;
                for (size_t v : g) out << "\t" << v;
                out << "\n";
            }
        }
        std::cout << "finished writing results to file " << output_file << std::endl;
    }

    return mvh;
}
