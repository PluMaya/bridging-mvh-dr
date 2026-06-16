#include "multivalued_heuristic/naive_dr_mvh_bridging.h"

#include <cassert>
#include <fstream>
#include <optional>
#include <queue>

NAIVE_DR_MVH_BRIDGING::NAIVE_DR_MVH_BRIDGING(
    const AdjacencyMatrix& adj_matrix, const EPS& eps)
    : AbstractSolver(adj_matrix, eps) {
    truncated_non_dominated_g.resize(adj_matrix.size() + 1);
}

bool NAIVE_DR_MVH_BRIDGING::dr_local_dominance_check(
    const std::vector<size_t>& g, size_t id) {
    num_dr_dominance_check += 1;
    for (const auto& truncated_g : truncated_non_dominated_g[id]) {
        bool cur_dominated = true;
        for (size_t i = 1; i < g.size(); i++) {
            if (g[i] < truncated_g[i]) {
                cur_dominated = false;
                break;
            }
        }
        if (cur_dominated) {
            return true;
        }
    }
    return false;
}

bool NAIVE_DR_MVH_BRIDGING::global_dominance_check(const NodePtr& node_ptr, const size_t& target_id) {
    num_global_dominance_check += 1;
    for (const auto& truncated_g : truncated_non_dominated_g[target_id]) {
        bool dominated = true;
        for (size_t i = 1; i < node_ptr->f.size(); i++) {
            if (node_ptr->f[i] < truncated_g[i]) {
                dominated = false;
                break;
            }
        }
        if (dominated) {
            return true;
        }
    }
    return false;
}

std::optional<std::pair<std::vector<size_t>, size_t>>
NAIVE_DR_MVH_BRIDGING::get_first_undominated_heuristic_value(
    const std::vector<size_t>& g_value, const size_t& target,
    const std::vector<std::vector<size_t>>& node_mvh,
    size_t start_idx) {
    for (size_t idx = start_idx; idx < node_mvh.size(); ++idx) {
        const auto& heuristic_value = node_mvh[idx];
        bool dominated = false;
        for (const auto& truncated_g : truncated_non_dominated_g[target]) {
            bool is_dominated = true;
            for (size_t i = 1; i < heuristic_value.size(); i++) {
                if (heuristic_value[i] + g_value[i] < truncated_g[i]) {
                    is_dominated = false;
                    break;
                }
            }
            if (is_dominated) {
                dominated = true;
                break;
            }
        }
        if (!dominated) {
            return std::make_pair(heuristic_value, idx);
        }
    }
    return std::nullopt;
}

void NAIVE_DR_MVH_BRIDGING::operator()(
    const size_t& source, const size_t& target,
    const MultiValuedHeuristic& heuristic, SolutionSet& solutions,
    unsigned int time_limit, const std::string& solutions_file, const std::string& stats_file) {
    init_search();
    start_time = std::clock();
    std::pmr::polymorphic_allocator<Node> alloc{&node_pool};
    std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNodeByFValue> open;

    const size_t num_obj = adj_matrix.num_of_objectives;
    std::vector<size_t> source_heuristic_value(num_obj, 0);
    NodePtr source_node = std::allocate_shared<Node>(alloc, source, std::vector<size_t>(num_obj, 0),
                                                     source_heuristic_value, nullptr, std::vector<size_t>(num_obj, 0));

    open.push(source_node);

    while (!open.empty()) {
        if (time_limit > 0 && (std::clock() - start_time) / CLOCKS_PER_SEC >= time_limit) {
            time_limit_reached = true;
            break;
        }
        auto node = open.top();
        open.pop();
        num_generation += 1;

        if (global_dominance_check(node, target)) {
            auto result = get_first_undominated_heuristic_value(node->g, target, heuristic[node->id], node->h_idx + 1);
            if (!result) {
                continue;
            }
            auto& [h_val, h_idx] = result.value();
            NodePtr new_node = std::allocate_shared<Node>(
                alloc, node->id, node->g, h_val, node->parent, node->c, h_idx);
            open.push(new_node);
            ++num_reinsertion;
            continue;
        }

        if (dr_local_dominance_check(node->g, node->id)) {
            continue;
        }

        // Update truncated_non_dominated_g list
        auto& truncated_list = truncated_non_dominated_g[node->id];

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

        auto insert_pos = std::lower_bound(truncated_list.begin(), truncated_list.end(),
                                           node->g);
        truncated_list.insert(insert_pos, node->g);

        num_expansion += 1;

        if (node->id == target) {
            solutions.push_back(node);
            continue;
        }

        const std::vector<Edge>& outgoing_edges = adj_matrix[node->id];

        for (const auto& outgoing_edge : outgoing_edges) {
            std::vector new_g(node->g);
            for (int i = 0; i < new_g.size(); i++) {
                new_g[i] += outgoing_edge.cost[i];
            }

            auto result = get_first_undominated_heuristic_value(new_g, target, heuristic[outgoing_edge.target]);
            if (!result) {
                continue;
            }

            if (dr_local_dominance_check(new_g, outgoing_edge.target)) {
                continue;
            }

            auto& [h_val, h_idx] = result.value();
            NodePtr successor_node = std::allocate_shared<Node>(
                alloc, outgoing_edge.target, new_g, h_val, node, outgoing_edge.cost, h_idx);
            open.push(successor_node);
        }
    }

    runtime = static_cast<float>(std::clock() - start_time);

    // Compute average size of truncated_non_dominated_g over active states only
    size_t total_truncated = 0;
    num_active_states = 0;
    size_t num_nodes = truncated_non_dominated_g.size();
    for (size_t i = 0; i < num_nodes; i++) {
        if (truncated_non_dominated_g[i].size() > 0) {
            num_active_states++;
            total_truncated += truncated_non_dominated_g[i].size();
        }
    }
    avg_truncated_size = num_active_states > 0 ? static_cast<float>(total_truncated) / static_cast<float>(num_active_states) : 0.0f;
    avg_pareto_size = 0.0f;

    if (!solutions_file.empty()) {
        std::ofstream sol_out(solutions_file);
        for (const auto& sol : solutions) {
            for (size_t i = 0; i < sol->g.size(); i++) {
                if (i > 0) sol_out << ",";
                sol_out << sol->g[i];
            }
            sol_out << "\n";
        }
    }

    if (!stats_file.empty()) {
        std::ofstream stats_out(stats_file);
        stats_out << runtime / CLOCKS_PER_SEC << "\t"
            << num_expansion << "\t" << num_generation << "\n";
    }
}
