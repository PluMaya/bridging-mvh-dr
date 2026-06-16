#include "multivalued_heuristic/namoa.h"

#include <cassert>
#include <chrono>
#include <fstream>
#include <optional>
#include <queue>

NAMOA::NAMOA(const AdjacencyMatrix& adj_matrix, const EPS& eps)
    : AbstractSolver(adj_matrix, eps) {
    pareto_list.resize(adj_matrix.size() + 1);
}

// Returns true if g is dominated by any element already in pareto_list[id].
bool NAMOA::full_local_dominance_check(const std::vector<size_t>& g, size_t id) {
    num_full_dominance_check += 1;
    for (const auto& pareto_elem : pareto_list[id]) {
        bool dominated = true;
        for (size_t i = 0; i < pareto_elem->g.size(); i++) {
            if (pareto_elem->g[i] > g[i]) {
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

// Returns true if node->f is dominated by any solution already in pareto_list[target_id].
bool NAMOA::global_dominance_check(const NodePtr& node_ptr, const size_t& target_id) {
    num_global_dominance_check += 1;
    for (const auto& pareto_elem : pareto_list[target_id]) {
        bool dominated = true;
        for (size_t i = 0; i < pareto_elem->g.size(); i++) {
            if (pareto_elem->g[i] > node_ptr->f[i]) {
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

// Returns the first heuristic value h (starting from start_idx) such that g+h is
// not dominated by any element in pareto_list[target].
std::optional<std::pair<std::vector<size_t>, size_t>>
NAMOA::get_first_undominated_heuristic_value(
    const std::vector<size_t>& g_value, const size_t& target,
    const std::vector<std::vector<size_t>>& node_mvh,
    size_t start_idx) {
    for (size_t idx = start_idx; idx < node_mvh.size(); ++idx) {
        const auto& h = node_mvh[idx];
        bool dominated = false;
        for (const auto& pareto_elem : pareto_list[target]) {
            bool dom = true;
            for (size_t i = 0; i < h.size(); i++) {
                if (pareto_elem->g[i] > g_value[i] + h[i]) {
                    dom = false;
                    break;
                }
            }
            if (dom) {
                dominated = true;
                break;
            }
        }
        if (!dominated) {
            return std::make_pair(h, idx);
        }
    }
    return std::nullopt;
}

void NAMOA::operator()(
    const size_t& source, const size_t& target,
    const MultiValuedHeuristic& heuristic, SolutionSet& solutions,
    unsigned int time_limit, const std::string& solutions_file, const std::string& stats_file) {
    init_search();
    start_time = std::clock();
    std::pmr::polymorphic_allocator<Node> alloc{&node_pool};
    std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNodeByFValue> open;

    const size_t num_obj = adj_matrix.num_of_objectives;
    std::vector<size_t> source_h(num_obj, 0);
    NodePtr source_node = std::allocate_shared<Node>(
        alloc, source, std::vector<size_t>(num_obj, 0), source_h, nullptr,
        std::vector<size_t>(num_obj, 0));

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
            auto result = get_first_undominated_heuristic_value(
                node->g, target, heuristic[node->id], node->h_idx + 1);
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

        if (full_local_dominance_check(node->g, node->id)) {
            continue;
        }

        num_expansion += 1;
        pareto_list[node->id].push_back(node);

        if (node->id == target) {
            solutions.push_back(node);
            continue;
        }

        const std::vector<Edge>& outgoing_edges = adj_matrix[node->id];

        for (const auto& outgoing_edge : outgoing_edges) {
            std::vector<size_t> new_g(node->g);
            for (size_t i = 0; i < new_g.size(); i++) {
                new_g[i] += outgoing_edge.cost[i];
            }

            auto result = get_first_undominated_heuristic_value(
                new_g, target, heuristic[outgoing_edge.target]);
            if (!result) {
                continue;
            }

            if (full_local_dominance_check(new_g, outgoing_edge.target)) {
                continue;
            }

            auto& [h_val, h_idx] = result.value();
            NodePtr successor_node = std::allocate_shared<Node>(
                alloc, outgoing_edge.target, new_g, h_val, node, outgoing_edge.cost, h_idx);
            open.push(successor_node);
        }
    }

    runtime = static_cast<float>(std::clock() - start_time);

    size_t total_pareto = 0;
    num_active_states = 0;
    for (size_t i = 0; i < pareto_list.size(); i++) {
        if (!pareto_list[i].empty()) {
            num_active_states++;
            total_pareto += pareto_list[i].size();
        }
    }
    avg_pareto_size = num_active_states > 0
        ? static_cast<float>(total_pareto) / static_cast<float>(num_active_states)
        : 0.0f;

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
