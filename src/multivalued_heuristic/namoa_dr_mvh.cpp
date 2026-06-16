#include "multivalued_heuristic/namoa_dr_mvh.h"

#include <algorithm>
#include <fstream>
#include <queue>

NAMOA_DR_MVH::NAMOA_DR_MVH(const AdjacencyMatrix& adj_matrix, const EPS& eps)
    : AbstractSolver(adj_matrix, eps) {
}

// ── DR dominance helpers (same logic as namoa_dr.cpp) ────────────────────────

static bool is_dominated_dr(const std::vector<size_t>& value1, const std::vector<size_t>& value2) {
    for (size_t i = 1; i < value1.size(); ++i) {
        if (value1[i] < value2[i])
            return false;
    }
    return true;
}

static bool local_dominance_check(const NodePtr& node_ptr, std::vector<std::vector<size_t>> &truncated_non_dominated_g) {
    for (const auto& truncated_g : truncated_non_dominated_g) {
        if (is_dominated_dr(node_ptr->g, truncated_g)) {
            return true;
        }
    }
    return false;
}

static bool global_dominance_check(const NodePtr& node_ptr, std::vector<std::vector<size_t>> &truncated_target_g) {
    // Check if node is dominated by comparing g values with truncated vectors
    for (const auto& truncated_g : truncated_target_g) {
        if (is_dominated_dr(node_ptr->f, truncated_g)) {
            return true;
        }
    }
    return false;
}
// Adds `node` to `front`, pruning any entries it dominates.
static void add_node_dr(const NodePtr& node, std::vector<std::vector<size_t>>& truncated_list) {

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
}

// ── Main search ───────────────────────────────────────────────────────────────

void NAMOA_DR_MVH::operator()(
    const size_t& source, const size_t& target,
    const MultiValuedHeuristic& heuristic,
    SolutionSet& solutions,
    unsigned int time_limit,
    const std::string& solutions_file,
    const std::string& stats_file)
{
    init_search();

    const size_t V = adj_matrix.size() + 1;
    const size_t K = adj_matrix.num_of_objectives;

    // closed[node_id][h_idx] = DR Pareto front for state (node_id, h_idx).
    closed.assign(V, {});
    for (size_t v = 0; v < V; ++v)
        closed[v].resize(heuristic[v].size());

    // Flat Pareto front accumulated from solutions found at target; used for
    // global pruning across all heuristic indices.
    std::vector<NodePtr> target_pareto;

    std::pmr::polymorphic_allocator<Node> alloc{&node_pool};
    std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNodeByFValue> open_list;

    auto initialization_time = std::clock();

    for (size_t i = 0; i < heuristic[source].size(); ++i) {
        open_list.push(std::allocate_shared<Node>(
            alloc,
            source,
            std::vector<size_t>(K, 0),
            heuristic[source][i],
            nullptr,
            std::vector<size_t>(K, 0),
            i));
    }

    while (!open_list.empty()) {
        if (time_limit > 0 && (std::clock() - start_time) / CLOCKS_PER_SEC >= time_limit) {
            time_limit_reached = true;
            break;
        }

        auto node = open_list.top();
        open_list.pop();
        ++num_generation;

        // Local dominance: has a non-dominated node already been expanded for
        // this (node_id, h_idx) state?
        if (local_dominance_check(node, closed[node->id][node->h_idx]))
            continue;

        // Global dominance: is this node's f dominated by a known solution?
        if (global_dominance_check(node, closed[target][0]))
            continue;

        add_node_dr(node, closed[node->id][node->h_idx]);
        ++num_expansion;

        if (node->id == target) {
            solutions.push_back(node);
            add_node_dr(node, closed[target][0]);
            target_pareto.push_back(node);
            continue;
        }

        for (const auto& edge : adj_matrix[node->id]) {
            std::vector<size_t> new_g(K);
            for (size_t k = 0; k < K; ++k)
                new_g[k] = node->g[k] + edge.cost[k];

            ++unique_gen;

            const auto& neighbor_heuristics = heuristic[edge.target];
            for (size_t i = 0; i < neighbor_heuristics.size(); ++i) {
                const auto& hval = neighbor_heuristics[i];

                // Heuristic compatibility: parent.h[k] <= child.h[k] + edge.cost[k]
                // for all k (generalised triangle inequality).
                bool compatible = true;
                for (size_t k = 0; k < K; ++k) {
                    if (node->h[k] > hval[k] + edge.cost[k]) {
                        compatible = false;
                        break;
                    }
                }
                if (!compatible) continue;

                auto successor = std::allocate_shared<Node>(
                    alloc, edge.target, new_g, hval, node, edge.cost, i);

                // Cheap pre-allocation dominance checks.
                if (local_dominance_check(successor, closed[edge.target][i]))
                    continue;
                if (global_dominance_check(successor, closed[target][0]))
                    continue;

                open_list.push(successor);
            }
        }
    }

    runtime = static_cast<float>(std::clock() - start_time);

    // Compute average size of closed entries per (state, h-value) over active states only
    size_t total_entries = 0;
    num_active_states = 0;
    for (size_t v = 0; v < V; ++v) {
        for (size_t h = 0; h < closed[v].size(); ++h) {
            if (closed[v][h].size() > 0) {
                num_active_states++;
                total_entries += closed[v][h].size();
            }
        }
    }
    avg_closed_size = num_active_states > 0
        ? static_cast<float>(total_entries) / static_cast<float>(num_active_states)
        : 0.0f;

    std::ofstream sol_out(solutions_file);
    for (const auto& sol : solutions) {
        for (size_t k = 0; k < sol->g.size(); ++k) {
            if (k > 0) sol_out << ',';
            sol_out << sol->g[k];
        }
        sol_out << '\n';
    }

    auto initialization_runtime = initialization_time - start_time;
    std::ofstream stats_out(stats_file);
    stats_out << initialization_runtime / CLOCKS_PER_SEC << "\t"
              << runtime / CLOCKS_PER_SEC << "\t"
              << num_expansion << "\t" << num_generation << '\n'
              << num_generation << "\t" << num_expansion << '\n';
}
