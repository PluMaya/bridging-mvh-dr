//
// Created by crl on 14/07/2024.
//

#include <fstream>
#include <iostream>
#include <queue>
#include <solvers/boa.h>

BOAStar::BOAStar(const AdjacencyMatrix& adj_matrix)
    : AbstractSolver(adj_matrix, {0, 0}) {
}

void BOAStar::operator()(const size_t& source, const size_t& target,
                         const Heuristic& heuristic, SolutionSet& solutions,
                         const unsigned int time_limit,
                         const std::string& solutions_file,
                         const std::string& stats_file) {
    init_search();
    std::pmr::polymorphic_allocator<Node> alloc{&node_pool};
    // Vector to hold mininum cost of 2nd criteria per node
    std::vector<size_t> min_g2(adj_matrix.size() + 1, MAX_COST);

    // Init open heap
    std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNodeByFValue> open;

    auto initialization_time = std::clock();
    auto node = std::allocate_shared<Node>(alloc, source, std::vector<size_t>(adj_matrix.num_of_objectives, 0),
                                       heuristic(source));
    open.push(node);

    while (!open.empty()) {
        node = open.top();
        open.pop();
        num_generation += 1;
        num_local_dominance_check_dr++;
        num_global_dominance_check++;

        if ((node->f[1]) >= min_g2[target] ||
            (node->g[1] >= min_g2[node->id])) {
            continue;
        }

        min_g2[node->id] = node->g[1];
        num_expansion += 1;

        if (node->id == target) {
            solutions.push_back(node);
            continue;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge>& outgoing_edges = adj_matrix[node->id];

        for (const auto& outgoing_edge : outgoing_edges) {
            const size_t next_id = outgoing_edge.target;
            const size_t next_g1 = node->g[1] + outgoing_edge.cost[1];

            // Local dominance check — scalar only, no allocation.
            num_local_dominance_check_dr++;
            if (next_g1 >= min_g2[next_id]) {
                continue;
            }

            // Heuristic call deferred until local check passes.
            const auto next_h = heuristic(next_id);
            num_global_dominance_check++;
            // Global dominance check.
            if (next_g1 + next_h[1] >= min_g2[target]) {
                continue;
            }
            std::vector<size_t> next_g = {node->g[0] + outgoing_edge.cost[0], next_g1};
            open.push(std::allocate_shared<Node>(alloc, next_id, next_g, next_h, node));
        }
    }

    runtime = static_cast<float>(std::clock() - start_time);

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
            << num_expansion << "\t" << num_generation << "\t"
            << num_local_dominance_check_dr << "\t" << num_global_dominance_check << "\n";
    }
}
