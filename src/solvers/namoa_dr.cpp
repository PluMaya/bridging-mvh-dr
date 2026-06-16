#include <fstream>
#include <iostream>
#include <queue>
#include <solvers/namoa_dr.h>

NAMOAdr::NAMOAdr(const AdjacencyMatrix& adj_matrix)
    : AbstractSolver(adj_matrix, {}) {
}

bool is_dominated_dr(const NodePtr& node, const NodePtr& other_node) {
    // returns true if node is weakly dominated by other_node
    for (int i = 1; i < node->f.size(); i++) {
        if (node->f[i] < other_node->f[i]) {  // it is enough to have one entry that is lower than the other node
            return false;
        }
    }
    return true;
}


bool is_dominated_dr(const NodePtr& node, const std::vector<NodePtr>& vec) {
    for (const auto& n : vec) {
        if (is_dominated_dr(node, n)) {
            return true;
        }
    }
    return false;
}


void add_node_dr(const NodePtr& node, std::vector<NodePtr>& vec) {
    vec.erase(
        std::remove_if(vec.begin(), vec.end(),
            [&node](const NodePtr& n) { return is_dominated_dr(n, node); }),
        vec.end());
    vec.push_back(node);
}


void NAMOAdr::operator()(const size_t& source, const size_t& target,
                         const Heuristic& heuristic, SolutionSet& solutions,
                         const unsigned int time_limit,
                         const std::string& solutions_file,
                         const std::string& stats_file) {
    init_search();
    std::pmr::polymorphic_allocator<Node> alloc{&node_pool};
    std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNodeByFValue> open;
    std::vector<std::vector<NodePtr>> closed(this->adj_matrix.size() + 1);
    // Init open heap
    auto node = std::allocate_shared<Node>(alloc, source, std::vector<size_t>(adj_matrix.num_of_objectives, 0),
                                           heuristic(source));
    open.push(node);

    while (!open.empty()) {
        if (time_limit > 0 && (std::clock() - start_time) / CLOCKS_PER_SEC >= time_limit) {
            time_limit_reached = true;
            break;
        }
        // Pop min from queue and process
        node = open.top();
        open.pop();
        num_generation += 1;

        // Dominance check
        num_local_dominance_check_dr++;
        if (is_dominated_dr(node, closed[node->id])) {
            continue;
        }
        num_global_dominance_check++;
        if (is_dominated_dr(node, closed[target])) {
            continue;
        }

        add_node_dr(node, closed[node->id]);
        num_expansion += 1;

        if (node->id == target) {
            solutions.push_back(node);
            continue;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge>& outgoing_edges = adj_matrix[node->id];

        for (const auto& outgoing_edge : outgoing_edges) {
            size_t next_id = outgoing_edge.target;
            std::vector<size_t> next_g(node->g.size());
            for (size_t i = 0; i < next_g.size(); i++) {
                next_g[i] = node->g[i] + outgoing_edge.cost[i];
            }
            auto next_h = heuristic(next_id);
            auto next = std::allocate_shared<Node>(alloc, next_id, next_g, next_h, node);

            num_local_dominance_check_dr++;
            if (is_dominated_dr(next, closed[next_id])) {
                continue;
            }
            num_global_dominance_check++;
            if (is_dominated_dr(next, closed[target])) {
                continue;
            }
            open.push(next);
        }
    }

    runtime = static_cast<float>(std::clock() - start_time);

    // Compute average size of closed lists over active states only
    {
        size_t total_closed_entries = 0;
        num_active_states = 0;
        size_t num_nodes = closed.size();
        for (size_t i = 0; i < num_nodes; i++) {
            if (closed[i].size() > 0) {
                num_active_states++;
                total_closed_entries += closed[i].size();
            }
        }
        avg_closed_size = num_active_states > 0
            ? static_cast<float>(total_closed_entries) / static_cast<float>(num_active_states)
            : 0.0f;
    }

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
