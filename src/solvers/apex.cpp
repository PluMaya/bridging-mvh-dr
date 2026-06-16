//
// Created by crl on 15/07/2024.
//

#include "solvers/apex.h"

#include <iostream>

ApexSearch::ApexSearch(const AdjacencyMatrix& adj_matrix, const EPS& eps)
    : AbstractSolver(adj_matrix, eps) {
    expanded.resize(adj_matrix.size() + 1);
}

void ApexSearch::insert(ApexPathPairPtr& ap, MapQueue& queue) {
    std::vector<ApexPathPairPtr>& relevant_aps = queue.get_open(ap->id);
    for (size_t i = 0; i < relevant_aps.size(); ) {
        if (!relevant_aps[i]->is_active) {
            relevant_aps[i] = std::move(relevant_aps.back());
            relevant_aps.pop_back();
            continue;
        }
        if (ap->update_nodes_by_merge_if_bounded(relevant_aps[i], eps)) {
            if ((ap->apex != relevant_aps[i]->apex) ||
                (ap->path_node != relevant_aps[i]->path_node)) {
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

void ApexSearch::merge_to_solutions(const ApexPathPairPtr& ap,
                                    ApexSolutionSet& solutions) {
    for (auto& solution : solutions) {
        if (solution->update_nodes_by_merge_if_bounded(ap, eps)) {
            return;
        }
    }
    solutions.push_back(ap);
    last_solution = ap;
}

bool ApexSearch::global_dominance_check(const ApexPathPairPtr& ap) const {
    if (last_solution == nullptr) {
        return false;
    }
    if (is_bounded(ap->apex, last_solution->path_node, eps)) {
        last_solution->update_apex_by_merge_if_bounded(ap->apex, eps);
        return true;
    }
    return false;
}

bool ApexSearch::is_dominated(const ApexPathPairPtr& ap) const {
    if (local_dominance_check(ap)) {
        return true;
    }
    return global_dominance_check(ap);
}

void ApexSearch::init_search() {
    AbstractSolver::init_search();
    expanded.clear();
    expanded.resize(adj_matrix.size() + 1);
    last_solution = nullptr;
}

void ApexSearch::operator()(const size_t& source, const size_t& target,
                            const Heuristic& heuristic, SolutionSet& solutions,
                            unsigned int time_limit) {
    init_search();
    start_time = std::clock();

    ApexSolutionSet ap_solutions;

    MapQueue open(adj_matrix.size() + 1);

    NodePtr source_node = std::make_shared<Node>(source, std::vector<size_t>(adj_matrix.num_of_objectives, 0),
                                                 heuristic(source));
    ApexPathPairPtr ap =
        std::make_shared<ApexPathPair>(source_node, source_node, heuristic(source));
    open.insert(ap);

    while (!open.empty()) {
        ap = open.pop();
        num_generation += 1;

        if (ap->is_active == false) {
            continue;
        }

        if (is_dominated(ap)) {
            continue;
        }

        num_expansion += 1;

        expanded[ap->id].push_back(ap);

        if (ap->id == target) {
            merge_to_solutions(ap, ap_solutions);
            continue;
        }

        const std::vector<Edge>& outgoing_edges = adj_matrix[ap->id];
        for (const auto& outgoing_edge : outgoing_edges) {
            ApexPathPairPtr next_ap =
                std::make_shared<ApexPathPair>(ap, outgoing_edge, heuristic(outgoing_edge.target));

            if (is_dominated(next_ap)) {
                continue;
            }

            this->insert(next_ap, open);
        }
    }

    for (auto& ap_solution : ap_solutions) {
        solutions.push_back(ap_solution->path_node);
    }
    runtime = static_cast<float>(std::clock() - start_time);
}
