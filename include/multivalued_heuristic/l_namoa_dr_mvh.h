//
// Created by crl on 28/02/2026.
//

#ifndef L_NAMOA_DR_MVH_H
#define L_NAMOA_DR_MVH_H

#include "data_structures/adjacency_matrix.h"
#include "data_structures/apex_path_pair.h"
#include "definitions.h"
#include "solvers/abstract_solver.h"

#include <chrono>
#include <iostream>
#include <memory_resource>
#include <optional>

class L_NAMOA_DR_MVH : public AbstractSolver {
public:
    std::pmr::unsynchronized_pool_resource node_pool;
    std::vector<std::vector<std::vector<size_t>>> truncated_non_dominated_g;
    std::vector<std::vector<NodePtr>> pareto_list;
    std::unordered_map<std::string, std::chrono::duration<long long, std::ratio<1, 1000000000>>> time_map;

    size_t num_full_dominance_check   = 0;
    size_t num_dr_dominance_check     = 0;
    size_t num_global_dominance_check = 0;
    size_t num_good_fallback          = 0;
    size_t num_bad_fallback           = 0;
    size_t num_g_dominates_existing   = 0;
    size_t num_reinsertion            = 0;
    float avg_truncated_size          = 0.0f;
    float avg_pareto_size             = 0.0f;
    size_t num_active_states          = 0;

    void init_search() override {
        AbstractSolver::init_search();
        num_full_dominance_check   = 0;
        num_dr_dominance_check     = 0;
        num_global_dominance_check = 0;
        num_good_fallback          = 0;
        num_bad_fallback           = 0;
        num_g_dominates_existing   = 0;
        num_reinsertion            = 0;
        avg_truncated_size         = 0.0f;
        avg_pareto_size            = 0.0f;
        num_active_states          = 0;
    }

    std::string get_solver_name() override { return "L_NAMOA_DR_MVH"; }

    void operator()(const size_t& source, const size_t& target,
                    const MultiValuedHeuristic& heuristic,
                    SolutionSet& solutions,
                    unsigned int time_limit = 0,
                    const std::string& solutions_file = "",
                    const std::string& stats_file = "");

    void operator()(const size_t& source, const size_t& target,
                    const Heuristic& heuristic, SolutionSet& solutions,
                    unsigned time_limit) override {
        std::cout << "not a real func " << std::endl;
    };

    L_NAMOA_DR_MVH(const AdjacencyMatrix& adj_matrix, const EPS& eps);

    [[nodiscard]] bool naive_local_dominance_check(const NodePtr& node_ptr);

    [[nodiscard]] bool better_local_dominance_check(const std::vector<size_t>& g, size_t id);

    bool global_dominance_check(const NodePtr& node_ptr,
                                const size_t& target_id);

    std::optional<std::vector<size_t>> naive_get_first_undominated_heuristic_value(
        const std::vector<size_t>& g_value,
        const std::vector<std::vector<size_t>>& node_mvh, const std::vector<size_t>& parent_h,
        const std::vector<size_t>& edge_cost, const
        size_t& global_target);

    std::optional<std::pair<std::vector<size_t>, size_t>> get_first_undominated_heuristic_value(
        const std::vector<size_t>& g_value, const size_t& target,
        const std::vector<std::vector<size_t>>& node_mvh,
        size_t start_idx = 0);
};


#endif //L_NAMOA_DR_MVH_H
