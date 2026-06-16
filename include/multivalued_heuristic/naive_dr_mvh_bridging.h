#ifndef NAIVE_DR_MVH_BRIDGING_H
#define NAIVE_DR_MVH_BRIDGING_H

#include "data_structures/adjacency_matrix.h"
#include "data_structures/apex_path_pair.h"
#include "definitions.h"
#include "solvers/abstract_solver.h"

#include <iostream>
#include <memory_resource>
#include <optional>

// Naive attempt to combine NAMOA_DR-style dimensionality-reduction dominance
// checks with a multi-valued heuristic.
class NAIVE_DR_MVH_BRIDGING : public AbstractSolver {
public:
    std::pmr::unsynchronized_pool_resource node_pool;
    std::vector<std::vector<std::vector<size_t>>> truncated_non_dominated_g;

    size_t num_full_dominance_check   = 0;
    size_t num_dr_dominance_check     = 0;
    size_t num_global_dominance_check = 0;
    size_t num_good_fallback          = 0;
    size_t num_bad_fallback           = 0;
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
        num_reinsertion            = 0;
        avg_truncated_size         = 0.0f;
        avg_pareto_size            = 0.0f;
        num_active_states          = 0;
    }

    std::string get_solver_name() override { return "NAIVE_DR_MVH_BRIDGING"; }

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

    NAIVE_DR_MVH_BRIDGING(const AdjacencyMatrix& adj_matrix, const EPS& eps);

    [[nodiscard]] bool dr_local_dominance_check(const std::vector<size_t>& g, size_t id);

    bool global_dominance_check(const NodePtr& node_ptr,
                                const size_t& target_id);

    std::optional<std::pair<std::vector<size_t>, size_t>> get_first_undominated_heuristic_value(
        const std::vector<size_t>& g_value, const size_t& target,
        const std::vector<std::vector<size_t>>& node_mvh,
        size_t start_idx = 0);
};

#endif //NAIVE_DR_MVH_BRIDGING_H
