//
// Created by crl on 28/09/2024.
//

#ifndef EXPERIMENT_UTILS_H
#define EXPERIMENT_UTILS_H
#include "data_structures/adjacency_matrix.h"
#include "definitions.h"
#include "solvers/abstract_solver.h"


class 
ExperimentUtils {
public:
    static void run_namoa_dr(const AdjacencyMatrix& adjecency_matrix,
                              const size_t& source, const size_t& target,
                              const Heuristic& heuristic,
                              const std::string& logging_file,
                              unsigned int time_limit = 0);

    static void run_boa_star(const AdjacencyMatrix& adjecency_matrix,
                             const size_t& source, const size_t& target,
                             const Heuristic& heuristic,
                             const std::string& logging_file);

    static void run_apex(const AdjacencyMatrix& adjecency_matrix,
                         const size_t& source, const size_t& target,
                         const EPS& eps, const Heuristic& heuristic, long heuristic_duration,
                         const std::string& logging_file);

    static void run_apex_mvh(const AdjacencyMatrix& adjecency_matrix,
                                    const size_t& target,
                                    const EPS& eps,
                                    const std::string& logging_file);

    static void run_namoa_backward_search(const AdjacencyMatrix& adjecency_matrix,
        const size_t& target,
        const std::string& logging_file);

    static void run_namoa_dr_mvh(const AdjacencyMatrix& adjecency_matrix,
                             const size_t& source, const size_t& target,
                             const EPS& eps,
                             const MultiValuedHeuristic& mvh,
                             const std::string& logging_file,
                             unsigned int time_limit = 0);

    static void run_l_namoa_dr_mvh(const AdjacencyMatrix& adjecency_matrix,
                               const size_t& source, const size_t& target,
                               const EPS& eps,
                               const MultiValuedHeuristic& mvh,
                               const std::string& logging_file,
                               unsigned int time_limit = 0);

    static void run_naive_dr_mvh_bridging(const AdjacencyMatrix& adjecency_matrix,
                                  const size_t& source, const size_t& target,
                                  const EPS& eps,
                                  const MultiValuedHeuristic& mvh,
                                  const std::string& logging_file,
                                  unsigned int time_limit = 0);

    static void run_namoa(const AdjacencyMatrix& adjecency_matrix,
                              const size_t& source, const size_t& target,
                              const EPS& eps,
                              const MultiValuedHeuristic& mvh,
                              const std::string& logging_file,
                              unsigned int time_limit = 0);

    static void single_run(AdjacencyMatrix& adjecency_matrix, size_t source,
                           size_t target, const std::string& algorithm, const EPS& eps,
                           std::vector<int> multi_sources, const Heuristic& svh_heuristic = nullptr,
                           const MultiValuedHeuristic& mvh = {},
                           const std::string& logging_file = "",
                           unsigned int time_limit = 0);
};

#endif // EXPERIMENT_UTILS_H
