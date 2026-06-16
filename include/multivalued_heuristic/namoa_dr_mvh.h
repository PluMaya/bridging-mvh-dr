//
// Created by crl on 14/02/2026.
//

#ifndef NAMOA_DR_MVH_H
#define NAMOA_DR_MVH_H


#include "data_structures/adjacency_matrix.h"
#include "data_structures/apex_path_pair.h"
#include "definitions.h"
#include "solvers/abstract_solver.h"
#include <iostream>
#include <memory_resource>


class NAMOA_DR_MVH : public AbstractSolver {
public:
  std::pmr::unsynchronized_pool_resource node_pool;
  // closed[node_id][h_idx] = DR Pareto front for the (node_id, h_idx) state.
  std::vector<std::vector<std::vector<std::vector<size_t>>>> closed;
  float avg_closed_size = 0.0f;
  size_t unique_gen = 0;
  size_t num_active_states = 0;
  std::string get_solver_name() override { return "NAMOA_DR_MVH"; }

  void operator()(const size_t &source, const size_t &target,
                  const MultiValuedHeuristic &heuristic,
                  SolutionSet &solutions,
                  unsigned int time_limit = 0,
                  const std::string& solutions_file = "",
                  const std::string& stats_file = "");

  void operator()(const size_t &source, const size_t &target,
                  const Heuristic &heuristic, SolutionSet &solutions,
                  unsigned time_limit) override {
    std::cout << "not a real func " << std::endl;
  };

  NAMOA_DR_MVH(const AdjacencyMatrix &adj_matrix, const EPS &eps);

};

#endif //NAMOA_DR_MVH_H
