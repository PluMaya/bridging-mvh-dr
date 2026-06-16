//
// Created by crl on 15/07/2024.
//

#ifndef APEX_H
#define APEX_H
#include "abstract_solver.h"
#include "data_structures/apex_path_pair.h"
#include "data_structures/map_queue.h"

#include <cassert>

using ApexSolutionSet = std::vector<ApexPathPairPtr>;

class ApexSearch : public AbstractSolver {
public:
    static bool is_weakly_dominated(const std::vector<size_t>& v1, const std::vector<size_t>& v2) {
        // returns true if v1 is weakly dominated v2
        for (int i = 0; i < v1.size(); i++) {
            if (v1[i] < v2[i]) {
                // it is enough to have one entry that is lower than the other
                return false;
            }
        }
        return true;
    }

    static bool is_strongly_dominated(const std::vector<size_t>& v1, const std::vector<size_t>& v2) {
        // returns true if v1 is weakly dominated v2
        for (int i = 0; i < v1.size(); i++) {
            if (v1[i] < v2[i]) {
                // it is enough to have one entry that is lower than the other
                return false;
            }
        }
        if (v1 == v2) {
            return false;
        }
        return true;
    }

    std::vector<std::vector<ApexPathPairPtr>> expanded;
    ApexPathPairPtr last_solution = nullptr;

    std::string get_solver_name() override { return "A*pex"; }

    void operator()(const size_t& source, const size_t& target, const Heuristic& heuristic,
                    SolutionSet& solutions, unsigned int time_limit) override;

    virtual void insert(ApexPathPairPtr& ap, MapQueue& queue);
    bool is_dominated(const ApexPathPairPtr& ap) const;

    bool local_dominance_check(const ApexPathPairPtr& ap) const {
        for (const auto& exp_ap : expanded[ap->id]) {
            if (is_weakly_dominated(ap->apex->g, exp_ap->apex->g)) {
                return true;
            }
        }
        return false;
    }

    bool global_dominance_check(const ApexPathPairPtr& ap) const;
    void merge_to_solutions(const ApexPathPairPtr& ap,
                            ApexSolutionSet& solutions);
    ApexSearch(const AdjacencyMatrix& adj_matrix, const EPS& eps);

    void init_search() override;
};

#endif // APEX_H
