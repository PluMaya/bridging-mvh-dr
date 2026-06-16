#include "data_structures/adjacency_matrix.h"
#include "definitions.h"
#include "multivalued_heuristic/apex_mvh.h"
#include "multivalued_heuristic/l_namoa_dr_mvh.h"
#include "multivalued_heuristic/namoa.h"
#include "multivalued_heuristic/namoa_dr_mvh.h"
#include "parser.h"
#include "solvers/namoa_dr.h"
#include "solvers/shortest_path_heuristic_computer.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

using CostVectorSet = std::set<std::vector<size_t>>;

struct QueryCase {
    std::string name;
    std::vector<size_t> objectives;
    size_t source;
    size_t target;
};

struct RunResult {
    std::string name;
    CostVectorSet costs;
    size_t expansions = 0;
    size_t generations = 0;
    bool time_limit_reached = false;
};

Heuristic shortest_path_heuristic(const AdjacencyMatrix& graph, size_t target) {
    auto heuristic = std::make_shared<ShortestPathHeuristic>(target, graph.graph_size, graph);
    return [heuristic](size_t node_id) {
        return (*heuristic)(node_id);
    };
}

CostVectorSet solution_costs(const SolutionSet& solutions) {
    CostVectorSet costs;
    for (const auto& solution : solutions) {
        costs.insert(solution->g);
    }
    return costs;
}

std::string format_objectives(const std::vector<size_t>& objectives) {
    std::ostringstream out;
    out << "(";
    for (size_t i = 0; i < objectives.size(); ++i) {
        if (i > 0) {
            out << ", ";
        }
        out << objectives[i];
    }
    out << ")";
    return out.str();
}

std::string format_cost_vector(const std::vector<size_t>& cost) {
    std::ostringstream out;
    out << "(";
    for (size_t i = 0; i < cost.size(); ++i) {
        if (i > 0) {
            out << ", ";
        }
        out << cost[i];
    }
    out << ")";
    return out.str();
}

std::string format_cost_set(const CostVectorSet& costs) {
    std::ostringstream out;
    bool first = true;
    for (const auto& cost : costs) {
        if (!first) {
            out << " ";
        }
        first = false;
        out << format_cost_vector(cost);
    }
    return out.str();
}

void print_set_difference(const CostVectorSet& expected,
                          const CostVectorSet& actual,
                          const std::string& label) {
    CostVectorSet diff;
    std::set_difference(expected.begin(), expected.end(),
                        actual.begin(), actual.end(),
                        std::inserter(diff, diff.begin()));
    if (!diff.empty()) {
        std::cout << "    " << label << ": " << format_cost_set(diff) << "\n";
    }
}

RunResult run_namoa_dr_reference(const AdjacencyMatrix& graph,
                                 const QueryCase& query) {
    NAMOAdr solver(graph);
    SolutionSet solutions;
    solver(query.source, query.target,
           shortest_path_heuristic(graph, query.target), solutions, 0, "", "");
    return {"NAMOA_DR + shortest-path heuristic", solution_costs(solutions),
            solver.num_expansion, solver.num_generation, solver.time_limit_reached};
}

RunResult run_l_namoa_dr_mvh(const AdjacencyMatrix& graph,
                      const QueryCase& query,
                      const EPS& eps,
                      const MultiValuedHeuristic& mvh) {
    L_NAMOA_DR_MVH solver(graph, eps);
    SolutionSet solutions;
    solver(query.source, query.target, mvh, solutions, 0, "", "");
    return {"L_NAMOA_DR_MVH + APEX_MVH", solution_costs(solutions),
            solver.num_expansion, solver.num_generation, solver.time_limit_reached};
}

RunResult run_namoa_dr_mvh(const AdjacencyMatrix& graph,
                       const QueryCase& query,
                       const EPS& eps,
                       const MultiValuedHeuristic& mvh) {
    NAMOA_DR_MVH solver(graph, eps);
    SolutionSet solutions;
    solver(query.source, query.target, mvh, solutions, 0, "", "");
    return {"NAMOA_DR_MVH + APEX_MVH", solution_costs(solutions),
            solver.num_expansion, solver.num_generation, solver.time_limit_reached};
}

RunResult run_namoa(const AdjacencyMatrix& graph,
                        const QueryCase& query,
                        const EPS& eps,
                        const MultiValuedHeuristic& mvh) {
    NAMOA solver(graph, eps);
    SolutionSet solutions;
    solver(query.source, query.target, mvh, solutions, 0, "", "");
    return {"NAMOA + APEX_MVH", solution_costs(solutions),
            solver.num_expansion, solver.num_generation, solver.time_limit_reached};
}

bool compare_to_reference(const QueryCase& query,
                          const RunResult& reference,
                          const RunResult& candidate) {
    const bool matches = reference.costs == candidate.costs && !candidate.time_limit_reached;
    if (matches) {
        std::cout << "  OK  " << candidate.name
                  << " solutions=" << candidate.costs.size()
                  << " expansions=" << candidate.expansions
                  << " generations=" << candidate.generations << "\n";
        return true;
    }

    std::cout << "  FAIL " << candidate.name << "\n"
              << "    case=" << query.name << " source=" << query.source
              << " target=" << query.target << "\n"
              << "    NAMOA_DR solutions=" << reference.costs.size()
              << " candidate solutions=" << candidate.costs.size()
              << " candidate_time_limit_reached=" << candidate.time_limit_reached << "\n";
    print_set_difference(reference.costs, candidate.costs, "missing vs NAMOA_DR");
    print_set_difference(candidate.costs, reference.costs, "extra vs NAMOA_DR");
    return false;
}

} // namespace

int main(int argc, char** argv) {
    const std::string synthetic_graph_dir = argc > 1 ? argv[1] : "resources/synthetic_graph";

    const std::vector<QueryCase> cases = {
        {"synthetic_graph_3obj_distance_difficulty_time_1_to_20", {0, 1, 4}, 1, 20},
        {"synthetic_graph_4obj_no_length_10_to_77", {0, 1, 3, 4}, 10, 77},
        {"synthetic_graph_5obj_all_43_to_96", {0, 1, 2, 3, 4}, 43, 96},
    };

    bool all_ok = true;
    for (const auto& query : cases) {
        std::cout << "\n=== " << query.name
                  << " objectives=" << format_objectives(query.objectives) << " ===\n";

        auto graph = Parser(synthetic_graph_dir, query.objectives).default_graph();
        const EPS exact_eps(graph.num_of_objectives, 0.0);

        const RunResult reference = run_namoa_dr_reference(graph, query);
        std::cout << "  REF NAMOA_DR solutions=" << reference.costs.size()
                  << " costs=" << format_cost_set(reference.costs) << "\n";

        APEX_MVH apex_mvh_builder(graph, exact_eps);
        MultiValuedHeuristic apex_mvh = apex_mvh_builder(query.target, "");
        std::cout << "  APEX_MVH num_generation=" << apex_mvh_builder.num_generation
                  << " expansions=" << apex_mvh_builder.num_expansion << "\n";

        const std::vector<RunResult> candidates = {
            run_l_namoa_dr_mvh(graph, query, exact_eps, apex_mvh),
            run_namoa_dr_mvh(graph, query, exact_eps, apex_mvh),
            run_namoa(graph, query, exact_eps, apex_mvh),
        };

        for (const auto& candidate : candidates) {
            all_ok = compare_to_reference(query, reference, candidate) && all_ok;
        }
    }

    if (!all_ok) {
        std::cout << "\nOne or more multi-objective synthetic graph APEX_MVH checks failed.\n";
        return 1;
    }

    std::cout << "\nAll multi-objective synthetic graph APEX_MVH checks passed.\n";
    return 0;
}
