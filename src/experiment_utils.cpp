//
// Created by crl on 28/09/2024.
//

#include "experiment_utils.h"

#include "multivalued_heuristic/apex_mvh.h"
#include "multivalued_heuristic/namoa_dr_mvh.h"
#include "solvers/apex.h"
#include "solvers/boa.h"
#include "solvers/namoa_dr.h"
#include "solvers/shortest_path_heuristic_computer.h"

#include <fstream>
#include <iostream>

#include "multivalued_heuristic/naive_dr_mvh_bridging.h"
#include "multivalued_heuristic/l_namoa_dr_mvh.h"
#include "multivalued_heuristic/namoa.h"
#include "multivalued_heuristic/closure.h"
#include "multivalued_heuristic/namoa_backward_search.h"
using std::placeholders::_1;

void ExperimentUtils::run_namoa_dr(const AdjacencyMatrix& adjecency_matrix,
                                   const size_t& source, const size_t& target,
                                   const Heuristic& heuristic,
                                   const std::string& logging_file,
                                   unsigned int time_limit) {
    auto namoa_dr = NAMOAdr(adjecency_matrix);
    SolutionSet solutions;
    namoa_dr(source, target, heuristic, solutions, time_limit, "", "");
    std::cout << "algorithm=NAMOA_DR"
        << "\tsource=" << source << "\ttarget=" << target
        << "\tnum_solutions=" << solutions.size()
        << "\tnum_expansion=" << namoa_dr.num_expansion
        << "\tnum_generation=" << namoa_dr.num_generation
        << "\tnum_dr_dominance_check=" << namoa_dr.num_local_dominance_check_dr
        << "\tnum_global_dominance_check=" << namoa_dr.num_global_dominance_check
        << "\truntime_s=" << namoa_dr.runtime / CLOCKS_PER_SEC
        << "\ttime_limit_reached=" << namoa_dr.time_limit_reached
        << "\tavg_closed_size=" << namoa_dr.avg_closed_size
        << "\tnum_active_states=" << namoa_dr.num_active_states << std::endl;
}

void ExperimentUtils::run_boa_star(const AdjacencyMatrix& adjecency_matrix,
                                   const size_t& source, const size_t& target,
                                   const Heuristic& heuristic,
                                   const std::string& logging_file) {
    auto boa = BOAStar(adjecency_matrix);
    SolutionSet solutions;
    std::string sol_file = logging_file.empty() ? "" : logging_file + "_" + std::to_string(source) + "_solutions.txt";
    std::string stat_file = logging_file.empty() ? "" : logging_file + "_" + std::to_string(source) + "_stats.txt";
    boa(source, target, heuristic, solutions, 30, sol_file, stat_file);
    std::cout << "algorithm=BOA"
        << "\tsource=" << source << "\ttarget=" << target
        << "\tnum_solutions=" << solutions.size()
        << "\tnum_expansion=" << boa.num_expansion
        << "\tnum_generation=" << boa.num_generation
        << "\truntime_s=" << boa.runtime / CLOCKS_PER_SEC << std::endl;
}

void ExperimentUtils::run_apex(const AdjacencyMatrix& adjecency_matrix,
                               const size_t& source, const size_t& target,
                               const EPS& eps, const Heuristic& heuristic, const long heuristic_duration,
                               const std::string& logging_file) {
    auto apex = ApexSearch(adjecency_matrix, eps);
    SolutionSet solutions;
    apex(source, target, heuristic, solutions, 300);
    std::cout << "algorithm=Apex"
        << "\tsource=" << source << "\ttarget=" << target
        << "\tnum_solutions=" << solutions.size()
        << "\tnum_expansion=" << apex.num_expansion
        << "\tnum_generation=" << apex.num_generation
        << "\truntime_s=" << apex.runtime
        << "\theuristic_duration_s=" << heuristic_duration << std::endl;
}

void ExperimentUtils::run_apex_mvh(
    const AdjacencyMatrix& adjecency_matrix,
    const size_t& target, const EPS& eps,
    const std::string& logging_file) {
    auto apex_mvh_builder = APEX_MVH(adjecency_matrix, eps);
    MultiValuedHeuristic mvh =
        apex_mvh_builder(target, logging_file + "_apex_mvh.txt");
    float average_mvh_size = 0.0f;
    for (const auto& entry : mvh) {
        average_mvh_size += entry.size();
    }
    average_mvh_size /= mvh.size();
    
    std::cout << "algorithm=APEX_MVH"
        << "\ttarget=" << target
        << "\tavg_mvh_size=" << average_mvh_size
        << "\tnum_expansion=" << apex_mvh_builder.num_expansion
        << "\tnum_generation=" << apex_mvh_builder.num_generation
        << "\truntime_s=" << apex_mvh_builder.runtime / CLOCKS_PER_SEC << std::endl;
}

void ExperimentUtils::run_namoa_backward_search(const AdjacencyMatrix& adjecency_matrix,
    const size_t& target,
    const std::string& logging_file) {
    auto namoa_backward_search = NAMOABackwardSearch(adjecency_matrix);
    MultiValuedHeuristic mvh = namoa_backward_search(target,
                                                     logging_file + "_namoa_mvh.txt");
    float average_mvh_size = 0.0f;
    for (const auto& entry : mvh) {
        average_mvh_size += entry.size();
    }
    average_mvh_size /= mvh.size();
    
    std::cout << "algorithm=NAMOA-MVH"
        << "\ttarget=" << target
        << "\tavg_mvh_size=" << average_mvh_size
        << "\tnum_expansion=" << namoa_backward_search.num_expansion
        << "\tnum_generation=" << namoa_backward_search.num_generation
        << "\truntime_s=" << namoa_backward_search.runtime / CLOCKS_PER_SEC << std::endl;
}

void ExperimentUtils::run_namoa_dr_mvh(
    const AdjacencyMatrix& adjecency_matrix, const size_t& source,
    const size_t& target, const EPS& eps, const MultiValuedHeuristic& mvh,
    const std::string& logging_file, unsigned int time_limit) {
    auto forward_search = NAMOA_DR_MVH(adjecency_matrix, eps);
    SolutionSet solutions;
    std::string sol_file = logging_file.empty() ? "" : logging_file + "_" + std::to_string(source) + "_solutions.txt";
    std::string stat_file = logging_file.empty() ? "" : logging_file + "_" + std::to_string(source) + "_stats.txt";
    forward_search(source, target, mvh, solutions, time_limit, sol_file, stat_file);
    float avg_dup = forward_search.unique_gen > 0
        ? static_cast<float>(forward_search.num_generation) / static_cast<float>(forward_search.unique_gen)
        : 0.0f;
    std::cout << "algorithm=NAMOA_DR_MVH"
        << "\tsource=" << source << "\ttarget=" << target
        << "\tnum_solutions=" << solutions.size()
        << "\tnum_expansion=" << forward_search.num_expansion
        << "\tnum_generation=" << forward_search.num_generation
        << "\truntime_s=" << forward_search.runtime / CLOCKS_PER_SEC
        << "\ttime_limit_reached=" << forward_search.time_limit_reached
        << "\tavg_closed_size=" << forward_search.avg_closed_size
        << "\tavg_dup_instances=" << avg_dup
        << "\tnum_active_states=" << forward_search.num_active_states << std::endl;
}

void ExperimentUtils::run_l_namoa_dr_mvh(
    const AdjacencyMatrix& adjecency_matrix, const size_t& source,
    const size_t& target, const EPS& eps, const MultiValuedHeuristic& mvh,
    const std::string& logging_file, unsigned int time_limit) {
    auto cfs = L_NAMOA_DR_MVH(adjecency_matrix, eps);
    SolutionSet solutions;
    cfs(source, target, mvh, solutions, time_limit, "", "");
    std::cout << "algorithm=L_NAMOA_DR_MVH"
        << "\tsource=" << source << "\ttarget=" << target
        << "\tnum_solutions=" << solutions.size()
        << "\tnum_expansion=" << cfs.num_expansion
        << "\tnum_generation=" << cfs.num_generation
        << "\tnum_full_dominance_check=" << cfs.num_full_dominance_check
        << "\tnum_dr_dominance_check=" << cfs.num_dr_dominance_check
        << "\tnum_global_dominance_check=" << cfs.num_global_dominance_check
        << "\truntime_s=" << cfs.runtime / CLOCKS_PER_SEC
        << "\ttime_limit_reached=" << cfs.time_limit_reached
        << "\tnum_good_fallback=" << cfs.num_good_fallback
        << "\tnum_bad_fallback=" << cfs.num_bad_fallback
        << "\tnum_g_dominates_existing=" << cfs.num_g_dominates_existing
        << "\tavg_truncated_size=" << cfs.avg_truncated_size
        << "\tavg_pareto_size=" << cfs.avg_pareto_size
        << "\tnum_reinsertion=" << cfs.num_reinsertion
        << "\tnum_active_states=" << cfs.num_active_states << std::endl;
}

void ExperimentUtils::run_naive_dr_mvh_bridging(
    const AdjacencyMatrix& adjecency_matrix, const size_t& source,
    const size_t& target, const EPS& eps, const MultiValuedHeuristic& mvh,
    const std::string& logging_file, unsigned int time_limit) {
    auto cfs = NAIVE_DR_MVH_BRIDGING(adjecency_matrix, eps);
    SolutionSet solutions;
    cfs(source, target, mvh, solutions, time_limit, "", "");
    std::cout << "algorithm=NAIVE_DR_MVH_BRIDGING"
        << "\tsource=" << source << "\ttarget=" << target
        << "\tnum_solutions=" << solutions.size()
        << "\tnum_expansion=" << cfs.num_expansion
        << "\tnum_generation=" << cfs.num_generation
        << "\tnum_full_dominance_check=" << cfs.num_full_dominance_check
        << "\tnum_dr_dominance_check=" << cfs.num_dr_dominance_check
        << "\tnum_global_dominance_check=" << cfs.num_global_dominance_check
        << "\truntime_s=" << cfs.runtime / CLOCKS_PER_SEC
        << "\ttime_limit_reached=" << cfs.time_limit_reached
        << "\tnum_good_fallback=" << cfs.num_good_fallback
        << "\tnum_bad_fallback=" << cfs.num_bad_fallback
        << "\tavg_truncated_size=" << cfs.avg_truncated_size
        << "\tavg_pareto_size=" << cfs.avg_pareto_size
        << "\tnum_reinsertion=" << cfs.num_reinsertion
        << "\tnum_active_states=" << cfs.num_active_states << std::endl;
}

void ExperimentUtils::run_namoa(
    const AdjacencyMatrix& adjecency_matrix, const size_t& source,
    const size_t& target, const EPS& eps, const MultiValuedHeuristic& mvh,
    const std::string& logging_file, unsigned int time_limit) {
    auto solver = NAMOA(adjecency_matrix, eps);
    SolutionSet solutions;
    solver(source, target, mvh, solutions, time_limit, "", "");
    std::cout << "algorithm=NAMOA"
        << "\tsource=" << source << "\ttarget=" << target
        << "\tnum_solutions=" << solutions.size()
        << "\tnum_expansion=" << solver.num_expansion
        << "\tnum_generation=" << solver.num_generation
        << "\tnum_full_dominance_check=" << solver.num_full_dominance_check
        << "\tnum_global_dominance_check=" << solver.num_global_dominance_check
        << "\truntime_s=" << solver.runtime / CLOCKS_PER_SEC
        << "\ttime_limit_reached=" << solver.time_limit_reached
        << "\tavg_pareto_size=" << solver.avg_pareto_size
        << "\tnum_reinsertion=" << solver.num_reinsertion
        << "\tnum_active_states=" << solver.num_active_states << std::endl;
}

void ExperimentUtils::single_run(AdjacencyMatrix& adjecency_matrix,
                                 size_t source, size_t target,
                                 const std::string& algorithm, const EPS& eps,
                                 std::vector<int> multi_sources,
                                 const Heuristic& svh_heuristic, const MultiValuedHeuristic& mvh,
                                 const std::string& logging_file, unsigned int time_limit) {
    Heuristic used_heuristic = svh_heuristic;
    if (used_heuristic == nullptr) {
        auto heuristic_start_time = std::clock();
        ShortestPathHeuristic sp_heuristic(target, adjecency_matrix.graph_size, adjecency_matrix);            
        Heuristic heuristic = std::bind(&ShortestPathHeuristic::operator(), sp_heuristic, _1);
            used_heuristic = heuristic;
        auto heuristic_duration = static_cast<long>(std::clock() - heuristic_start_time);
    }
    if (multi_sources.empty()) {
        multi_sources.push_back(source);
    }
    if (algorithm == "NAMOA_DR") {
            for (auto multi_source : multi_sources) {
            run_namoa_dr(adjecency_matrix, multi_source, target, used_heuristic, logging_file, time_limit);
        }
    } else if (algorithm == "BOA") {
        for (auto multi_source : multi_sources) {
            run_boa_star(adjecency_matrix, multi_source, target, svh_heuristic, logging_file);
        }
    }
    else if (algorithm == "APEX") {
        for (auto multi_source : multi_sources) {
                run_apex(adjecency_matrix, multi_source, target, eps, svh_heuristic, 0, logging_file);
        }

    } else if (algorithm == "IPH") {
        auto heuristic_start_time = std::clock();
        ShortestPathHeuristic sp_heuristic(target, adjecency_matrix.graph_size, adjecency_matrix);
        Heuristic heuristic = std::bind(&ShortestPathHeuristic::operator(), sp_heuristic, _1);
            used_heuristic = heuristic;
        auto heuristic_duration = static_cast<long>(std::clock() - heuristic_start_time);
        std::cout << "algorithm=IPH"
            << "\ttarget=" << target
            << "\theuristic_duration_s=" << heuristic_duration / CLOCKS_PER_SEC
            << std::endl;
        std::ofstream svh_file(logging_file + "_iph.txt");
        for (size_t i = 0; i <= adjecency_matrix.graph_size; ++i) {
            const auto& h = heuristic(i);
            svh_file << i;
            for (auto v : h) svh_file << "\t" << v;
            svh_file << "\n";
        }
    }
    else if ( algorithm == "NAMOA-MVH" ) {
        run_namoa_backward_search(adjecency_matrix, target, logging_file);
    }
    else if (algorithm == "APEX_MVH") {
        Heuristic blind_heuristic = [](size_t vertex) -> std::vector<size_t> {
            return {0, 0};
        };
        run_apex_mvh(adjecency_matrix, target, eps, logging_file);

    }
    else if (algorithm == "NAMOA_DR_MVH") {
        if (multi_sources.empty()) {
            multi_sources.push_back(source);
        }
        for (auto multi_source : multi_sources) {
            run_namoa_dr_mvh(adjecency_matrix, multi_source, target, eps, mvh, logging_file, time_limit);
        }
    }
    else if (algorithm == "L_NAMOA_DR_MVH") {
        const MultiValuedHeuristic* used_mvh = &mvh;
        MultiValuedHeuristic computed_mvh;
        if (mvh.empty()) {
            auto apex_mvh_builder = APEX_MVH(adjecency_matrix, eps);
            computed_mvh = apex_mvh_builder(target, "");
            used_mvh = &computed_mvh;
        }
        for (auto multi_source : multi_sources) {
            run_l_namoa_dr_mvh(adjecency_matrix, multi_source, target, eps, *used_mvh, logging_file, time_limit);
        }
    } else if (algorithm == "NAIVE_DR_MVH_BRIDGING") {
        const MultiValuedHeuristic* used_mvh = &mvh;
        MultiValuedHeuristic computed_mvh;
        if (mvh.empty()) {
            auto apex_mvh_builder = APEX_MVH(adjecency_matrix, eps);
            computed_mvh = apex_mvh_builder(target, "");
            used_mvh = &computed_mvh;
        }
        for (auto multi_source : multi_sources) {
            run_naive_dr_mvh_bridging(adjecency_matrix, multi_source, target, eps, *used_mvh, logging_file, time_limit);
        }
    } else if (algorithm == "NAMOA") {
        const MultiValuedHeuristic* used_mvh = &mvh;
        MultiValuedHeuristic computed_mvh;
        if (mvh.empty()) {
            auto apex_mvh_builder = APEX_MVH(adjecency_matrix, eps);
            computed_mvh = apex_mvh_builder(target, "");
            used_mvh = &computed_mvh;
        }
        for (auto multi_source : multi_sources) {
            run_namoa(adjecency_matrix, multi_source, target, eps, *used_mvh, logging_file, time_limit);
        }
    } else if (algorithm == "Closure") {
        auto closure = Closure(adjecency_matrix);
        MultiValuedHeuristic new_mvh = closure(mvh, logging_file + "_closure_mvh.txt");
        float average_mvh_size = 0.0f;
        for (const auto& entry : new_mvh) {
            average_mvh_size += entry.size();
        }
        average_mvh_size /= new_mvh.size();
        std::cout << "algorithm=Closure"
            << "\ttarget=" << target
            << "\tavg_mvh_size=" << average_mvh_size
            << "\tnum_expansion=" << closure.num_expansion
            << "\tnum_generation=" << closure.num_generation
            << "\truntime_s=" << closure.runtime / CLOCKS_PER_SEC << std::endl;
    }
    else {
        std::cout << "Unknown algorithm: " << algorithm << std::endl;
    }
}
