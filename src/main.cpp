#include "experiment_utils.h"
#include "parser.h"
#include <algorithm>
#include <fstream>
#include <iostream>

#include <boost/program_options.hpp>

#include "parsers/multi_valued_heuristic_parser.h"
#include "parsers/single_valued_heuristic_parser.h"

using namespace std;

const std::string output_path = "../output/";

int main(int argc, char** argv) {
    boost::program_options::options_description options_description(
        "Allowed options");
    options_description.add_options()("help,h", "produce help message")(
        "start,s", boost::program_options::value<int>()->default_value(-1),
        "start location")("goal,g",
                          boost::program_options::value<int>()->default_value(-1),
                          "goal location")(
        "query,q",
        boost::program_options::value<std::string>()->default_value(""),
        "number of agents")(
        "map,m", boost::program_options::value<std::string>()->default_value(""),
        "directory for edge weights files")(
        "eps", boost::program_options::value<double>()->default_value(0),
        "approximation factor (applied uniformly to all objectives)")(
        "algorithm,a",
        boost::program_options::value<std::string>()->default_value("Apex"),
        "solvers")("cutoffTime,t",
                   boost::program_options::value<int>()->default_value(0),
                   "cutoff time in seconds (0 = no limit)")(
        "logging_file",
        boost::program_options::value<std::string>()->default_value(""),
        "logging file")("multi_source,ms",
                        boost::program_options::value<vector<int>>()->multitoken()->default_value(vector<int>{}, ""),
                        "sources")(
        "svh,h", boost::program_options::value<std::string>()->default_value(""),
        "directory for single valued heuristic file")(
        "mvh,h", boost::program_options::value<std::string>()->default_value(""),
        "directory for multi valued heuristic file")(
        "objectives",
        boost::program_options::value<vector<int>>()->multitoken()->default_value(vector<int>{}, ""),
        "0-based indices of objectives to load (default: all)");


    boost::program_options::variables_map vm;
    auto options = parse_command_line(argc, argv, options_description);
    store(options, vm);
    notify(vm);

    // ── Phase 1: validate all inputs before touching the filesystem ───────────

    const std::string query_file  = vm["query"].as<std::string>();
    const std::string map_dir     = vm["map"].as<std::string>();
    const std::string svh_path    = vm["svh"].as<std::string>();
    const std::string mvh_path    = vm["mvh"].as<std::string>();
    const std::string algorithm   = vm["algorithm"].as<std::string>();
    const std::string logging_file = vm["logging_file"].as<std::string>();
    const int start_val = vm["start"].as<int>();
    const int goal_val  = vm["goal"].as<int>();
    const double eps_val = vm["eps"].as<double>();
    const unsigned int cutoff_time = vm["cutoffTime"].as<int>() > 0 ? static_cast<unsigned int>(vm["cutoffTime"].as<int>()) : 0;
    const std::vector<int> multi_sources = vm["multi_source"].as<std::vector<int>>();
    const std::vector<int> objectives_raw = vm["objectives"].as<std::vector<int>>();
    std::vector<size_t> objectives(objectives_raw.begin(), objectives_raw.end());

    if (map_dir.empty()) {
        std::cerr << "error: --map is required" << std::endl;
        return -1;
    }

    if (!query_file.empty()) {
        if (start_val != -1 || goal_val != -1) {
            std::cerr << "error: --query and --start/--goal cannot be given together" << std::endl;
            return -1;
        }
        if (!std::ifstream(query_file).good()) {
            std::cerr << "error: cannot open query file: " << query_file << std::endl;
            return -1;
        }
    } else {
        if (start_val == -1 || goal_val == -1) {
            std::cerr << "error: must provide either --query or both --start and --goal" << std::endl;
            return -1;
        }
    }

    if (!svh_path.empty() && !std::ifstream(svh_path).good()) {
        std::cerr << "error: cannot open SVH file: " << svh_path << std::endl;
        return -1;
    }

    if (!mvh_path.empty() && !std::ifstream(mvh_path).good()) {
        std::cerr << "error: cannot open MVH file: " << mvh_path << std::endl;
        return -1;
    }

    const std::vector<std::string> known_algorithms = {
        "NAMOA_DR", "BOA", "APEX", "IPH", "NAMOA-MVH", "APEX_MVH", "NAMOA_DR_MVH", "L_NAMOA_DR_MVH", "NAIVE_DR_MVH_BRIDGING", "Closure", "NAMOA"
    };
    if (std::find(known_algorithms.begin(), known_algorithms.end(), algorithm) == known_algorithms.end()) {
        std::cerr << "error: unknown algorithm '" << algorithm << "'" << std::endl;
        return -1;
    }

    // ── Phase 2: load data structures ────────────────────────────────────────

    auto adjecency_matrix = Parser(map_dir, objectives).default_graph();

    // Validate start/goal against graph size
    const size_t graph_size = adjecency_matrix.size();
    if (!query_file.empty()) {
        // bounds will be checked per-query below
    } else {
        if (static_cast<size_t>(start_val) >= graph_size + 1) {
            std::cerr << "error: start node " << start_val << " out of bounds (graph has " << graph_size << " nodes)" << std::endl;
            return -1;
        }
        if (static_cast<size_t>(goal_val) >= graph_size + 1) {
            std::cerr << "error: goal node " << goal_val << " out of bounds (graph has " << graph_size << " nodes)" << std::endl;
            return -1;
        }
    }

    const EPS eps(adjecency_matrix.num_of_objectives, eps_val);

    Heuristic heuristic = nullptr;
    if (!svh_path.empty()) {
        heuristic = SVHParser::parse_heuristic(svh_path);
    }

    MultiValuedHeuristic mvh = {};
    if (!mvh_path.empty()) {
        mvh = MVHParser::parse_heuristic(mvh_path);
    }

    // ── Phase 3: run ─────────────────────────────────────────────────────────

    if (!query_file.empty()) {
        std::ifstream qfile(query_file);
        int s, t;
        while (qfile >> s >> t) {
            ExperimentUtils::single_run(adjecency_matrix, s, t, algorithm,
                                        eps, multi_sources, heuristic, mvh, logging_file, cutoff_time);
        }
    } else {
        ExperimentUtils::single_run(adjecency_matrix, start_val, goal_val, algorithm,
                                    eps, multi_sources, heuristic, mvh, logging_file, cutoff_time);
    }

    return 0;
}
