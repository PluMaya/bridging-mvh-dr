//
// Created by crl on 15/02/2026.
//


#include "parsers/multi_valued_heuristic_parser.h"

#include <fstream>
#include <iostream>
#include <boost/algorithm/string.hpp>


MultiValuedHeuristic MVHParser::parse_heuristic(const std::string& heuristic_path) {
    std::ifstream heuristic_infile(heuristic_path);
    if (!heuristic_infile) {
        throw std::runtime_error("Error opening file: " + heuristic_path);
    }

    std::string line;
    size_t max_state = 0;

    // First pass: find max state
    while (std::getline(heuristic_infile, line)) {
        if (line.empty()) continue;

        std::vector<std::string> tokens;
        split(tokens, line, boost::is_any_of("\t"));

        size_t state = std::stoul(tokens[0]);
        max_state = std::max(max_state, state);
    }

    // Allocate all states with empty vectors
    MultiValuedHeuristic mvh(max_state + 1);

    // Reset stream
    heuristic_infile.clear();
    heuristic_infile.seekg(0);

    // Second pass: fill
    while (std::getline(heuristic_infile, line)) {
        if (line.empty()) continue;

        std::vector<std::string> tokens;
        split(tokens, line, boost::is_any_of("\t"));

        size_t state = std::stoul(tokens[0]);

        std::vector<size_t> h;
        h.reserve(tokens.size() - 1);
        for (size_t k = 1; k < tokens.size(); ++k)
            h.push_back(static_cast<size_t>(std::stod(tokens[k])));

        mvh[state].push_back(std::move(h));
    }

    return mvh;
}