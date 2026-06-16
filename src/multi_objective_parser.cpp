//
// Created by crl on 13/12/2024.
//


#include "multi_objective_parser.h"

#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>

AdjacencyMatrix MultiObjectiveParser::parse_graph(const int max_objectives) {
  std::ifstream infile(file_name);
  if (!infile) {
    std::cerr << "Error opening file: " << file_name << std::endl;
    exit(1);
  }

  std::vector<Edge> edges;
  size_t max_node_num = 0;
  std::string line;
  while (!infile.eof()) {
    std::getline(infile, line);

    if (line.empty()) {
      break;
    }

    std::vector<std::string> decomposed_line;

    split(decomposed_line, line, boost::is_any_of(" "));
    size_t source = std::stoul(decomposed_line[0]);
    size_t target = std::stoul(decomposed_line[1]);
    std::vector<size_t> cost(decomposed_line.size() - 2);
    int n_objectives = std::min(int(decomposed_line.size() - 2), max_objectives);
    for (int i = 0; i < n_objectives; i++) {
      cost[i] = static_cast<size_t>(std::stod(decomposed_line[i + 2]));
    }
    Edge e(source, target, cost);

    edges.push_back(e);
    max_node_num = std::max({max_node_num, e.source, e.target});
  }
  return {max_node_num, edges, true};
}