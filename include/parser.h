//
// Created by crl on 13/07/2024.
//

#ifndef PARSER_H
#define PARSER_H

#include <data_structures/adjacency_matrix.h>
#include <string>
#include <utility>
#include <vector>

inline std::string DEFAULT_FILES_DIRECTORY = "C:/Users/crl/CLionProjects/MultivaluedHeuristicSearch/resources/NYC";

class Parser {
public:
  std::string files_directory;
  std::vector<size_t> objective_indices; // empty = use all objectives

  explicit Parser(std::string files_directory, std::vector<size_t> objectives = {})
      : files_directory(std::move(files_directory)),
        objective_indices(std::move(objectives)) {}

  explicit Parser()
      : files_directory(DEFAULT_FILES_DIRECTORY) {}

  // If the directory contains distances.gr + times.gr, use the classic 2-file
  // format.  Otherwise, every *.gr file in the directory is treated as one
  // objective (sorted alphabetically).  `objectives` is a list of 0-based
  // indices into the sorted file list; empty means "use all".
  static AdjacencyMatrix parse_graph(const std::string &default_files_directory,
                                     const std::vector<size_t> &objectives = {});

  [[nodiscard]] AdjacencyMatrix default_graph() const {
    return parse_graph(files_directory, objective_indices);
  }
};

#endif // PARSER_H
