//
// Created by crl on 13/07/2024.
//

#ifndef ADJACENCY_MATRIX_H
#define ADJACENCY_MATRIX_H
#include <vector>

#include "edge.h"

// Graph representation as adjacency matrix
class AdjacencyMatrix {
public:
  std::vector<std::vector<Edge>> matrix;
  size_t graph_size{};
  size_t num_of_objectives = 2;

  AdjacencyMatrix() = default;
  AdjacencyMatrix(size_t graph_size, std::vector<Edge> &edges,
                  bool inverse = false);

  void add(const Edge &edge);
  size_t size() const;
  const std::vector<Edge> &operator[](size_t vertex_id) const;
};

#endif // ADJACENCY_MATRIX_H
