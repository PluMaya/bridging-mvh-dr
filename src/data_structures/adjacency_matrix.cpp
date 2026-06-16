//
// Created by crl on 13/07/2024.
//

#include "data_structures/adjacency_matrix.h"

AdjacencyMatrix::AdjacencyMatrix(size_t graph_size, std::vector<Edge> &edges,
                                 const bool inverse)
    : matrix((graph_size + 1), std::vector<Edge>()), graph_size(graph_size) {
  num_of_objectives = edges[0].cost.size();

  for (auto &edge : edges) {
    add(edge);
    if (inverse) {
      add(edge.inverse());
    }
  }
}

void AdjacencyMatrix::add(const Edge &edge) {
  matrix[edge.source].push_back(edge);
}

size_t AdjacencyMatrix::size() const { return graph_size; }

const std::vector<Edge> &AdjacencyMatrix::operator[](size_t vertex_id) const {
  return matrix.at(vertex_id);
}