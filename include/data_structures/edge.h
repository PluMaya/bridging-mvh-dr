//
// Created by crl on 13/07/2024.
//

#ifndef EDGE_H
#define EDGE_H
#include <vector>

class Edge {
public:
  size_t source;
  size_t target;
  std::vector<size_t> cost;

  Edge(const size_t source, const size_t target, std::vector<size_t> cost)
      : source(source), target(target), cost(std::move(cost)) {}

  Edge inverse() const { return {target, source, cost}; }
};

#endif // EDGE_H
