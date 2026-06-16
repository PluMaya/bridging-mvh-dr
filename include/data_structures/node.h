//
// Created by crl on 13/07/2024.
//

#ifndef NODE_H
#define NODE_H
#include <memory>
#include <utility>
#include <vector>

/**
 * Class representing a node in a search problem
 */
class Node;
using NodePtr = std::shared_ptr<Node>;

class Node {
public:
  size_t id;
  std::vector<size_t> g;
  std::vector<size_t> h;
  std::vector<size_t> f;
  NodePtr parent;
  const std::vector<size_t>& c;
  size_t h_idx = 0;

  /**
   * Constructing a new node.
   *
   * @param id the ID (state?) of a node
   * @param g the g-value of a node
   * @param h the h-value (heuristic) of a node
   * @param parent the parent of a node
   */
  Node(const size_t id, const std::vector<size_t> &g,
       const std::vector<size_t> &h, NodePtr parent = nullptr, const std::vector<size_t> &c = std::vector<size_t>(2, 0 ),
       const size_t h_idx = 0)
      : id(id), g(g), h(h), f(g.size()), parent(std::move(parent)), c(c), h_idx(h_idx){
    for (int i = 0; i < g.size(); i++) {
      f[i] = g[i] + h[i];
    }
  };

  Node(const size_t id, const std::vector<size_t> &g,
       const std::vector<size_t> &h, const std::vector<size_t> &f,const std::vector<size_t> &c, NodePtr parent = nullptr,
       const size_t h_idx = 0)
      : id(id), g(g), h(h), f(f), c(c), parent(std::move(parent)), h_idx(h_idx){ };
};

struct CompareNodeByFValue {
  bool operator()(const NodePtr& a, const NodePtr& b) const {

    return a->f > b->f;
  }
};

struct CompareNodeByGValue {
  bool operator()(const NodePtr& a, const NodePtr& b) const {

    return a->g > b->g;
  }
};

#endif // NODE_H
