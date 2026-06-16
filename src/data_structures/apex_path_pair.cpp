//
// Created by crl on 13/07/2024.
//

#include "data_structures/apex_path_pair.h"

bool is_bounded(const NodePtr& apex, const NodePtr& node, const EPS& eps) {
    for (int i = 0; i < apex->f.size(); i++) {
        if (node->f[i] > (1 + eps[i]) * apex->f[i]) {
            return false;
        }
    }
    return true;
}

ApexPathPair::ApexPathPair(const ApexPathPairPtr& parent, const Edge& edge,
                           const std::vector<size_t>& h)
    : id(edge.target), h(h), parent(parent) {
    std::vector new_apex_g(parent->apex->g);
    std::vector new_g(parent->path_node->g);

    for (int i = 0; i < new_apex_g.size(); i++) {
        new_apex_g[i] += edge.cost[i];
        new_g[i] += edge.cost[i];
    }
    this->edge = edge;
    apex = std::make_shared<Node>(id, new_apex_g, h);
    path_node = std::make_shared<Node>(id, new_g, h);
}

bool ApexPathPair::update_nodes_by_merge_if_bounded(
    const ApexPathPairPtr& other, const std::vector<double>& eps) {
    // Returns true on sucessful merge
    if (id != other->id) {
        return false;
    }

    std::vector<size_t> f_value(apex->f.size());
    for (int i = 0; i < apex->f.size(); i++) {
        f_value[i] = std::min(apex->f[i], other->apex->f[i]);
    }

    for (int i = 0; i < path_node->f.size(); i++) {
        if (path_node->f[i] > (1 + eps[i]) * f_value[i]) {
            return false;
        }
    }

    for (int i = 0; i < apex->g.size(); i++) {
        apex->g[i] = std::min(apex->g[i], other->apex->g[i]);
        apex->f[i] = std::min(apex->f[i], other->apex->f[i]);
    }
    return true;
}

bool ApexPathPair::update_apex_by_merge_if_bounded(
    const NodePtr& other_apex, const std::vector<double>& eps) {
    std::vector<size_t> f(apex->f.size());
    for (int i = 0; i < apex->f.size(); i++) {
        f[i] = std::min(apex->f[i], other_apex->f[i]);
    }

    for (int i = 0; i < path_node->f.size(); i++) {
        if (path_node->f[i] > (1 + eps[i]) * f[i]) {
            return false;
        }
    }

    apex->f = f;
    for (int i = 0; i < apex->g.size(); i++) {
        apex->g[i] = std::min(apex->g[i], other_apex->g[i]);
    }
    return true;
}
