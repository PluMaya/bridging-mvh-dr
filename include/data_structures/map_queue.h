//
// Created by crl on 15/07/2024.
//

#ifndef MAP_QUEUE_H
#define MAP_QUEUE_H
#include "apex_path_pair.h"

#include <queue>
#include <vector>

class MapQueue {
public:
  std::priority_queue<ApexPathPairPtr, std::vector<ApexPathPairPtr>,
                      CompareApexPathPairByValue> heap;
  std::vector<std::vector<ApexPathPairPtr>> open_map;

  explicit MapQueue(size_t graph_size) : open_map(graph_size) {};
  bool empty() const { return heap.empty(); }
  ApexPathPairPtr pop();
  void insert(const ApexPathPairPtr &pp);
  std::vector<ApexPathPairPtr> &get_open(size_t id) { return open_map[id]; };
};

#endif // MAP_QUEUE_H
