//
// Created by crl on 15/07/2024.
//

#include "data_structures/map_queue.h"

#include <algorithm>

ApexPathPairPtr MapQueue::pop() {
  ApexPathPairPtr pp = heap.top();
  heap.pop();
  std::vector<ApexPathPairPtr>& relevant_pps = open_map[pp->id];
  for (size_t i = 0; i < relevant_pps.size(); ++i) {
    if (pp == relevant_pps[i]) {
      relevant_pps[i] = std::move(relevant_pps.back());
      relevant_pps.pop_back();
      break;
    }
  }
  return pp;
}

void MapQueue::insert(const ApexPathPairPtr &pp) {
  heap.push(pp);
  open_map[pp->id].push_back(pp);
}
