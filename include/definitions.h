//
// Created by crl on 13/07/2024.
//

#ifndef DEFINITIONS_H
#define DEFINITIONS_H
#include <array>
#include <functional>
#include <limits>
#include <map>
#include <ostream>
#include <vector>

constexpr size_t MAX_COST = std::numeric_limits<size_t>::max();

template <typename T> using Pair = std::array<T, 2>;

template <typename T>
std::ostream &operator<<(std::ostream &stream, const Pair<T> pair) {
  stream << "[" << pair[0] << ", " << pair[1] << "]";
  return stream;
}

using Heuristic = std::function<std::vector<size_t>(size_t)>;
using EdgeHeuristic = std::function<std::vector<size_t>(size_t, size_t)>;
using MapBasedEdgeHeuristic = std::map<std::pair<size_t, size_t>, std::vector<size_t>>;
using UpperBoundHeuristic = std::function<std::vector<std::vector<size_t>>(size_t)>;
using MultiValuedHeuristic = std::vector<std::vector<std::vector<size_t>>>;

using EPS = std::vector<double>;


/** Other definitions **/

// Custom hash function for std::pair
struct pair_hash {
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2>& pair) const {
    auto hash1 = std::hash<T1>{}(pair.first);
    auto hash2 = std::hash<T2>{}(pair.second);
    return hash1 ^ (hash2 << 1); // Combine the hashes
  }
};

using EdgeBasedHeuristic = std::unordered_map<std::pair<size_t, size_t>, std::vector<size_t>, pair_hash>;

#endif // DEFINITIONS_H
