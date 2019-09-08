// Copyright 2019 Wojciech Replin

#ifndef GRAPHS_GRAPH_HPP_
#define GRAPHS_GRAPH_HPP_

#include <memory>
#include <utility>

#include "../AdjacencyLists/adjacency_container.hpp"
#include "../edge.hpp"

namespace Graphlib {
template <typename Container>
class Graph {
 public:
  using vertex = int;
  Graph(bool directed, std::size_t verticies);
  bool Directed() const;
  std::size_t NVerticies() const;
  std::size_t NEdges() const;

 private:
  AdjacencyContainer<Container, vertex> adjacency_list_;
  bool directed_ = true;
  std::size_t nedges_ = 0;
};

template <typename Container>
inline Graph<Container>::Graph(bool directed, std::size_t verticies)
    : adjacency_list_(verticies), directed_(directed) {}

}  // namespace Graphlib

#endif  // GRAPHS_GRAPH_HPP_
