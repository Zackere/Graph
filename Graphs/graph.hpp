// Copyright 2019 Wojciech Replin

#ifndef GRAPHS_GRAPH_HPP_
#define GRAPHS_GRAPH_HPP_

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "../AdjacencyLists/adjacency_container.hpp"
#include "edge.hpp"

namespace Graphlib {
template <typename Container>
class Graph {
 public:
  using vertex = int;
  Graph(bool directed, std::size_t verticies);
  bool directed() const;
  std::size_t nverticies() const;
  std::size_t nedges() const;
  bool add_edge(Edge<vertex> const& edge);
  bool add_edge(vertex from,
                vertex to,
                typename Edge<vertex>::Weight weight = 1.0);
  bool remove_edge(Edge<vertex> const& edge);
  bool remove_edge(vertex from, vertex to);
  std::optional<typename Edge<vertex>::Weight> get_edge_weight(vertex from,
                                                               vertex to) const;
  unsigned int in_degree(vertex v) const;
  unsigned int out_degree(vertex v) const;
  bool modify_edge_weight(vertex from,
                          vertex to,
                          typename Edge<vertex>::Weight add);
  std::vector<Edge<vertex>> out_edges(vertex from) const;

 private:
  std::vector<AdjacencyContainer<Container, vertex>> adjacency_container_;
  bool directed_ = true;
  std::size_t nedges_ = 0;
};

template <typename Container>
inline Graph<Container>::Graph(bool directed, std::size_t verticies)
    : adjacency_container_(
          std::vector(verticies,
                      AdjacencyContainer<Container, vertex>(verticies))),
      directed_(directed) {}

template <typename Container>
inline bool Graph<Container>::directed() const {
  return directed_;
}

template <typename Container>
inline std::size_t Graph<Container>::nverticies() const {
  return adjacency_container_.size();
}

template <typename Container>
inline std::size_t Graph<Container>::nedges() const {
  return nedges_;
}

template <typename Container>
inline bool Graph<Container>::add_edge(Edge<vertex> const& edge) {
  return add_edge(edge.from, edge.to, edge.weight);
}

template <typename Container>
inline bool Graph<Container>::add_edge(vertex from,
                                       vertex to,
                                       typename Edge<vertex>::Weight weight) {
  return adjacency_container_[from].insert(to, weight);
}

template <typename Container>
inline bool Graph<Container>::remove_edge(Edge<vertex> const& edge) {
  return remove_edge(edge.from, edge.to, edge.weight);
}

template <typename Container>
inline bool Graph<Container>::remove_edge(vertex from, vertex to) {
  return adjacency_container_[from].remove(to);
}

template <typename Container>
inline std::optional<typename Edge<typename Graph<Container>::vertex>::Weight>
Graph<Container>::get_edge_weight(vertex from, vertex to) const {
  return adjacency_container_[from][to];
}

template <typename Container>
inline unsigned int Graph<Container>::in_degree(vertex v) const {
  unsigned int in_deg = 0;
  for (auto const& container : adjacency_container_)
    if (container.exist(v))
      ++in_deg;
  return in_deg;
}

template <typename Container>
inline unsigned int Graph<Container>::out_degree(vertex v) const {
  return adjacency_container_[v].size();
}

template <typename Container>
inline bool Graph<Container>::modify_edge_weight(
    vertex from,
    vertex to,
    typename Edge<vertex>::Weight add) {
  if (!adjacency_container_[from][to].has_value())
    return false;
  adjacency_container_[from][to].value() += add;
  return true;
}

template <typename Container>
inline std::vector<Edge<typename Graph<Container>::vertex>>
Graph<Container>::out_edges(vertex from) const {
  std::vector<Edge<vertex>> ret;
  ret.reserve(out_degree(from));
  for (auto const& elem : adjacency_container_[from])
    ret.emplace_back(Edge<vertex>{from, elem.first, elem.second});
  return ret;
}

}  // namespace Graphlib

#endif  // GRAPHS_GRAPH_HPP_
