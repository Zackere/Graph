// Copyright 2019 Wojciech Replin

#ifndef ADJACENCYLISTS_ADJACENCY_CONTAINER_HPP_
#define ADJACENCYLISTS_ADJACENCY_CONTAINER_HPP_

#include <utility>
#include <vector>

#include "../edge.hpp"

namespace Graphlib {
template <typename Container, typename Vertex>
class AdjacencyContainer {
 public:
  explicit AdjacencyContainer(std::size_t size);
  std::size_t size() const;
  bool insert(Vertex key, typename Edge<Vertex>::Weight value);
  bool remove(Vertex key);
  bool exist(Vertex key) const;
  typename Edge<Vertex>::Weight const& operator[](Vertex key) const;
  typename Edge<Vertex>::Weight& operator[](Vertex key);
};
}  // namespace Graphlib
#endif  // ADJACENCYLISTS_ADJACENCY_CONTAINER_HPP_
