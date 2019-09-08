// Copyright 2019 Wojciech Replin

#ifndef ADJACENCYLISTS_ADJACENCY_CONTAINER_HPP_
#define ADJACENCYLISTS_ADJACENCY_CONTAINER_HPP_

#include <memory>
#include <utility>
#include <vector>

#include "../edge.hpp"

namespace Graphlib {
template <typename Container, typename Vertex, typename Enable = void>
class AdjacencyContainer {
 public:
  class iterator {
   public:
    iterator& operator++();
    iterator& operator--();
    bool operator==(iterator const& other) const;
    bool operator!=(iterator const& other) const;
    std::pair<Vertex, typename Edge<Vertex>::Weight&> operator*();
    std::unique_ptr<std::pair<int, typename Edge<int>::Weight&>> operator->();
  };
  explicit AdjacencyContainer(std::size_t size);
  std::size_t size() const;
  bool insert(Vertex key, typename Edge<Vertex>::Weight value);
  bool remove(Vertex key);
  bool exist(Vertex key) const;
  typename Edge<Vertex>::Weight const& operator[](Vertex key) const;
  typename Edge<Vertex>::Weight& operator[](Vertex key);
  iterator begin() const;
  iterator end() const;
};
}  // namespace Graphlib
#endif  // ADJACENCYLISTS_ADJACENCY_CONTAINER_HPP_
