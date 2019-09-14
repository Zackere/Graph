// Copyright 2019 Wojciech Replin

#ifndef ADJACENCYLISTS_ADJACENCY_CONTAINER_HPP_
#define ADJACENCYLISTS_ADJACENCY_CONTAINER_HPP_

#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "../Graphs/edge.hpp"

namespace Graphlib {
template <typename Container, typename Vertex, typename Enable = void>
class AdjacencyContainer {
 public:
  class Iterator {
   public:
    Iterator& operator++();
    Iterator& operator--();
    bool operator==(Iterator const& other) const;
    bool operator!=(Iterator const& other) const;
    std::pair<Vertex, typename Edge<Vertex>::Weight&> operator*();
    std::unique_ptr<std::pair<int, typename Edge<int>::Weight&>> operator->();
  };
  explicit AdjacencyContainer(std::size_t size);
  std::size_t size() const;
  bool insert(Vertex key, typename Edge<Vertex>::Weight value);
  bool remove(Vertex key);
  bool exist(Vertex key) const;
  std::optional<std::reference_wrapper<typename Edge<Vertex>::Weight const>>
  operator[](Vertex key) const;
  std::optional<std::reference_wrapper<typename Edge<Vertex>::Weight>>
  operator[](Vertex key);
  Iterator begin() const;
  Iterator end() const;
};
}  // namespace Graphlib
#endif  // ADJACENCYLISTS_ADJACENCY_CONTAINER_HPP_
