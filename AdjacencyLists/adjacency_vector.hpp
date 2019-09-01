// Copyright 2019 Wojciech Replin

#ifndef ADJACENCYLISTS_ADJACENCY_VECTOR_HPP_
#define ADJACENCYLISTS_ADJACENCY_VECTOR_HPP_

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <utility>
#include <vector>

#include "adjacency_container.hpp"

namespace Graphlib {
template <>
class AdjacencyContainer<
    std::vector<std::pair<bool, typename Edge<int>::Weight>>,
    int> {
 public:
  explicit AdjacencyContainer(std::size_t size);
  std::size_t size() const;
  bool insert(int key, typename Edge<int>::Weight value);
  bool remove(int key);
  bool exist(int key) const;
  typename Edge<int>::Weight const& operator[](int key) const;
  typename Edge<int>::Weight& operator[](int key);

 private:
  void CheckRange(int key) const;
  std::vector<std::pair<bool, double>> vector_;
};

using AdjacencyVector =
    AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                       int>;

inline AdjacencyContainer<
    std::vector<std::pair<bool, typename Edge<int>::Weight>>,
    int>::AdjacencyContainer(std::size_t size)
    : vector_(size, std::make_pair(false, 0.0f)) {}

inline std::size_t
AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                   int>::size() const {
  return vector_.size();
}

inline bool
AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                   int>::insert(int key, typename Edge<int>::Weight value) {
  CheckRange(key);
  if (vector_[key].first)
    return false;
  vector_[key] = std::make_pair(true, value);
  return true;
}

inline bool
AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                   int>::remove(int key) {
  CheckRange(key);
  if (!vector_[key].first)
    return false;
  vector_[key].first = false;
  return true;
}

inline bool
AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                   int>::exist(int key) const {
  CheckRange(key);
  return vector_[key].first;
}

inline typename Edge<int>::Weight const&
    AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                       int>::operator[](int key) const {
  CheckRange(key);
  return vector_[key].second;
}

inline typename Edge<int>::Weight&
    AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                       int>::operator[](int key) {
  return const_cast<typename Edge<int>::Weight&>(
      static_cast<AdjacencyContainer<
          std::vector<std::pair<bool, typename Edge<int>::Weight>>,
          int> const&>(*this)
          .
          operator[](key));
}

inline void
AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                   int>::CheckRange(int key) const {
  if (key < 0 || key >= vector_.size())
    throw std::out_of_range("Index is out of vectors' range");
}
}  // namespace Graphlib
#endif  // ADJACENCYLISTS_ADJACENCY_VECTOR_HPP_
