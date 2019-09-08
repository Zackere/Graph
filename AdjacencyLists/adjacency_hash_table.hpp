// Copyright 2019 Wojciech Replinstd::unordered_map<int, typename
// Edge<int>::Weight>

#ifndef ADJACENCYLISTS_ADJACENCY_HASH_TABLE_HPP_
#define ADJACENCYLISTS_ADJACENCY_HASH_TABLE_HPP_

#include <algorithm>
#include <iterator>
#include <memory>
#include <unordered_map>
#include <utility>

#include "../edge.hpp"
#include "adjacency_container.hpp"

namespace Graphlib {
template <>
class AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                         int> {
 public:
  class iterator {
   public:
    iterator& operator++();
    iterator& operator--();
    bool operator==(iterator const& other) const;
    bool operator!=(iterator const& other) const;
    std::pair<int, typename Edge<int>::Weight&> operator*();
    std::unique_ptr<std::pair<int, typename Edge<int>::Weight&>> operator->();

   private:
    friend class AdjacencyContainer<
        std::unordered_map<int, typename Edge<int>::Weight>,
        int>;
    iterator(std::unordered_map<int, typename Edge<int>::Weight>::iterator iter,
             std::unordered_map<int, typename Edge<int>::Weight>* map);
    std::unordered_map<int, typename Edge<int>::Weight>::iterator current_;
    std::unordered_map<int, typename Edge<int>::Weight>* const map_;
  };

  explicit AdjacencyContainer(std::size_t size);
  std::size_t size() const;
  bool insert(int key, typename Edge<int>::Weight value);
  bool remove(int key);
  bool exist(int key) const;
  typename Edge<int>::Weight const& operator[](int key) const;
  typename Edge<int>::Weight& operator[](int key);
  iterator begin();
  iterator end();

 private:
  std::unordered_map<int, typename Edge<int>::Weight> map_;
};

using AdjacencyHashTable =
    AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                       int>;

inline AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                          int>::AdjacencyContainer(std::size_t size) {}

inline std::size_t
AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                   int>::size() const {
  return map_.size();
}

inline bool
AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                   int>::insert(int key, typename Edge<int>::Weight value) {
  if (exist(key))
    return false;
  map_.emplace(std::make_pair(key, value));
  return true;
}

inline bool
AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                   int>::remove(int key) {
  for (auto it = map_.begin(); it != map_.end(); ++it) {
    if (it->first == key) {
      map_.erase(it);
      return true;
    }
  }
  return false;
}

inline bool
AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                   int>::exist(int key) const {
  return map_.find(key) != map_.end();
}

inline typename Edge<int>::Weight const&
    AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                       int>::operator[](int key) const {
  return std::find_if(map_.begin(), map_.end(),
                      [key](auto const& pair) { return pair.first == key; })
      ->second;
}

typename Edge<int>::Weight&
    AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                       int>::operator[](int key) {
  return const_cast<Edge<int>::Weight&>(
      static_cast<AdjacencyContainer<
          std::unordered_map<int, typename Edge<int>::Weight>, int> const&>(
          *this)[key]);
}

inline AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                          int>::iterator
AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                   int>::begin() {
  return iterator(map_.begin(), &map_);
}

inline AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                          int>::iterator
AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                   int>::end() {
  return iterator(map_.end(), &map_);
}

inline AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                          int>::iterator::
    iterator(std::unordered_map<int, typename Edge<int>::Weight>::iterator iter,
             std::unordered_map<int, typename Edge<int>::Weight>* map)
    : current_(iter), map_(map) {}

inline AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                          int>::iterator&
AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                   int>::iterator::operator++() {
  if (current_ != map_->end())
    ++current_;
  return *this;
}

inline AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                          int>::iterator&
AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                   int>::iterator::operator--() {
  if (current_ != map_->begin())
    --current_;
  return *this;
}

inline bool
AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                   int>::iterator::operator==(iterator const& other) const {
  return current_ == other.current_ && map_ == other.map_;
}

inline bool
AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                   int>::iterator::operator!=(iterator const& other) const {
  return !(*this == other);
}

inline std::pair<int, typename Edge<int>::Weight&>
    AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                       int>::iterator::operator*() {
  if (current_ == map_->end())
    throw std::out_of_range("End iterator is not dereferencable");
  return {current_->first, current_->second};
}

inline std::unique_ptr<std::pair<int, typename Edge<int>::Weight&>>
    AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                       int>::iterator::operator->() {
  return std::make_unique<std::pair<int, typename Edge<int>::Weight&>>(
      operator*());
}

}  // namespace Graphlib
#endif  // ADJACENCYLISTS_ADJACENCY_LIST_HPP_
