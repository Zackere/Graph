// Copyright 2019 Wojciech Replinstd::unordered_map<int, typename
// Edge<int>::Weight>

#ifndef ADJACENCYLISTS_ADJACENCY_MAP_HPP_
#define ADJACENCYLISTS_ADJACENCY_MAP_HPP_

#include <algorithm>
#include <iterator>
#include <map>
#include <memory>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include "../edge.hpp"
#include "adjacency_container.hpp"

namespace Graphlib {
template <typename T>
class IsMapType;

template <typename MapType>
class AdjacencyContainer<
    MapType,
    int,
    typename std::enable_if<IsMapType<MapType>::value>::type> {
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
        MapType,
        int,
        typename std::enable_if<IsMapType<MapType>::value>::type>;
    iterator(typename MapType::iterator iter, MapType* map);
    typename MapType::iterator current_;
    MapType* const map_;
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
  MapType map_;
};

template <typename T>
class IsMapType : public std::false_type {};

template <>
class IsMapType<std::unordered_map<int, typename Edge<int>::Weight>>
    : public std::true_type {};

template <>
class IsMapType<std::map<int, typename Edge<int>::Weight>>
    : public std::true_type {};

template <typename T>
using EnableIfIsMap = typename std::enable_if<IsMapType<T>::value>::type;

using AdjacencyHashTable =
    AdjacencyContainer<std::unordered_map<int, typename Edge<int>::Weight>,
                       int>;
using AdjacencyAVLTree =
    AdjacencyContainer<std::map<int, typename Edge<int>::Weight>, int>;

template <typename MapType>
inline AdjacencyContainer<MapType, int, EnableIfIsMap<MapType>>::
    AdjacencyContainer(std::size_t size) {}

template <typename MapType>
inline std::size_t
AdjacencyContainer<MapType, int, EnableIfIsMap<MapType>>::size() const {
  return map_.size();
}

template <typename MapType>
inline bool AdjacencyContainer<MapType, int, EnableIfIsMap<MapType>>::insert(
    int key,
    typename Edge<int>::Weight value) {
  if (exist(key))
    return false;
  map_.emplace(std::make_pair(key, value));
  return true;
}

template <typename MapType>
inline bool AdjacencyContainer<MapType, int, EnableIfIsMap<MapType>>::remove(
    int key) {
  for (auto it = map_.begin(); it != map_.end(); ++it) {
    if (it->first == key) {
      map_.erase(it);
      return true;
    }
  }
  return false;
}

template <typename MapType>
inline bool AdjacencyContainer<MapType, int, EnableIfIsMap<MapType>>::exist(
    int key) const {
  return map_.find(key) != map_.end();
}

template <typename MapType>
inline typename Edge<int>::Weight const&
    AdjacencyContainer<MapType, int, EnableIfIsMap<MapType>>::operator[](
        int key) const {
  return std::find_if(map_.begin(), map_.end(),
                      [key](auto const& pair) { return pair.first == key; })
      ->second;
}

template <typename MapType>
typename Edge<int>::Weight&
    AdjacencyContainer<MapType, int, EnableIfIsMap<MapType>>::operator[](
        int key) {
  return const_cast<Edge<int>::Weight&>(
      static_cast<AdjacencyContainer<
          MapType, int,
          typename std::enable_if<IsMapType<MapType>::value>::type> const&>(
          *this)[key]);
}

template <typename MapType>
inline
    typename AdjacencyContainer<MapType, int, EnableIfIsMap<MapType>>::iterator
    AdjacencyContainer<
        MapType,
        int,
        typename std::enable_if<IsMapType<MapType>::value>::type>::begin() {
  return iterator(map_.begin(), &map_);
}

template <typename MapType>
inline
    typename AdjacencyContainer<MapType, int, EnableIfIsMap<MapType>>::iterator
    AdjacencyContainer<
        MapType,
        int,
        typename std::enable_if<IsMapType<MapType>::value>::type>::end() {
  return iterator(map_.end(), &map_);
}

template <typename MapType>
inline AdjacencyContainer<MapType, int, EnableIfIsMap<MapType>>::iterator::
    iterator(typename MapType::iterator iter, MapType* map)
    : current_(iter), map_(map) {}

template <typename MapType>
inline
    typename AdjacencyContainer<MapType, int, EnableIfIsMap<MapType>>::iterator&
    AdjacencyContainer<
        MapType,
        int,
        typename std::enable_if<IsMapType<MapType>::value>::type>::iterator::
    operator++() {
  if (current_ != map_->end())
    ++current_;
  return *this;
}

template <typename MapType>
inline
    typename AdjacencyContainer<MapType, int, EnableIfIsMap<MapType>>::iterator&
    AdjacencyContainer<MapType, int, EnableIfIsMap<MapType>>::iterator::
    operator--() {
  if (current_ != map_->begin())
    --current_;
  return *this;
}

template <typename MapType>
inline bool AdjacencyContainer<MapType, int, EnableIfIsMap<MapType>>::iterator::
operator==(iterator const& other) const {
  return current_ == other.current_ && map_ == other.map_;
}

template <typename MapType>
inline bool AdjacencyContainer<MapType, int, EnableIfIsMap<MapType>>::iterator::
operator!=(iterator const& other) const {
  return !(*this == other);
}

template <typename MapType>
inline std::pair<int, typename Edge<int>::Weight&>
    AdjacencyContainer<MapType, int, EnableIfIsMap<MapType>>::iterator::
    operator*() {
  if (current_ == map_->end())
    throw std::out_of_range("End iterator is not dereferencable");
  return {current_->first, current_->second};
}

template <typename MapType>
inline std::unique_ptr<std::pair<int, typename Edge<int>::Weight&>>
    AdjacencyContainer<MapType, int, EnableIfIsMap<MapType>>::iterator::
    operator->() {
  return std::make_unique<std::pair<int, typename Edge<int>::Weight&>>(
      operator*());
}

}  // namespace Graphlib
#endif  // ADJACENCYLISTS_ADJACENCY_MAP_HPP_
