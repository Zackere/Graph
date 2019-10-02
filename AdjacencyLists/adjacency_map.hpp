// Copyright 2019 Wojciech Replinstd::unordered_map<int, typename
// Edge<int>::Weight>

#ifndef ADJACENCYLISTS_ADJACENCY_MAP_HPP_
#define ADJACENCYLISTS_ADJACENCY_MAP_HPP_

#include <algorithm>
#include <map>
#include <memory>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include "../Graphs/edge.hpp"
#include "../Graphs/graph.hpp"
#include "../Utils/map_utils.hpp"
#include "./adjacency_container.hpp"

namespace graphlib {

template <typename MapType>
class AdjacencyContainer<MapType,
                         graphlib::int_vertex,
                         util::EnableIfMap<MapType>> {
 public:
  class Iterator {
   public:
    Iterator& operator++();
    Iterator& operator--();
    bool operator==(Iterator const& other) const;
    bool operator!=(Iterator const& other) const;
    std::pair<graphlib::int_vertex,
              typename Edge<graphlib::int_vertex>::Weight&>
    operator*();
    std::unique_ptr<std::pair<graphlib::int_vertex,
                              typename Edge<graphlib::int_vertex>::Weight&>>
    operator->();

   private:
    friend class AdjacencyContainer<MapType,
                                    graphlib::int_vertex,
                                    util::EnableIfMap<MapType>>;
    Iterator(typename MapType::iterator iter, MapType* map);
    typename MapType::iterator current_;
    MapType* const map_;
  };

  explicit AdjacencyContainer(std::size_t size);
  std::size_t size() const;
  bool insert(graphlib::int_vertex key,
              typename Edge<graphlib::int_vertex>::Weight value);
  bool remove(graphlib::int_vertex key);
  bool exist(graphlib::int_vertex key) const;
  std::optional<
      std::reference_wrapper<typename Edge<graphlib::int_vertex>::Weight const>>
  operator[](graphlib::int_vertex key) const;
  std::optional<
      std::reference_wrapper<typename Edge<graphlib::int_vertex>::Weight>>
  operator[](graphlib::int_vertex key);
  Iterator begin();
  Iterator end();

 private:
  MapType map_;
};

using AdjacencyHashTable = AdjacencyContainer<
    std::unordered_map<graphlib::int_vertex,
                       typename Edge<graphlib::int_vertex>::Weight>,
    graphlib::int_vertex>;

using AdjacencyAVLTree = AdjacencyContainer<
    std::map<graphlib::int_vertex, typename Edge<graphlib::int_vertex>::Weight>,
    graphlib::int_vertex>;

template <typename MapType>
inline AdjacencyContainer<
    MapType,
    graphlib::int_vertex,
    util::EnableIfMap<MapType>>::AdjacencyContainer(std::size_t size) {}

template <typename MapType>
inline std::size_t AdjacencyContainer<MapType,
                                      graphlib::int_vertex,
                                      util::EnableIfMap<MapType>>::size()
    const {
  return map_.size();
}

template <typename MapType>
inline bool
AdjacencyContainer<MapType, graphlib::int_vertex, util::EnableIfMap<MapType>>::
    insert(graphlib::int_vertex key,
           typename Edge<graphlib::int_vertex>::Weight value) {
  if (exist(key))
    return false;
  map_.emplace(std::make_pair(key, value));
  return true;
}

template <typename MapType>
inline bool
AdjacencyContainer<MapType, graphlib::int_vertex, util::EnableIfMap<MapType>>::
    remove(graphlib::int_vertex key) {
  for (auto it = map_.begin(); it != map_.end(); ++it) {
    if (it->first == key) {
      map_.erase(it);
      return true;
    }
  }
  return false;
}

template <typename MapType>
inline bool
AdjacencyContainer<MapType, graphlib::int_vertex, util::EnableIfMap<MapType>>::
    exist(graphlib::int_vertex key) const {
  return map_.find(key) != map_.end();
}

template <typename MapType>
std::optional<
    std::reference_wrapper<typename Edge<graphlib::int_vertex>::Weight const>>
    AdjacencyContainer<MapType,
                       graphlib::int_vertex,
                       util::EnableIfMap<MapType>>::
    operator[](graphlib::int_vertex key) const {
  auto it = std::find_if(map_.cbegin(), map_.cend(),
                         [key](auto const& pair) { return pair.first == key; });
  if (it != map_.end())
    return std::ref(it->second);
  return std::nullopt;
}

template <typename MapType>
std::optional<
    std::reference_wrapper<typename Edge<graphlib::int_vertex>::Weight>>
    AdjacencyContainer<MapType,
                       graphlib::int_vertex,
                       util::EnableIfMap<MapType>>::
    operator[](graphlib::int_vertex key) {
  auto it = std::find_if(map_.begin(), map_.end(),
                         [key](auto const& pair) { return pair.first == key; });
  if (it != map_.end())
    return std::ref(it->second);
  return std::nullopt;
}

template <typename MapType>
inline typename AdjacencyContainer<MapType,
                                   graphlib::int_vertex,
                                   util::EnableIfMap<MapType>>::Iterator
AdjacencyContainer<MapType, graphlib::int_vertex, util::EnableIfMap<MapType>>::
    begin() {
  return Iterator(map_.begin(), &map_);
}

template <typename MapType>
inline typename AdjacencyContainer<MapType,
                                   graphlib::int_vertex,
                                   util::EnableIfMap<MapType>>::Iterator
AdjacencyContainer<MapType, graphlib::int_vertex, util::EnableIfMap<MapType>>::
    end() {
  return Iterator(map_.end(), &map_);
}

template <typename MapType>
inline AdjacencyContainer<MapType,
                          graphlib::int_vertex,
                          util::EnableIfMap<MapType>>::Iterator::
    Iterator(typename MapType::iterator iter, MapType* map)
    : current_(iter), map_(map) {}

template <typename MapType>
inline typename AdjacencyContainer<MapType,
                                   graphlib::int_vertex,
                                   util::EnableIfMap<MapType>>::Iterator&
AdjacencyContainer<MapType, graphlib::int_vertex, util::EnableIfMap<MapType>>::
    Iterator::operator++() {
  if (current_ != map_->end())
    ++current_;
  return *this;
}

template <typename MapType>
inline typename AdjacencyContainer<MapType,
                                   graphlib::int_vertex,
                                   util::EnableIfMap<MapType>>::Iterator&
AdjacencyContainer<MapType, graphlib::int_vertex, util::EnableIfMap<MapType>>::
    Iterator::operator--() {
  if (current_ != map_->begin())
    --current_;
  return *this;
}

template <typename MapType>
inline bool
AdjacencyContainer<MapType, graphlib::int_vertex, util::EnableIfMap<MapType>>::
    Iterator::operator==(Iterator const& other) const {
  return current_ == other.current_ && map_ == other.map_;
}

template <typename MapType>
inline bool
AdjacencyContainer<MapType, graphlib::int_vertex, util::EnableIfMap<MapType>>::
    Iterator::operator!=(Iterator const& other) const {
  return !(*this == other);
}

template <typename MapType>
inline std::pair<graphlib::int_vertex,
                 typename Edge<graphlib::int_vertex>::Weight&>
    AdjacencyContainer<MapType,
                       graphlib::int_vertex,
                       util::EnableIfMap<MapType>>::Iterator::operator*() {
  return {current_->first, current_->second};
}

template <typename MapType>
inline std::unique_ptr<std::pair<graphlib::int_vertex,
                                 typename Edge<graphlib::int_vertex>::Weight&>>
    AdjacencyContainer<MapType,
                       graphlib::int_vertex,
                       util::EnableIfMap<MapType>>::Iterator::operator->() {
  return std::make_unique<std::pair<
      graphlib::int_vertex, typename Edge<graphlib::int_vertex>::Weight&>>(
      operator*());
}

}  // namespace graphlib
#endif  // ADJACENCYLISTS_ADJACENCY_MAP_HPP_
