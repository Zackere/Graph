// Copyright 2019 Wojciech Replin

#ifndef ADJACENCYLISTS_ADJACENCY_LIST_HPP_
#define ADJACENCYLISTS_ADJACENCY_LIST_HPP_

#include <algorithm>
#include <list>
#include <map>
#include <memory>
#include <type_traits>
#include <utility>

#include "../Graphs/edge.hpp"
#include "../Graphs/graph.hpp"
#include "../Utils/list_utils.hpp"
#include "adjacency_container.hpp"

namespace graphlib {

template <typename ListType>
class AdjacencyContainer<ListType,
                         graphlib::int_vertex,
                         util::EnableIfList<ListType>> {
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
    friend class AdjacencyContainer<ListType,
                                    graphlib::int_vertex,
                                    util::EnableIfList<ListType>>;
    Iterator(typename ListType::iterator iter, ListType* map);
    typename ListType::iterator current_;
    ListType* const list_;
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
  ListType list_;
};

using AdjacencyList = AdjacencyContainer<
    std::list<std::pair<graphlib::int_vertex,
                        typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>;

template <typename ListType>
AdjacencyContainer<ListType,
                   graphlib::int_vertex,
                   util::EnableIfList<ListType>>::AdjacencyContainer(std::size_t
                                                                         size) {
}

template <typename ListType>
inline std::size_t AdjacencyContainer<ListType,
                                      graphlib::int_vertex,
                                      util::EnableIfList<ListType>>::size()
    const {
  return list_.size();
}

template <typename ListType>
inline bool AdjacencyContainer<ListType,
                               graphlib::int_vertex,
                               util::EnableIfList<ListType>>::
    insert(graphlib::int_vertex key,
           typename Edge<graphlib::int_vertex>::Weight value) {
  if (exist(key))
    return false;
  list_.emplace_back(key, value);
  return true;
}

template <typename ListType>
inline bool AdjacencyContainer<
    ListType,
    graphlib::int_vertex,
    util::EnableIfList<ListType>>::remove(graphlib::int_vertex key) {
  for (auto it = list_.begin(); it != list_.end(); ++it) {
    if (it->first == key) {
      list_.erase(it);
      return true;
    }
  }
  return false;
}

template <typename ListType>
inline bool AdjacencyContainer<
    ListType,
    graphlib::int_vertex,
    util::EnableIfList<ListType>>::exist(graphlib::int_vertex key) const {
  return std::find_if(list_.begin(), list_.end(), [key](auto const& pair) {
           return pair.first == key;
         }) != list_.end();
}

template <typename ListType>
std::optional<
    std::reference_wrapper<typename Edge<graphlib::int_vertex>::Weight const>>
    AdjacencyContainer<ListType,
                       graphlib::int_vertex,
                       util::EnableIfList<ListType>>::
    operator[](graphlib::int_vertex key) const {
  auto it = std::find_if(list_.cbegin(), list_.cend(),
                         [key](auto const& pair) { return pair.first == key; });
  if (it != list_.end())
    return std::ref(it->second);
  return std::nullopt;
}

template <typename ListType>
std::optional<
    std::reference_wrapper<typename Edge<graphlib::int_vertex>::Weight>>
    AdjacencyContainer<ListType,
                       graphlib::int_vertex,
                       util::EnableIfList<ListType>>::
    operator[](graphlib::int_vertex key) {
  auto it = std::find_if(list_.begin(), list_.end(),
                         [key](auto const& pair) { return pair.first == key; });
  if (it != list_.end())
    return std::ref(it->second);
  return std::nullopt;
}

template <typename ListType>
inline typename AdjacencyContainer<ListType,
                                   graphlib::int_vertex,
                                   util::EnableIfList<ListType>>::Iterator
AdjacencyContainer<ListType,
                   graphlib::int_vertex,
                   util::EnableIfList<ListType>>::begin() {
  return Iterator(list_.begin(), &list_);
}

template <typename ListType>
inline typename AdjacencyContainer<ListType,
                                   graphlib::int_vertex,
                                   util::EnableIfList<ListType>>::Iterator
AdjacencyContainer<ListType,
                   graphlib::int_vertex,
                   util::EnableIfList<ListType>>::end() {
  return Iterator(list_.end(), &list_);
}

template <typename ListType>
inline AdjacencyContainer<ListType,
                          graphlib::int_vertex,
                          util::EnableIfList<ListType>>::Iterator::
    Iterator(typename ListType::iterator iter, ListType* list)
    : current_(iter), list_(list) {}

template <typename ListType>
inline typename AdjacencyContainer<ListType,
                                   graphlib::int_vertex,
                                   util::EnableIfList<ListType>>::Iterator&
AdjacencyContainer<ListType,
                   graphlib::int_vertex,
                   util::EnableIfList<ListType>>::Iterator::operator++() {
  if (current_ != list_->end())
    ++current_;
  return *this;
}

template <typename ListType>
inline typename AdjacencyContainer<ListType,
                                   graphlib::int_vertex,
                                   util::EnableIfList<ListType>>::Iterator&
AdjacencyContainer<ListType,
                   graphlib::int_vertex,
                   util::EnableIfList<ListType>>::Iterator::operator--() {
  if (current_ != list_->begin())
    --current_;
  return *this;
}

template <typename ListType>
inline bool AdjacencyContainer<ListType,
                               graphlib::int_vertex,
                               util::EnableIfList<ListType>>::Iterator::
operator==(Iterator const& other) const {
  return current_ == other.current_ && list_ == other.list_;
}

template <typename ListType>
inline bool AdjacencyContainer<ListType,
                               graphlib::int_vertex,
                               util::EnableIfList<ListType>>::Iterator::
operator!=(Iterator const& other) const {
  return !(*this == other);
}

template <typename ListType>
inline typename std::pair<graphlib::int_vertex,
                          typename Edge<graphlib::int_vertex>::Weight&>
    AdjacencyContainer<ListType,
                       graphlib::int_vertex,
                       util::EnableIfList<ListType>>::Iterator::operator*() {
  return {current_->first, current_->second};
}

template <typename ListType>
inline std::unique_ptr<std::pair<graphlib::int_vertex,
                                 typename Edge<graphlib::int_vertex>::Weight&>>
    AdjacencyContainer<ListType,
                       graphlib::int_vertex,
                       util::EnableIfList<ListType>>::Iterator::operator->() {
  return std::make_unique<ListType::value_type>(operator*());
}

}  // namespace graphlib
#endif  // ADJACENCYLISTS_ADJACENCY_LIST_HPP_
