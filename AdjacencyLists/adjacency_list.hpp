// Copyright 2019 Wojciech Replin

#ifndef ADJACENCYLISTS_ADJACENCY_LIST_HPP_
#define ADJACENCYLISTS_ADJACENCY_LIST_HPP_

#include <algorithm>
#include <list>
#include <memory>
#include <optional>
#include <utility>

#include "../Graphs/edge.hpp"
#include "adjacency_container.hpp"

namespace Graphlib {
template <>
class AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                         int> {
 public:
  class Iterator {
   public:
    Iterator& operator++();
    Iterator& operator--();
    bool operator==(Iterator const& other) const;
    bool operator!=(Iterator const& other) const;
    std::pair<int, typename Edge<int>::Weight&> operator*();
    std::unique_ptr<std::pair<int, typename Edge<int>::Weight&>> operator->();

   private:
    friend class AdjacencyContainer<
        std::list<std::pair<int, typename Edge<int>::Weight>>,
        int>;
    Iterator(
        std::list<std::pair<int, typename Edge<int>::Weight>>::iterator iter,
        std::list<std::pair<int, typename Edge<int>::Weight>>* list);
    std::list<std::pair<int, typename Edge<int>::Weight>>::iterator current_;
    std::list<std::pair<int, typename Edge<int>::Weight>>* const list_;
  };

  explicit AdjacencyContainer(std::size_t size);
  std::size_t size() const;
  bool insert(int key, typename Edge<int>::Weight value);
  bool remove(int key);
  bool exist(int key) const;
  std::optional<std::reference_wrapper<typename Edge<int>::Weight const>>
  operator[](int key) const;
  std::optional<std::reference_wrapper<typename Edge<int>::Weight>> operator[](
      int key);
  Iterator begin();
  Iterator end();

 private:
  std::list<std::pair<int, typename Edge<int>::Weight>> list_;
};

using AdjacencyList =
    AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                       int>;

inline AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                          int>::AdjacencyContainer(std::size_t size) {}

inline std::size_t
AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                   int>::size() const {
  return list_.size();
}

inline bool
AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                   int>::insert(int key, typename Edge<int>::Weight value) {
  if (exist(key))
    return false;
  list_.emplace_back(key, value);
  return true;
}

inline bool
AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                   int>::remove(int key) {
  for (auto it = list_.begin(); it != list_.end(); ++it) {
    if (it->first == key) {
      list_.erase(it);
      return true;
    }
  }
  return false;
}

inline bool
AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                   int>::exist(int key) const {
  return std::find_if(list_.begin(), list_.end(), [key](auto const& pair) {
           return pair.first == key;
         }) != list_.end();
}

std::optional<std::reference_wrapper<typename Edge<int>::Weight const>>
    AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                       int>::operator[](int key) const {
  auto it = std::find_if(list_.cbegin(), list_.cend(),
                         [key](auto const& pair) { return pair.first == key; });
  if (it != list_.end())
    return std::ref(it->second);
  return std::nullopt;
}

std::optional<std::reference_wrapper<typename Edge<int>::Weight>>
    AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                       int>::operator[](int key) {
  auto it = std::find_if(list_.begin(), list_.end(),
                         [key](auto const& pair) { return pair.first == key; });
  if (it != list_.end())
    return std::ref(it->second);
  return std::nullopt;
}

inline AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                          int>::Iterator
AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                   int>::begin() {
  return Iterator(list_.begin(), &list_);
}

inline AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                          int>::Iterator
AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                   int>::end() {
  return Iterator(list_.end(), &list_);
}

inline AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                          int>::Iterator::
    Iterator(
        std::list<std::pair<int, typename Edge<int>::Weight>>::iterator iter,
        std::list<std::pair<int, typename Edge<int>::Weight>>* list)
    : current_(iter), list_(list) {}

inline AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                          int>::Iterator&
AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                   int>::Iterator::operator++() {
  if (current_ != list_->end())
    ++current_;
  return *this;
}

inline AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                          int>::Iterator&
AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                   int>::Iterator::operator--() {
  if (current_ != list_->begin())
    --current_;
  return *this;
}

inline bool
AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                   int>::Iterator::operator==(Iterator const& other) const {
  return current_ == other.current_ && list_ == other.list_;
}

inline bool
AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                   int>::Iterator::operator!=(Iterator const& other) const {
  return !(*this == other);
}

inline std::pair<int, typename Edge<int>::Weight&>
    AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                       int>::Iterator::operator*() {
  if (current_ == list_->end())
    throw std::out_of_range("End Iterator is not dereferencable");
  return {current_->first, current_->second};
}

inline std::unique_ptr<std::pair<int, typename Edge<int>::Weight&>>
    AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                       int>::Iterator::operator->() {
  return std::make_unique<std::pair<int, typename Edge<int>::Weight&>>(
      operator*());
}

}  // namespace Graphlib
#endif  // ADJACENCYLISTS_ADJACENCY_LIST_HPP_
