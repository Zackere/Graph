// Copyright 2019 Wojciech Replin

#ifndef ADJACENCYLISTS_ADJACENCY_LIST_HPP_
#define ADJACENCYLISTS_ADJACENCY_LIST_HPP_

#include <algorithm>
#include <iterator>
#include <list>
#include <utility>

#include "../edge.hpp"
#include "adjacency_container.hpp"

namespace Graphlib {
template <>
class AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                         int> {
 public:
  class iterator {
   public:
    iterator& operator++();
    iterator& operator--();
    bool operator==(iterator const& other) const;
    bool operator!=(iterator const& other) const;
    std::pair<int, typename Edge<int>::Weight&> operator*();

   private:
    friend class AdjacencyContainer<
        std::list<std::pair<int, typename Edge<int>::Weight>>,
        int>;
    iterator(
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
  typename Edge<int>::Weight const& operator[](int key) const;
  typename Edge<int>::Weight& operator[](int key);
  iterator begin();
  iterator end();

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

inline typename Edge<int>::Weight const&
    AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                       int>::operator[](int key) const {
  return std::find_if(list_.begin(), list_.end(),
                       [key](auto const& pair) { return pair.first == key; })->second;
}

typename Edge<int>::Weight&
    AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                       int>::operator[](int key) {
  return const_cast<Edge<int>::Weight&>(
      static_cast<AdjacencyContainer<
          std::list<std::pair<int, typename Edge<int>::Weight>>, int> const&>(
          *this)[key]);
}

inline AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                          int>::iterator
AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                   int>::begin() {
  return iterator(list_.begin(), &list_);
}

inline AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                          int>::iterator
AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                   int>::end() {
  return iterator(list_.end(), &list_);
}

inline AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                          int>::iterator::
    iterator(
        std::list<std::pair<int, typename Edge<int>::Weight>>::iterator iter,
        std::list<std::pair<int, typename Edge<int>::Weight>>* list)
    : current_(iter), list_(list) {}

inline AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                          int>::iterator&
AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                   int>::iterator::operator++() {
  if (current_ != list_->end())
    ++current_;
  return *this;
}

inline AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                          int>::iterator&
AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                   int>::iterator::operator--() {
  if (current_ != list_->begin())
    --current_;
  return *this;
}

inline bool
AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                   int>::iterator::operator==(iterator const& other) const {
  return current_ == other.current_ && list_ == other.list_;
}

inline bool
AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                   int>::iterator::operator!=(iterator const& other) const {
  return !(*this == other);
}

inline std::pair<int, typename Edge<int>::Weight&>
    AdjacencyContainer<std::list<std::pair<int, typename Edge<int>::Weight>>,
                       int>::iterator::operator*() {
  if (current_ == list_->end())
    throw std::out_of_range("End iterator is not dereferencable");
  return {current_->first, current_->second};
}

}  // namespace Graphlib
#endif  // ADJACENCYLISTS_ADJACENCY_LIST_HPP_
