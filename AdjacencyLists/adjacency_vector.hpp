// Copyright 2019 Wojciech Replin

#ifndef ADJACENCYLISTS_ADJACENCY_VECTOR_HPP_
#define ADJACENCYLISTS_ADJACENCY_VECTOR_HPP_

#include <algorithm>
#include <iterator>
#include <memory>
#include <utility>
#include <vector>

#include "../edge.hpp"
#include "adjacency_container.hpp"

namespace Graphlib {
template <>
class AdjacencyContainer<
    std::vector<std::pair<bool, typename Edge<int>::Weight>>,
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
        std::vector<std::pair<bool, typename Edge<int>::Weight>>,
        int>;
    iterator(int index,
             std::vector<std::pair<bool, typename Edge<int>::Weight>>* data);
    int index_ = 0;
    std::vector<std::pair<bool, typename Edge<int>::Weight>>* const data_;
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
  bool InRange(int key) const;
  std::vector<std::pair<bool, typename Edge<int>::Weight>> vector_;
  std::size_t nelems_ = 0;
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
  return nelems_;
}

inline bool
AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                   int>::insert(int key, typename Edge<int>::Weight value) {
  if (!InRange(key) || vector_[key].first)
    return false;
  vector_[key] = std::make_pair(true, value);
  ++nelems_;
  return true;
}

inline bool
AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                   int>::remove(int key) {
  if (!InRange(key) || !vector_[key].first)
    return false;
  vector_[key].first = false;
  --nelems_;
  return true;
}

inline bool
AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                   int>::exist(int key) const {
  return InRange(key) && vector_[key].first;
}

inline typename Edge<int>::Weight const&
    AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                       int>::operator[](int key) const {
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

inline AdjacencyContainer<
    std::vector<std::pair<bool, typename Edge<int>::Weight>>,
    int>::iterator
AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                   int>::begin() {
  return vector_.size() && !vector_[0].first ? ++iterator(0, &vector_)
                                             : iterator(0, &vector_);
}

inline AdjacencyContainer<
    std::vector<std::pair<bool, typename Edge<int>::Weight>>,
    int>::iterator
AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                   int>::end() {
  return iterator(vector_.size(), &vector_);
}

inline bool
AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                   int>::InRange(int key) const {
  return key >= 0 && key < vector_.size();
}

inline AdjacencyContainer<
    std::vector<std::pair<bool, typename Edge<int>::Weight>>,
    int>::iterator::iterator(int index,
                             std::vector<std::pair<bool, double>>* data)
    : index_(index), data_(data) {}

inline AdjacencyContainer<
    std::vector<std::pair<bool, typename Edge<int>::Weight>>,
    int>::iterator&
AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                   int>::iterator::operator++() {
  if (index_ >= data_->size())
    return *this;
  do {
    ++index_;
  } while (index_ < data_->size() && !data_->at(index_).first);
  return *this;
}

inline AdjacencyContainer<
    std::vector<std::pair<bool, typename Edge<int>::Weight>>,
    int>::iterator&
AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                   int>::iterator::operator--() {
  int seeker = index_ - 1;
  while (seeker >= 0 && !data_->at(seeker).first)
    --seeker;
  if (seeker >= 0)
    index_ = seeker;
  return *this;
}

inline bool
AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                   int>::iterator::operator==(iterator const& other) const {
  return index_ == other.index_ && data_ == other.data_;
}

inline bool
AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                   int>::iterator::operator!=(iterator const& other) const {
  return !(*this == other);
}

inline std::pair<int, typename Edge<int>::Weight&>
    AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                       int>::iterator::operator*() {
  if (this->index_ == data_->size())
    throw std::out_of_range("End iterator is not dereferencable");
  return {index_, data_->at(index_).second};
}

inline std::unique_ptr<std::pair<int, typename Edge<int>::Weight&>>
    AdjacencyContainer<std::vector<std::pair<bool, typename Edge<int>::Weight>>,
                       int>::iterator::operator->() {
  return std::make_unique<std::pair<int, typename Edge<int>::Weight&>>(
      operator*());
}
}  // namespace Graphlib
#endif  // ADJACENCYLISTS_ADJACENCY_VECTOR_HPP_
