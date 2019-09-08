// Copyright 2019 Wojciech Replin

#ifndef ADJACENCYLISTS_ADJACENCY_VECTOR_HPP_
#define ADJACENCYLISTS_ADJACENCY_VECTOR_HPP_

#include <algorithm>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "../edge.hpp"
#include "adjacency_container.hpp"

namespace Graphlib {
template <>
class AdjacencyContainer<std::vector<std::optional<typename Edge<int>::Weight>>,
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
        std::vector<std::optional<typename Edge<int>::Weight>>,
        int>;
    Iterator(int index,
             std::vector<std::optional<typename Edge<int>::Weight>>* data);
    int index_ = 0;
    std::vector<std::optional<typename Edge<int>::Weight>>* const data_;
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
  bool InRange(int key) const;
  std::vector<std::optional<typename Edge<int>::Weight>> vector_;
  std::size_t nelems_ = 0;
};

using AdjacencyVector =
    AdjacencyContainer<std::vector<std::optional<typename Edge<int>::Weight>>,
                       int>;

inline AdjacencyContainer<
    std::vector<std::optional<typename Edge<int>::Weight>>,
    int>::AdjacencyContainer(std::size_t size)
    : vector_(size, std::nullopt) {}

inline std::size_t
AdjacencyContainer<std::vector<std::optional<typename Edge<int>::Weight>>,
                   int>::size() const {
  return nelems_;
}

inline bool
AdjacencyContainer<std::vector<std::optional<typename Edge<int>::Weight>>,
                   int>::insert(int key, typename Edge<int>::Weight value) {
  if (!InRange(key) || vector_[key].has_value())
    return false;
  vector_[key] = value;
  ++nelems_;
  return true;
}

inline bool
AdjacencyContainer<std::vector<std::optional<typename Edge<int>::Weight>>,
                   int>::remove(int key) {
  if (!InRange(key) || !vector_[key].has_value())
    return false;
  vector_[key].reset();
  --nelems_;
  return true;
}

inline bool
AdjacencyContainer<std::vector<std::optional<typename Edge<int>::Weight>>,
                   int>::exist(int key) const {
  return InRange(key) && vector_[key].has_value();
}

std::optional<std::reference_wrapper<typename Edge<int>::Weight const>>
    AdjacencyContainer<std::vector<std::optional<typename Edge<int>::Weight>>,
                       int>::operator[](int key) const {
  if (vector_[key].has_value())
    return std::ref(vector_[key].value());
  return std::nullopt;
}

std::optional<std::reference_wrapper<typename Edge<int>::Weight>>
    AdjacencyContainer<std::vector<std::optional<typename Edge<int>::Weight>>,
                       int>::operator[](int key) {
  if (vector_[key].has_value())
    return std::ref(vector_[key].value());
  return std::nullopt;
}

inline AdjacencyContainer<
    std::vector<std::optional<typename Edge<int>::Weight>>,
    int>::Iterator
AdjacencyContainer<std::vector<std::optional<typename Edge<int>::Weight>>,
                   int>::begin() {
  return vector_.size() && !vector_[0].has_value() ? ++Iterator(0, &vector_)
                                                   : Iterator(0, &vector_);
}

inline AdjacencyContainer<
    std::vector<std::optional<typename Edge<int>::Weight>>,
    int>::Iterator
AdjacencyContainer<std::vector<std::optional<typename Edge<int>::Weight>>,
                   int>::end() {
  return Iterator(vector_.size(), &vector_);
}

inline bool
AdjacencyContainer<std::vector<std::optional<typename Edge<int>::Weight>>,
                   int>::InRange(int key) const {
  return key >= 0 && key < vector_.size();
}

inline AdjacencyContainer<
    std::vector<std::optional<typename Edge<int>::Weight>>,
    int>::Iterator::
    Iterator(int index,
             std::vector<std::optional<typename Edge<int>::Weight>>* data)
    : index_(index), data_(data) {}

inline AdjacencyContainer<
    std::vector<std::optional<typename Edge<int>::Weight>>,
    int>::Iterator&
AdjacencyContainer<std::vector<std::optional<typename Edge<int>::Weight>>,
                   int>::Iterator::operator++() {
  if (index_ >= data_->size())
    return *this;
  do {
    ++index_;
  } while (index_ < data_->size() && !data_->at(index_).has_value());
  return *this;
}

inline AdjacencyContainer<
    std::vector<std::optional<typename Edge<int>::Weight>>,
    int>::Iterator&
AdjacencyContainer<std::vector<std::optional<typename Edge<int>::Weight>>,
                   int>::Iterator::operator--() {
  int seeker = index_ - 1;
  while (seeker >= 0 && !data_->at(seeker).has_value())
    --seeker;
  if (seeker >= 0)
    index_ = seeker;
  return *this;
}

inline bool
AdjacencyContainer<std::vector<std::optional<typename Edge<int>::Weight>>,
                   int>::Iterator::operator==(Iterator const& other) const {
  return index_ == other.index_ && data_ == other.data_;
}

inline bool
AdjacencyContainer<std::vector<std::optional<typename Edge<int>::Weight>>,
                   int>::Iterator::operator!=(Iterator const& other) const {
  return !(*this == other);
}

inline std::pair<int, typename Edge<int>::Weight&>
    AdjacencyContainer<std::vector<std::optional<typename Edge<int>::Weight>>,
                       int>::Iterator::operator*() {
  if (this->index_ == data_->size())
    throw std::out_of_range("End Iterator is not dereferencable");
  return {index_, data_->at(index_).value()};
}

inline std::unique_ptr<std::pair<int, typename Edge<int>::Weight&>>
    AdjacencyContainer<std::vector<std::optional<typename Edge<int>::Weight>>,
                       int>::Iterator::operator->() {
  return std::make_unique<std::pair<int, typename Edge<int>::Weight&>>(
      operator*());
}
}  // namespace Graphlib
#endif  // ADJACENCYLISTS_ADJACENCY_VECTOR_HPP_
