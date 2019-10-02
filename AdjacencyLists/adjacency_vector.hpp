// Copyright 2019 Wojciech Replin

#ifndef ADJACENCYLISTS_ADJACENCY_VECTOR_HPP_
#define ADJACENCYLISTS_ADJACENCY_VECTOR_HPP_

#include <algorithm>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "../Graphs/edge.hpp"
#include "../Graphs/graph.hpp"
#include "adjacency_container.hpp"

namespace graphlib {
template <>
class AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex,
    void> {
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
    friend class AdjacencyContainer<
        std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
        graphlib::int_vertex>;
    Iterator(
        graphlib::int_vertex index,
        std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>*
            data);
    graphlib::int_vertex index_;
    std::vector<
        std::optional<typename Edge<graphlib::int_vertex>::Weight>>* const
        data_;
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
  bool InRange(graphlib::int_vertex key) const;
  std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>
      vector_;
  std::size_t nelems_ = 0;
};

using AdjacencyVector = AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>;

inline AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>::AdjacencyContainer(std::size_t size)
    : vector_(size, std::nullopt) {}

inline std::size_t AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>::size() const {
  return nelems_;
}

inline bool AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>::insert(graphlib::int_vertex key,
                                  typename Edge<graphlib::int_vertex>::Weight
                                      value) {
  if (!InRange(key) || vector_[key.value()].has_value())
    return false;
  vector_[key.value()] = value;
  ++nelems_;
  return true;
}

inline bool AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>::remove(graphlib::int_vertex key) {
  if (!InRange(key) || !vector_[key.value()].has_value())
    return false;
  vector_[key.value()].reset();
  --nelems_;
  return true;
}

inline bool AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>::exist(graphlib::int_vertex key) const {
  return InRange(key) && vector_[key.value()].has_value();
}

std::optional<
    std::reference_wrapper<typename Edge<graphlib::int_vertex>::Weight const>>
    AdjacencyContainer<
        std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
        graphlib::int_vertex>::operator[](graphlib::int_vertex key) const {
  if (vector_[key.value()].has_value())
    return std::ref(vector_[key.value()].value());
  return std::nullopt;
}

std::optional<
    std::reference_wrapper<typename Edge<graphlib::int_vertex>::Weight>>
    AdjacencyContainer<
        std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
        graphlib::int_vertex>::operator[](graphlib::int_vertex key) {
  if (vector_[key.value()].has_value())
    return std::ref(vector_[key.value()].value());
  return std::nullopt;
}

inline AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>::Iterator
AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>::begin() {
  return vector_.size() && !vector_[0].has_value()
             ? ++Iterator(graphlib::int_vertex(0), &vector_)
             : Iterator(graphlib::int_vertex(0), &vector_);
}

inline AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>::Iterator
AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>::end() {
  return Iterator(graphlib::int_vertex(vector_.size()), &vector_);
}

inline bool AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>::InRange(graphlib::int_vertex key) const {
  return key.value() >= 0 && key.value() < vector_.size();
}

inline AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>::Iterator::
    Iterator(
        graphlib::int_vertex index,
        std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>*
            data)
    : index_(index), data_(data) {}

inline AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>::Iterator&
AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>::Iterator::operator++() {
  if (index_.value() >= data_->size())
    return *this;
  do {
    ++index_;
  } while (index_.value() < data_->size() &&
           !data_->at(index_.value()).has_value());
  return *this;
}

inline AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>::Iterator&
AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>::Iterator::operator--() {
  graphlib::int_vertex seeker(index_.value() - 1);
  while (seeker.value() >= 0 && !data_->at(seeker.value()).has_value())
    --seeker;
  if (seeker.value() >= 0)
    index_ = seeker;
  return *this;
}

inline bool AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>::Iterator::operator==(Iterator const& other) const {
  return index_ == other.index_ && data_ == other.data_;
}

inline bool AdjacencyContainer<
    std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
    graphlib::int_vertex>::Iterator::operator!=(Iterator const& other) const {
  return !(*this == other);
}

inline std::pair<graphlib::int_vertex,
                 typename Edge<graphlib::int_vertex>::Weight&>
    AdjacencyContainer<
        std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
        graphlib::int_vertex>::Iterator::operator*() {
  _ASSERT_EXPR(index_.value() < data_->size(),
               L"can't dereference out of range AdjacencyVector iterator");
  return {index_, data_->at(index_.value()).value()};
}

inline std::unique_ptr<std::pair<graphlib::int_vertex,
                                 typename Edge<graphlib::int_vertex>::Weight&>>
    AdjacencyContainer<
        std::vector<std::optional<typename Edge<graphlib::int_vertex>::Weight>>,
        graphlib::int_vertex>::Iterator::operator->() {
  return std::make_unique<std::pair<
      graphlib::int_vertex, typename Edge<graphlib::int_vertex>::Weight&>>(
      operator*());
}
}  // namespace graphlib
#endif  // ADJACENCYLISTS_ADJACENCY_VECTOR_HPP_
