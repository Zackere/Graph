#ifndef GRAPHLIB_GRAPHS_GRAPH_HPP_
#define GRAPHLIB_GRAPHS_GRAPH_HPP_

#include <memory>

namespace Graphlib {
template <typename Vertex>
class Graph {
 public:
  struct Edge {
    Vertex beg, end;
    double weight;
  };
  class VertexIterator {
   public:
    explicit VertexIterator(
        std::unique_ptr<typename Graph<Vertex>::VertexIteratorBase> iterator);
    VertexIterator(VertexIterator const& other);

    VertexIterator& operator++();
    VertexIterator& operator--();
    bool operator!=(VertexIterator const& other) const;
    bool operator==(VertexIterator const& other) const;
    Vertex const& operator*() const;
    Vertex const* operator->() const;

   private:
    std::unique_ptr<typename Graph<Vertex>::VertexIteratorBase> iterator_;
  };

  virtual ~Graph() = default;

  virtual VertexIterator Vbegin() const = 0;
  virtual VertexIterator Vend() const = 0;

 protected:
  class VertexIteratorBase {
   public:
    virtual ~VertexIteratorBase() = default;
    virtual VertexIteratorBase const* next() = 0;
    virtual VertexIteratorBase const* prev() = 0;
    virtual bool equals(VertexIteratorBase const* other) const = 0;
    virtual Vertex const& operator*() const = 0;
    virtual Vertex const* operator->() const = 0;
    virtual std::unique_ptr<VertexIteratorBase> clone() const = 0;
  };
};

template <typename Vertex>
Graph<Vertex>::VertexIterator::VertexIterator(
    std::unique_ptr<typename Graph<Vertex>::VertexIteratorBase> iterator)
    : iterator_(std::move(iterator)) {}

template <typename Vertex>
Graph<Vertex>::VertexIterator::VertexIterator(VertexIterator const& other)
    : iterator_(other.iterator_->clone()) {}

template <typename Vertex>
typename Graph<Vertex>::VertexIterator& Graph<Vertex>::VertexIterator::
operator++() {
  iterator_->next();
  return *this;
}

template <typename Vertex>
typename Graph<Vertex>::VertexIterator& Graph<Vertex>::VertexIterator::
operator--() {
  iterator_->prev();
  return *this;
}

template <typename Vertex>
bool Graph<Vertex>::VertexIterator::operator!=(
    VertexIterator const& other) const {
  return !(*this == other);
}

template <typename Vertex>
bool Graph<Vertex>::VertexIterator::operator==(
    VertexIterator const& other) const {
  return iterator_->equals(other.iterator_.get());
}

template <typename Vertex>
Vertex const& Graph<Vertex>::VertexIterator::operator*() const {
  return iterator_->operator*();
}

template <typename Vertex>
Vertex const* Graph<Vertex>::VertexIterator::operator->() const {
  return iterator_->operator->();
}

}  // namespace Graphlib

#endif  // !GRAPHLIB_GRAPHS_GRAPH_HPP_
