#ifndef GRAPHLIB_GRAPHS_ADJACENCY_MATRIX_GRAPH_HPP_
#define GRAPHLIB_GRAPHS_ADJACENCY_MATRIX_GRAPH_HPP_

#include "graph.hpp"

#include <initializer_list>
#include <unordered_map>
#include <vector>

namespace Graphlib {
template <typename Vertex>
class AdjacencyMatrixGraph : public Graph<Vertex> {
 public:
  AdjacencyMatrixGraph(bool directed,
                       std::initializer_list<Vertex> const& verticies);

  typename Graph<Vertex>::VertexIterator Vbegin() const override;
  typename Graph<Vertex>::VertexIterator Vend() const override;

 private:
  class VertexIteratorAM : public Graph<Vertex>::VertexIteratorBase {
   public:
    using Base = typename Graph<Vertex>::VertexIteratorBase;
    using iterator_type =
        typename std::unordered_map<std::size_t, Vertex>::const_iterator;

    explicit VertexIteratorAM(
        typename std::unordered_map<std::size_t, Vertex>::const_iterator
            vertex);

    Base const* next();
    Base const* prev();
    bool equals(Base const* other) const;
    Vertex const& operator*() const;
    Vertex const* operator->() const;

   private:
    iterator_type vertex_;
  };

 private:
  // Mapping verticies of arbitrary type to easily managable ints.
  std::unordered_map<std::size_t, Vertex> verticies_;
  std::vector<std::vector<std::pair<bool, double>>> adjacency_matrix_;
  bool directed_;
};

template <typename Vertex>
inline AdjacencyMatrixGraph<Vertex>::AdjacencyMatrixGraph(
    bool directed,
    std::initializer_list<Vertex> const& verticies)
    : directed_(directed) {
  verticies_.reserve(verticies.size());
  auto i = static_cast<std::size_t>(-1);
  for (auto const& v : verticies)
    verticies_.emplace(std::make_pair(++i, v));
  for (auto i = 0u; i < verticies_.size(); ++i)
    adjacency_matrix_.emplace_back(verticies_.size(),
                                   std::make_pair(false, 0.0f));
}

template <typename Vertex>
typename Graph<Vertex>::VertexIterator AdjacencyMatrixGraph<Vertex>::Vbegin()
    const {
  return Graph<Vertex>::VertexIterator(
      std::make_shared<VertexIteratorAM>(verticies_.cbegin()));
}

template <typename Vertex>
typename Graph<Vertex>::VertexIterator AdjacencyMatrixGraph<Vertex>::Vend()
    const {
  return Graph<Vertex>::VertexIterator(
      std::make_shared<VertexIteratorAM>(verticies_.cend()));
}

template <typename Vertex>
AdjacencyMatrixGraph<Vertex>::VertexIteratorAM::VertexIteratorAM(
    typename std::unordered_map<std::size_t, Vertex>::const_iterator vertex)
    : vertex_(vertex) {}

template <typename Vertex>
typename AdjacencyMatrixGraph<Vertex>::VertexIteratorAM::Base const*
AdjacencyMatrixGraph<Vertex>::VertexIteratorAM::next() {
  ++vertex_;
  return this;
}

template <typename Vertex>
typename AdjacencyMatrixGraph<Vertex>::VertexIteratorAM::Base const*
AdjacencyMatrixGraph<Vertex>::VertexIteratorAM::prev() {
  --vertex_;
  return this;
}

template <typename Vertex>
bool AdjacencyMatrixGraph<Vertex>::VertexIteratorAM::equals(
    Base const* other) const {
  // TODO: find better solution than dynamic_cast
  auto const* other_ptr = dynamic_cast<VertexIteratorAM const*>(other);
  return other_ptr && vertex_ == other_ptr->vertex_;
}

template <typename Vertex>
Vertex const& AdjacencyMatrixGraph<Vertex>::VertexIteratorAM::operator*()
    const {
  return vertex_->second;
}

template <typename Vertex>
Vertex const* AdjacencyMatrixGraph<Vertex>::VertexIteratorAM::operator->()
    const {
  return &vertex_->second;
}

}  // namespace Graphlib

#endif  // !GRAPHLIB_GRAPHS_ADJACENCY_MATRIX_GRAPH_HPP_