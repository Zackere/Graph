// Copyright 2019 Wojciech Replin

#ifndef GRAPHS_EDGE_HPP_
#define GRAPHS_EDGE_HPP_

template <typename T>
struct Edge {
  using Vertex = T;
  using Weight = double;
  Vertex from, to;
  Weight weight;
};

#endif  // GRAPHS_EDGE_HPP_
