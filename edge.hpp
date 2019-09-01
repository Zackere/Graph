// Copyright 2019 Wojciech Replin

#ifndef EDGE_HPP_
#define EDGE_HPP_

template <typename T>
struct Edge {
  using Vertex = T;
  using Weight = double;
  Vertex from, to;
  Weight weight;
};

#endif  // EDGE_HPP_
