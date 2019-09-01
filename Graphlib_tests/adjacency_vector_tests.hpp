// Copyright 2019 Wojciech Replin

#ifndef GRAPHLIB_TESTS_ADJACENCY_VECTOR_TESTS_HPP_
#define GRAPHLIB_TESTS_ADJACENCY_VECTOR_TESTS_HPP_

#include "../AdjacencyLists/adjacency_vector.hpp"

#include <memory>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace {
struct TestParams {
  std::vector<std::pair<int, double>> data;
};
class AdjacencyVectorTestParamatrizedFixture
    : ::testing::TestWithParam<TestParams> {
 public:
  void SetUp() {
    auto& params = GetParam();
    av = std::make_unique<Graphlib::AdjacencyVector>(params.data.size());
  }
  std::unique_ptr<Graphlib::AdjacencyVector> av;
};
}  // namespace

TEST(AdjacencyVectorTest, DefaultConstructionOK) {
  std::vector<std::size_t> data{0, 1, 2, 3, 4};
  for (auto elem : data) {
    Graphlib::AdjacencyVector av(elem);
    ASSERT_EQ(elem, av.size());
    for (int i = 0; i < elem; ++i)
      EXPECT_FALSE(av.exist(i));
  }
}

#endif  // GRAPHLIB_TESTS_ADJACENCY_VECTOR_TESTS_HPP_
