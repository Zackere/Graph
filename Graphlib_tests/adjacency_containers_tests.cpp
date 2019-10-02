// Copyright 2019 Wojciech Replin

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "../AdjacencyLists/adjacency_list.hpp"
#include "../AdjacencyLists/adjacency_map.hpp"
#include "../AdjacencyLists/adjacency_vector.hpp"

namespace {
struct TestParams {
  std::vector<std::pair<int, double>> data;
};
template <typename TestedContainer>
class ContructorTestFixture : public testing::Test {
 public:
  const std::vector<std::size_t> okvalues{0, 1, 2, 3};
};

template <typename TestedContainer>
class InsertRemoveTestFixture : public testing::Test {
 public:
  const std::vector<TestParams> okparams{
      TestParams{}, TestParams{std::vector<std::pair<int, double>>{{0, 1.0f}}},
      TestParams{std::vector<std::pair<int, double>>{{0, 2.0f}, {1, 5.0f}}},
      TestParams{std::vector<std::pair<int, double>>{
          {0, -12.0f},
          {1, 3.14159},
          {2, 0.0f},
      }}};
};
template <typename TestedContainer>
class ExistTestFixture : public InsertRemoveTestFixture<TestedContainer> {};
template <typename TestedContainer>
class IndexOperatorTestFixture
    : public InsertRemoveTestFixture<TestedContainer> {};
template <typename TestedContainer>
class IteratorTestsFixture : public testing::Test {
 public:
  const std::vector<std::pair<int, std::vector<std::pair<int, double>>>> params{
      {6, std::vector<std::pair<int, double>>{{0, -12.0f},
                                              {4, 3.14159},
                                              {2, 0.0f}}},
      {6, std::vector<std::pair<int, double>>{{5, -12.0f},
                                              {3, 3.14159},
                                              {1, 0.0f}}}};
};
}  // namespace

using ContainerTypes = testing::Types<graphlib::AdjacencyVector,
                                      graphlib::AdjacencyList,
                                      graphlib::AdjacencyHashTable,
                                      graphlib::AdjacencyAVLTree>;
TYPED_TEST_CASE(ContructorTestFixture, ContainerTypes);
TYPED_TEST_CASE(InsertRemoveTestFixture, ContainerTypes);
TYPED_TEST_CASE(ExistTestFixture, ContainerTypes);
TYPED_TEST_CASE(IndexOperatorTestFixture, ContainerTypes);
TYPED_TEST_CASE(IteratorTestsFixture, ContainerTypes);

TYPED_TEST(ContructorTestFixture, ResultOK) {
  for (std::size_t elem : this->okvalues) {
    std::unique_ptr<TypeParam> p;
    ASSERT_NO_THROW(p = std::make_unique<TypeParam>(elem));
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->size(), 0u);
    for (graphlib::int_vertex i(0); i < graphlib::int_vertex(elem); ++i)
      EXPECT_FALSE((*p)[i].has_value());
  }
}

TYPED_TEST(ContructorTestFixture, CheckIterators) {
  for (std::size_t elem : this->okvalues) {
    std::unique_ptr<TypeParam> p;
    p = std::make_unique<TypeParam>(elem);
    EXPECT_EQ(p->begin(), p->end());

    EXPECT_EQ(++p->begin(), p->begin());
    EXPECT_EQ(++++p->begin(), p->begin());
    EXPECT_EQ(--p->begin(), p->begin());
    EXPECT_EQ(----p->begin(), p->begin());
    EXPECT_EQ(--p->end(), p->end());
    EXPECT_EQ(----p->end(), p->end());
    EXPECT_EQ(++p->end(), p->end());
    EXPECT_EQ(++++p->end(), p->end());
  }
}

TYPED_TEST(InsertRemoveTestFixture, InsertTestOK) {
  for (TestParams const& param : this->okparams) {
    auto p = std::make_unique<TypeParam>(param.data.size());
    std::size_t inserted_elems = 0u;
    for (auto const& elem : param.data) {
      graphlib::int_vertex v(elem.first);
      EXPECT_FALSE(p->exist(v));
      EXPECT_TRUE(p->insert(v, elem.second));
      EXPECT_TRUE(p->exist(v));
      EXPECT_FALSE(p->insert(v, elem.second));
      ASSERT_TRUE((*p)[v].has_value());
      EXPECT_EQ((*p)[v].value(), elem.second);
      EXPECT_EQ(const_cast<TypeParam&>(*p)[v].value(), elem.second);
      EXPECT_EQ(p->size(), ++inserted_elems);
    }
  }
}

TYPED_TEST(InsertRemoveTestFixture, RemoveTestOK) {
  for (TestParams const& param : this->okparams) {
    auto p = std::make_unique<TypeParam>(param.data.size());
    for (auto const& elem : param.data) {
      graphlib::int_vertex v(elem.first);
      EXPECT_FALSE(p->exist(v));
      EXPECT_FALSE(p->remove(v));
      EXPECT_EQ(p->size(), 0u);
      EXPECT_FALSE(p->exist(v));
      p->insert(v, elem.second);
      EXPECT_TRUE(p->exist(v));
      EXPECT_TRUE(p->remove(v));
      EXPECT_EQ(p->size(), 0u);
      EXPECT_FALSE(p->exist(v));
    }
  }
}

TYPED_TEST(IteratorTestsFixture, CheckAllIterators) {
  for (auto const& param : this->params) {
    auto p = std::make_unique<TypeParam>(param.first);
    for (auto const& elem : param.second)
      p->insert(graphlib::int_vertex(elem.first), elem.second);
    for (typename TypeParam::Iterator container_it = p->begin();
         container_it != p->end(); ++container_it)
      EXPECT_NE(std::find_if(param.second.begin(), param.second.end(),
                             [](auto const& pair) { return true; }),
                param.second.end());
  }
}
