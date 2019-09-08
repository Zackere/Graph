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

using ContainerTypes = testing::Types<Graphlib::AdjacencyVector,
                                      Graphlib::AdjacencyList,
                                      Graphlib::AdjacencyHashTable,
                                      Graphlib::AdjacencyAVLTree>;
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
  }
}

TYPED_TEST(ContructorTestFixture, CheckIterators) {
  for (std::size_t elem : this->okvalues) {
    std::unique_ptr<TypeParam> p;
    p = std::make_unique<TypeParam>(elem);
    EXPECT_EQ(p->begin(), p->end());
    EXPECT_EQ(++p->begin(), p->begin());
    EXPECT_EQ(--p->begin(), p->begin());
    EXPECT_EQ(--p->end(), p->end());
    EXPECT_EQ(++p->end(), p->end());
    EXPECT_THROW(*p->begin(), std::out_of_range);
    EXPECT_THROW(*p->end(), std::out_of_range);
  }
}

TYPED_TEST(InsertRemoveTestFixture, InsertTestOK) {
  for (TestParams const& param : this->okparams) {
    auto p = std::make_unique<TypeParam>(param.data.size());
    std::size_t inserted_elems = 0u;
    for (auto const& elem : param.data) {
      EXPECT_FALSE(p->exist(elem.first));
      EXPECT_TRUE(p->insert(elem.first, elem.second));
      EXPECT_TRUE(p->exist(elem.first));
      EXPECT_FALSE(p->insert(elem.first, elem.second));
      EXPECT_EQ((*p)[elem.first], elem.second);
      EXPECT_EQ(const_cast<TypeParam&>(*p)[elem.first], elem.second);
      EXPECT_EQ(p->size(), ++inserted_elems);
    }
  }
}

TYPED_TEST(InsertRemoveTestFixture, RemoveTestOK) {
  for (TestParams const& param : this->okparams) {
    auto p = std::make_unique<TypeParam>(param.data.size());
    for (auto const& elem : param.data) {
      EXPECT_FALSE(p->exist(elem.first));
      EXPECT_FALSE(p->remove(elem.first));
      EXPECT_EQ(p->size(), 0u);
      EXPECT_FALSE(p->exist(elem.first));
      p->insert(elem.first, elem.second);
      EXPECT_TRUE(p->exist(elem.first));
      EXPECT_TRUE(p->remove(elem.first));
      EXPECT_EQ(p->size(), 0u);
      EXPECT_FALSE(p->exist(elem.first));
    }
  }
}

TYPED_TEST(IteratorTestsFixture, CheckAllIterators) {
  for (auto const& param : this->params) {
    auto p = std::make_unique<TypeParam>(param.first);
    for (auto const& elem : param.second)
      p->insert(elem.first, elem.second);
    for (auto container_it = p->begin(); container_it != p->end();
         ++container_it)
      EXPECT_NE(std::find_if(param.second.begin(), param.second.end(),
                             [&container_it](auto const& pair) {
                               return container_it->first == pair.first &&
                                      container_it->second == pair.second;
                             }),
                param.second.end());
  }
}
