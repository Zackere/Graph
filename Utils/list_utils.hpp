// Copyright 2019 Wojciech Replin

#ifndef UTILS_LIST_UTILS_HPP_
#define UTILS_LIST_UTILS_HPP_

#include <list>
#include <type_traits>

#include "../Graphs/edge.hpp"
#include "../Graphs/graph.hpp"

namespace graphlib {
namespace util {


template <typename T>
class IsListType : public std::false_type {};

template <class T>
class IsListType<std::list<T>> : public std::true_type {};

template <typename T>
using EnableIfList = typename std::enable_if<IsListType<T>::value>::type;

}  // namespace util
}  // namespace graphlib

#endif  // UTILS_LIST_UTILS_HPP_
