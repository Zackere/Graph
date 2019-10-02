// Copyright 2019 Wojciech Replin

#ifndef UTILS_MAP_UTILS_HPP_
#define UTILS_MAP_UTILS_HPP_

#include <map>
#include <type_traits>
#include <unordered_map>

#include "../Graphs/edge.hpp"
#include "../Graphs/graph.hpp"

namespace graphlib {
namespace util {

template <typename T>
class IsMapType : public std::false_type {};

template <>
class IsMapType<std::unordered_map<graphlib::int_vertex,
                                   typename Edge<graphlib::int_vertex>::Weight>>
    : public std::true_type {};

template <>
class IsMapType<
    std::map<graphlib::int_vertex, typename Edge<graphlib::int_vertex>::Weight>>
    : public std::true_type {};

template <typename T>
using EnableIfMap = typename std::enable_if<util::IsMapType<T>::value>::type;

}  // namespace util
}  // namespace graphlib

#endif  // UTILS_MAP_UTILS_HPP_
