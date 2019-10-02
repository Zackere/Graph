// Copyright 2019 Wojciech Replin

#ifndef UTILS_STRONG_ALIAS_HPP_
#define UTILS_STRONG_ALIAS_HPP_

#include <functional>
#include <ostream>
#include <utility>

namespace graphlib {
namespace util {

template <typename TagType, typename UnderlyingType>
class StrongAlias {
 public:
  StrongAlias() = default;
  StrongAlias(StrongAlias const&) = default;
  StrongAlias& operator=(StrongAlias const&) = default;
  StrongAlias(StrongAlias&&) = default;
  StrongAlias& operator=(StrongAlias&&) = default;
  virtual ~StrongAlias() = default;

  explicit StrongAlias(UnderlyingType const& v) : value_(v) {}
  explicit StrongAlias(UnderlyingType&& v) : value_(std::move(v)) {}

  UnderlyingType const& value() const { return value_; }
  UnderlyingType& value() { return value_; }
  explicit operator UnderlyingType() { return value_; }

  bool operator==(StrongAlias const& other) const {
    return value_ == other.value_;
  }
  bool operator!=(StrongAlias const& other) const {
    return value_ != other.value_;
  }
  bool operator<(StrongAlias const& other) const {
    return value_ < other.value_;
  }
  bool operator<=(StrongAlias const& other) const {
    return value_ <= other.value_;
  }
  bool operator>(StrongAlias const& other) const {
    return value_ > other.value_;
  }
  bool operator>=(StrongAlias const& other) const {
    return value_ >= other.value_;
  }
  StrongAlias& operator++() {
    ++value_;
    return *this;
  }
  StrongAlias& operator--() {
    --value_;
    return *this;
  }
  StrongAlias operator++(int) {
    StrongAlias ret(value_++);
    return ret;
  }
  StrongAlias operator--(int) {
    StrongAlias ret(value_--);
    return ret;
  }
  StrongAlias operator+(StrongAlias const& other) {
    return StrongAlias(value_ + other.value_);
  }
  StrongAlias operator-(StrongAlias const& other) {
    return StrongAlias(value_ - other.value_);
  }

 private:
  UnderlyingType value_;
};

template <typename TagType, typename UnderlyingType>
std::ostream& operator<<(std::ostream& stream,
                         StrongAlias<TagType, UnderlyingType> const& alias) {
  return stream << alias.value();
}

}  // namespace util
}  // namespace graphlib

namespace std {
template <typename TagType, typename UnderlyingType>
struct hash<graphlib::util::StrongAlias<TagType, UnderlyingType>> {
  std::size_t operator()(
      graphlib::util::StrongAlias<TagType, UnderlyingType> const& id) const {
    return std::hash<UnderlyingType>()(id.value());
  }
};
}  // namespace std

#endif  // UTILS_STRONG_ALIAS_HPP_
