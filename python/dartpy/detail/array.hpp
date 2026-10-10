#pragma once

#include "dart_nb.hpp"

#include <nanobind/stl/detail/nb_array.h>

namespace nanobind::detail {
template <class T, std::size_t Size>
struct type_caster<std::array<T, Size>>
  : array_caster<std::array<T, Size>, T, Size>
{
  using Base = array_caster<std::array<T, Size>, T, Size>;
  static constexpr auto Name = const_name("typing.Annotated[") + Base::Name
                               + const_name(", \"FixedSize(")
                               + const_name<Size>() + const_name(")\"]");
};
} // namespace nanobind::detail
