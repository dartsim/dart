#pragma once

#include "detail/eigen.hpp"

namespace nanobind::detail {
template <class Scalar, int Dim>
struct type_caster<Eigen::Translation<Scalar, Dim>>
{
  using Vector = Eigen::Matrix<Scalar, Dim, 1>;
  using Translation = Eigen::Translation<Scalar, Dim>;
  NB_TYPE_CASTER(Translation, make_caster<Vector>::Name)
  bool from_python(handle src, uint32_t flags, cleanup_list* cleanup) noexcept
  {
    make_caster<Vector> caster;
    if (!caster.from_python(src, flags, cleanup))
      return false;
    value = Value(caster.operator Vector&());
    return true;
  }
  static handle from_cpp(
      const Value& value, rv_policy policy, cleanup_list* cleanup) noexcept
  {
    return make_caster<Vector>::from_cpp(
        Vector(value.vector()), policy, cleanup);
  }
};
} // namespace nanobind::detail
