//
// All components of Drake are licensed under the BSD 3-Clause License
// shown below. Where noted in the source code, some portions may
// be subject to other permissive, non-viral licenses.
//
// Copyright 2012-2016 Robot Locomotion Group @ CSAIL
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.  Redistributions
// in binary form must reproduce the above copyright notice, this list of
// conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.  Neither the name of
// the Massachusetts Institute of Technology nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#pragma once

/// @file
/// Provides pybind11 `type_caster`s for Eigen geometric types.
/// N.B. This uses some of pybind's coding conventions.
///
/// See http://pybind11.readthedocs.io/en/stable/advanced/cast/custom.html for
/// more details on custom type casters.

#include <string>
#include <utility>

#include <Eigen/Dense>
#include "pybind11/eigen.h"

namespace dart {
namespace python {
namespace detail {

// Implements a `type_caster<>` specialization used to convert types using a
// specific wrapping policy.
// @tparam Wrapper
//  Struct which must provide `Type`, `WrappedType`, `unwrap`, and `wrap`.
// @tparam copy_only
//  This may only pass between C++ and Python as copies, not references.
// See `eigen_wrapper_*` structs below for more details.
template <typename Wrapper>
struct type_caster_wrapped {
  using Type = typename Wrapper::Type;
  using WrappedType = typename Wrapper::WrappedType;
  using WrappedTypeCaster = ::pybind11::detail::type_caster<WrappedType>;

  // Python to C++.
  bool load(::pybind11::handle src, bool converter) {
    WrappedTypeCaster caster;
    if (!caster.load(src, converter)) {
      return false;
    }
    value_ = Wrapper::unwrap(caster.operator WrappedType&());
    loaded_ = true;
    return true;
  }

  // See `pybind11/eigen.h`, `type_caster<>` implementations.
  // N.B. Do not use `PYBIND11_TYPE_CASTER(...)` so we can avoid casting
  // garbage values.
  operator Type&() {
    assert(loaded_);
    return value_;
  }
  template <typename T> using cast_op_type =
      ::pybind11::detail::movable_cast_op_type<T>;
  static constexpr auto name = WrappedTypeCaster::props::descriptor;

  // C++ to Python.
  template <typename TType>
  static ::pybind11::handle cast(TType&& src, ::pybind11::return_value_policy policy,
      ::pybind11::handle parent) {
    if (policy == ::pybind11::return_value_policy::reference ||
        policy == ::pybind11::return_value_policy::reference_internal) {
      // N.B. We must declare a local `static constexpr` here to prevent
      // linking errors. This does not appear achievable with
      // `constexpr char[]`, so we use `::pybind11::detail::descr`.
      // See `pybind11/pybind11.h`, `cpp_function::initialize(...)` for an
      // example.
      static constexpr auto original_name = Wrapper::original_name;
      throw ::pybind11::cast_error(
          std::string("Can only pass ") + original_name.text +
          " by value.");
    }
    return WrappedTypeCaster::cast(
        Wrapper::wrap(std::forward<TType>(src)), policy, parent);
  }

 private:
  bool loaded_{false};
  Type value_;
};

// Wrapper for Eigen::Translation<>, to be used as first parameter to
// `type_caster_wrapped`.
// Since there are not many special operations for a translation vector, we
// shall return it as a nominal vector.
template <typename T, int Dim>
struct wrapper_eigen_translation {
  using Type = Eigen::Translation<T, Dim>;
  using WrappedType = Eigen::Matrix<T, Dim, 1>;
  static constexpr auto original_name = ::pybind11::detail::_("Eigen::Translation<>");
  static Type unwrap(const WrappedType& arg_wrapped) {
    return Type(arg_wrapped);
  }
  static WrappedType wrap(const Type& arg) {
    return arg.vector();
  }
};

// N.B. Since `Isometry3<>` and `Eigen::Quaternion<>` have more
// complicated structures, they are registered as types in `eigen_geometry_py`.

}  // namespace detail
}  // namespace python
}  // namespace dart

namespace pybind11 {
namespace detail {

template <typename T, int Dim>
struct type_caster<Eigen::Translation<T, Dim>>
    : public dart::python::detail::type_caster_wrapped<
        dart::python::detail::wrapper_eigen_translation<T, Dim>> {};

}  // namespace detail
}  // namespace pybind11
