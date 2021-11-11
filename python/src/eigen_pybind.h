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

#include "dart/common/eigen_include.hpp"

#include "pybind11/eigen.h"

namespace dart {
namespace python {

/// Provides a mutable Ref<> for a pointer.
/// Meant to be used for decorating methods passed to `pybind11` (e.g. virtual
/// function dispatch).
// TODO(eric.cousineau): Ensure that all C++ mutator call sites use `EigenPtr`.
template <typename Derived>
auto ToEigenRef(Eigen::VectorBlock<Derived>* derived)
    -> decltype(Eigen::Ref<Derived>(*derived))
{
  return Eigen::Ref<Derived>(*derived);
}

/// Converts a raw array to a numpy array.
template <typename T>
::pybind11::object ToArray(T* ptr, int size, ::pybind11::tuple shape)
{
  // Create flat array to be reshaped in numpy.
  using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;
  Eigen::Map<Vector> data(ptr, size);
  return ::pybind11::cast(
             Eigen::Ref<Vector>(data),
             ::pybind11::return_value_policy::reference)
      .attr("reshape")(shape);
}

/// Converts a raw array to a numpy array (`const` variant).
template <typename T>
::pybind11::object ToArray(const T* ptr, int size, ::pybind11::tuple shape)
{
  // Create flat array to be reshaped in numpy.
  using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;
  Eigen::Map<Vector> data(ptr, size);
  return ::pybind11::cast(
             Eigen::Ref<Vector>(data),
             ::pybind11::return_value_policy::reference)
      .attr("reshape")(shape);
}

} // namespace python
} // namespace dart
