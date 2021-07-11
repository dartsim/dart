/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dart {
namespace math {

template <typename S>
using MatrixX = Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic>;

template <typename S, int Dim>
using Matrix = Eigen::Matrix<S, Dim, Dim>;

template <typename S>
using Matrix1 = Matrix<S, 1>;

template <typename S>
using Matrix2 = Matrix<S, 2>;

template <typename S>
using Matrix3 = Matrix<S, 3>;

template <typename S>
using Matrix4 = Matrix<S, 4>;

template <typename S>
using Matrix5 = Matrix<S, 5>;

template <typename S>
using Matrix6 = Matrix<S, 6>;

template <typename S>
using VectorX = Eigen::Matrix<S, Eigen::Dynamic, 1>;

template <typename S, int Dim>
using Vector = Eigen::Matrix<S, Dim, 1>;

template <typename S>
using Vector1 = Vector<S, 1>;

template <typename S>
using Vector2 = Vector<S, 2>;

template <typename S>
using Vector3 = Vector<S, 3>;

template <typename S>
using Vector4 = Vector<S, 4>;

template <typename S>
using Vector5 = Vector<S, 5>;

template <typename S>
using Vector6 = Vector<S, 6>;

template <typename S>
using Isometry3 = Eigen::Transform<S, 3, Eigen::Isometry>;

} // namespace math
} // namespace dart
