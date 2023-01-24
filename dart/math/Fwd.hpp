/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/math/Export.hpp>

#include <dart/common/Fwd.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// Forward declarations for math classes

namespace dart::math {

//------------------------------------------------------------------------------
// Base classes and constants
//------------------------------------------------------------------------------

/// Index type
using Index = ::Eigen::Index;

/// MatrixBase is a base class for all Eigen matrix types.
/// @tparam Derived The derived type.
template <typename Derived>
using MatrixBase = ::Eigen::MatrixBase<Derived>;

/// DenseBase is a base class for all Eigen dense matrix types.
/// @tparam Derived The derived type.
template <typename Derived>
using DenseBase = ::Eigen::DenseBase<Derived>;

/// DenseBase is a base class for all Eigen plain object types.
/// @tparam Derived The derived type.
template <typename Derived>
using PlainObjectBase = ::Eigen::PlainObjectBase<Derived>;

// Eigen constants
constexpr auto Dynamic = ::Eigen::Dynamic;
constexpr auto ColMajor = ::Eigen::StorageOptions::ColMajor;
constexpr auto RowMajor = ::Eigen::StorageOptions::RowMajor;
constexpr auto ComputeFullV = ::Eigen::DecompositionOptions::ComputeFullV;
constexpr auto StrictlyLower = ::Eigen::UpLoType::StrictlyLower;
constexpr auto StrictlyUpper = ::Eigen::UpLoType::StrictlyUpper;
constexpr auto NoChange = ::Eigen::NoChange_t::NoChange;

/// @brief Eigen map type for a plain object type (e.g., MatrixXd)
/// @tparam PlainObjectType The plain object type
/// @tparam MapOptions The map options
template <typename PlainObjectType, int MapOptions = 0>
using Map = ::Eigen::Map<PlainObjectType, MapOptions>;

//------------------------------------------------------------------------------
// Eigen matrix types
//------------------------------------------------------------------------------

/// Fixed-size matrix
/// @tparam S Scalar type
/// @tparam Rows Size of the rows
/// @tparam Cols Size of the columns
template <typename S, int Rows, int Cols = Rows, int Options = 0>
using Matrix = ::Eigen::Matrix<S, Rows, Cols, Options>;

/// Fixed-size matrix (int)
/// @tparam Rows Size of the rows
/// @tparam Cols Size of the columns
template <int Rows, int Cols = Rows>
using Matrixi = Matrix<int, Rows, Cols>;

/// Fixed-size matrix (float)
/// @tparam Rows Size of the rows
/// @tparam Cols Size of the columns
template <int Rows, int Cols = Rows>
using Matrixf = Matrix<float, Rows, Cols>;

/// Fixed-size matrix (double)
/// @tparam Rows Size of the rows
/// @tparam Cols Size of the columns
template <int Rows, int Cols = Rows>
using Matrixd = Matrix<double, Rows, Cols>;

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

/// Dynamic-size matrix
template <typename S>
using MatrixX = Matrix<S, Dynamic, Dynamic>;

/// Dynamic-size matrix (int)
using MatrixXi = MatrixX<int>;

/// Dynamic-size matrix (float)
using MatrixXf = MatrixX<float>;

/// Dynamic-size matrix (double)
using MatrixXd = MatrixX<double>;

// Fixed-size matrices for specific sizes (int)
using Matrix1i = Matrixi<1>;
using Matrix2i = Matrixi<2>;
using Matrix3i = Matrixi<3>;
using Matrix4i = Matrixi<4>;
using Matrix5i = Matrixi<5>;
using Matrix6i = Matrixi<6>;

// Fixed-size matrices for specific sizes (float)
using Matrix1f = Matrixf<1>;
using Matrix2f = Matrixf<2>;
using Matrix3f = Matrixf<3>;
using Matrix4f = Matrixf<4>;
using Matrix5f = Matrixf<5>;
using Matrix6f = Matrixf<6>;

// Fixed-size matrices for specific sizes (double)
using Matrix1d = Matrixd<1>;
using Matrix2d = Matrixd<2>;
using Matrix3d = Matrixd<3>;
using Matrix4d = Matrixd<4>;
using Matrix5d = Matrixd<5>;
using Matrix6d = Matrixd<6>;

//------------------------------------------------------------------------------
// Eigen vector types
//------------------------------------------------------------------------------

/// Fixed-size column vector
/// @tparam S Scalar type
/// @tparam Size Size of the vector
template <typename S, int Size>
using Vector = Matrix<S, Size, 1>;

/// Fixed-size column vector (int)
/// @tparam Size Size of the vector
template <int Size>
using Vectori = Vector<int, Size>;

/// Fixed-size column vector (float)
/// @tparam Size Size of the vector
template <int Size>
using Vectorf = Vector<float, Size>;

/// Fixed-size column vector (double)
/// @tparam Size Size of the vector
template <int Size>
using Vectord = Vector<double, Size>;

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

/// Dynamic-size column vector
template <typename S>
using VectorX = Vector<S, Dynamic>;

/// Dynamic-size column vector (int)
using VectorXi = VectorX<int>;

/// Dynamic-size column vector (float)
using VectorXf = VectorX<float>;

/// Dynamic-size column vector (double)
using VectorXd = VectorX<double>;

// Fixed-size vectors for specific sizes (int)
using Vector1i = Vectori<1>;
using Vector2i = Vectori<2>;
using Vector3i = Vectori<3>;
using Vector4i = Vectori<4>;
using Vector5i = Vectori<5>;
using Vector6i = Vectori<6>;

// Fixed-size vectors for specific sizes (float)
using Vector1f = Vectorf<1>;
using Vector2f = Vectorf<2>;
using Vector3f = Vectorf<3>;
using Vector4f = Vectorf<4>;
using Vector5f = Vectorf<5>;
using Vector6f = Vectorf<6>;

// Fixed-size vectors for specific sizes (double)
using Vector1d = Vectord<1>;
using Vector2d = Vectord<2>;
using Vector3d = Vectord<3>;
using Vector4d = Vectord<4>;
using Vector5d = Vectord<5>;
using Vector6d = Vectord<6>;

//------------------------------------------------------------------------------
// Eigen geometry types
//------------------------------------------------------------------------------

/// Quaternion type
/// @tparam S Scalar type
template <typename S>
using Quaternion = ::Eigen::Quaternion<S>;

/// Quaternion type (float)
using Quaternionf = Quaternion<float>;

/// Quaternion type (double)
using Quaterniond = Quaternion<double>;

/// @brief Axis-angle type
/// @tparam S Scalar type
template <typename S>
using AngleAxis = ::Eigen::AngleAxis<S>;

/// Axis-angle type (float)
using AngleAxisf = AngleAxis<float>;

/// Axis-angle type (double)
using AngleAxisd = AngleAxis<double>;

/// Isometry transform (or SE(2))
/// @tparam S Scalar type
template <typename S>
using Isometry2 = ::Eigen::Transform<S, 2, ::Eigen::Isometry>;

/// Isometry transform (or SE(2)) (float)
using Isometry2f = Isometry2<float>;

/// Isometry transform (or SE(2)) (double)
using Isometry2d = Isometry2<double>;

/// Isometry transform (or SE(3))
/// @tparam S Scalar type
template <typename S>
using Isometry3 = ::Eigen::Transform<S, 3, ::Eigen::Isometry>;

/// Isometry transform (or SE(3)) (float)
using Isometry3f = Isometry3<float>;

/// Isometry transform (or SE(3)) (double)
using Isometry3d = Isometry3<double>;

/// Affine transform (or SE(3))
/// @tparam S Scalar type
template <typename S>
using Affine3 = ::Eigen::Transform<S, 3, ::Eigen::Affine>;

/// Affine transform (or Sim(3)) (float)
using Affine3f = Affine3<float>;

/// Affine transform (or Sim(3)) (double)
using Affine3d = Affine3<double>;

/// @brief Translation transform
/// @tparam S Scalar type
template <typename S>
using Translation3 = ::Eigen::Translation<S, 3>;

/// Translation transform (float)
using Translation3f = Translation3<float>;

/// Translation transform (double)
using Translation3d = Translation3<double>;

//------------------------------------------------------------------------------
// Eigen solvers
//------------------------------------------------------------------------------

template <typename MatrixType>
using JacobiSVD = ::Eigen::JacobiSVD<MatrixType>;

template <typename MatrixType>
using FullPivLU = ::Eigen::FullPivLU<MatrixType>;

//------------------------------------------------------------------------------
// Lie Groups
//------------------------------------------------------------------------------

template <typename Scalar>
class SO3;

template <typename Scalar>
class SE3;

template <typename Scalar, template <typename> class...>
class LieGroupProduct;

//------------------------------------------------------------------------------
// Other DART types
//------------------------------------------------------------------------------

using Inertia = Matrix6d; // TODO(JS): Rename to SpatialInertia
using LinearJacobian = Matrix<double, 3, Dynamic>;
using AngularJacobian = Matrix<double, 3, Dynamic>;
using Jacobian
    = Matrix<double, 6, Dynamic>; // TODO(JS): Rename to SpatialJacobian

DART_DECLARE_CLASS_POINTERS_S(Color);

} // namespace dart::math
