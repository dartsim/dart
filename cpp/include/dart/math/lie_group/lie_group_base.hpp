/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <Eigen/Core>

#include "dart/math/lie_group/type.hpp"

//==============================================================================
template <typename Derived>
class LieGroupBase
{
public:
  // Properties
  static constexpr int Options = Eigen::internal::traits<Derived>::Options;
  static constexpr int SpaceDim = Eigen::internal::traits<Derived>::SpaceDim;
  static constexpr int GroupDim = Eigen::internal::traits<Derived>::GroupDim;
  static constexpr int MatrixDim = Eigen::internal::traits<Derived>::MatrixDim;
  static constexpr int RepDim = Eigen::internal::traits<Derived>::RepDim;

  // Type defs
  using Scalar = typename Eigen::internal::traits<Derived>::Scalar;
  using Matrix = Eigen::Matrix<Scalar, MatrixDim, MatrixDim>;
  using LieGroup = typename Eigen::internal::traits<Derived>::LieGroup;
  // using LieGroupData = typename
  // Eigen::internal::traits<Derived>::LieGroupData;
  using LieGroupCoeffs =
      typename Eigen::internal::traits<Derived>::LieGroupCoeffs;
  using Tangent = typename Eigen::internal::traits<Derived>::Tangent;
  using TangentData = typename Eigen::internal::traits<Derived>::TangentData;
  using LieAlgebra = typename Eigen::internal::traits<Derived>::LieAlgebra;
  using Cotangent = typename Eigen::internal::traits<Derived>::Cotangent;
  using CotangentData =
      typename Eigen::internal::traits<Derived>::CotangentData;
  using Jacobian = typename Eigen::internal::traits<Derived>::Jacobian;

  using Rotation = math::Matrix<Scalar, SpaceDim, SpaceDim>;
  using Translation = math::Vector<Scalar, SpaceDim>;
  using Transformation = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry>;

  void set_identity();

  const LieGroupCoeffs& coeffs() const;

  LieGroupCoeffs& coeffs();

  const Derived& derived() const noexcept;

  Derived& derived() noexcept;
};
