/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_DYNAMICS_DETAIL_SOFTBODYASPECT_H_
#define DART_DYNAMICS_DETAIL_SOFTBODYASPECT_H_

#include "dart/dynamics/PointMass.h"

namespace dart {
namespace dynamics {

const double DART_DEFAULT_VERTEX_STIFFNESS = 1.0;
const double DART_DEFAULT_EDGE_STIFNESS    = 1.0;
const double DART_DEFAULT_DAMPING_COEFF    = 0.01;
// TODO(JS): remove the trailing 2 when SoftBodyNode is removed

namespace detail {

struct SoftBodyAspectState
{
  /// Array of States for PointMasses
  std::vector<PointMass::State> mPointStates;

  virtual ~SoftBodyAspectState() = default;
};

struct SoftBodyAspectProperties
{
  /// Spring stiffness for vertex deformation restoring spring force of the
  /// point masses
  double mKv;

  /// Spring stiffness for edge deformation restoring spring force of the
  /// point masses
  double mKe;

  /// Damping coefficient
  double mDampCoeff;

  /// Array of Properties for PointMasses
  std::vector<PointMass::Properties> mPointProps;

  // TODO(JS): Let's remove this because this is rendering part
  /// \brief Tri-mesh indexes for rendering.
  std::vector<Eigen::Vector3i> mFaces;

  SoftBodyAspectProperties(
      double Kv = DART_DEFAULT_VERTEX_STIFFNESS,
      double Ke = DART_DEFAULT_EDGE_STIFNESS,
      double DampCoeff = DART_DEFAULT_DAMPING_COEFF,
      const std::vector<PointMass::Properties>& points
          = std::vector<PointMass::Properties>(),
      const std::vector<Eigen::Vector3i>& faces
          = std::vector<Eigen::Vector3i>());

  virtual ~SoftBodyAspectProperties() = default;

  /// Add a PointMass to this Properties struct
  void addPointMass(const PointMass::Properties& _properties);

  /// Connect two PointMasses together in this Properties struct
  bool connectPointMasses(std::size_t i1, std::size_t i2);

  /// Add a face to this Properties struct
  void addFace(const Eigen::Vector3i& _newFace);
};

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_SOFTBODYASPECT_H_
