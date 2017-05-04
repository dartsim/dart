/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_DYNAMICS_IKFASTSOLVER_HPP_
#define DART_DYNAMICS_IKFASTSOLVER_HPP_

#include <memory>

#define IKFAST_HAS_LIBRARY
#include "dart/dynamics/detail/ikfast.h"

namespace dart {
namespace dynamics {

class MetaSkeleton;

// conversions between:
// - endeffector pose
// - solutions

// ik types
// - Transform6D - end effector reaches desired 6D transformation
// - Rotation3D - end effector reaches desired 3D rotation
// - Translation3D - end effector origin reaches desired 3D translation
// - Direction3D - direction on end effector coordinate system reaches desired direction
// - Ray4D - ray on end effector coordinate system reaches desired global ray
// - Lookat3D - direction on end effector coordinate system points to desired 3D position
// - TranslationDirection5D - end effector origin and direction reaches desired 3D translation and direction. Can be thought of as Ray IK where the origin of the ray must coincide.
// - TranslationXY2D - end effector origin reaches desired XY translation position, Z is ignored. The coordinate system with relative to the base link.
// - TranslationLocalGlobal6D - local point on end effector origin reaches desired 3D global point. Because both local point and global point can be specified, there are 6 values.
// - TranslationXAxisAngle4D, TranslationYAxisAngle4D, TranslationZAxisAngle4D - end effector origin reaches desired 3D translation, manipulator direction makes a specific angle with x/y/z-axis (defined in the manipulator base link’s coordinate system)
//
//The possible solve methods are defined by ikfast.IKFastSolver.GetSolvers()

// free parameter is in the range of [0, 1]

/// A wrapper class of the generated IKFast code.
class IkfastSolver
{
public:
  /// Constructor
  IkfastSolver() = default;

  /// Destructor
  virtual ~IkfastSolver() = default;

  virtual int getNumFreeParameters() = 0;
  virtual int* getFreeParameters() = 0;
  virtual int getNumJoints() = 0;
  virtual int getIkRealSize() = 0;
  virtual int getIkType() = 0;

  /// Computes the inverse kinematics solutions using the generated IKFast code.
  virtual bool computeIk(
      const IkReal* eetrans,
      const IkReal* eerot,
      const IkReal* pfree,
      ikfast::IkSolutionListBase<IkReal>& solutions) = 0;

  virtual const char* getKinematicsHash() = 0;

  virtual const char* getIkFastVersion() = 0;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_IKFASTSOLVER_HPP_
