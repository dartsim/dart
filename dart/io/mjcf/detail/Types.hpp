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

#ifndef DART_IO_MJCF_DETAIL_TYPES_HPP_
#define DART_IO_MJCF_DETAIL_TYPES_HPP_

#include "dart/common/Platform.hpp"

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

/// This attribute specifies whether the angles in the MJCF model are
/// expressed in units of degrees or radians. The compiler converts degrees
/// into radians, and mjModel always uses radians. For URDF models the parser
/// sets this attribute to "radian" internally, regardless of the XML setting.
enum class Angle
{
  /// [Default] Always use for MJCF
  DEGREE = 0,

  /// Always use for URDF
  RADIAN = 1,
};

/// This attribute specifies whether the frame positions and orientations in the
/// MJCF model are expressed in local or global coordinates; recall Coordinate
/// frames. The compiler converts global into local coordinates, and mjModel
/// always uses local coordinates. For URDF models the parser sets this
/// attribute to "local" internally, regardless of the XML setting.
enum class Coordinate
{
  /// [Default]
  LOCAL = 0,
  GLOBAL = 1,
};

/// This attribute selects the numerical integrator to be used. Currently the
/// available integrators are the semi-implicit Euler method and the fixed-step
/// 4-th order Runge Kutta method.
enum class Integrator
{
  /// [Default]
  EULER = 0,
  RK4 = 1,
};

enum class InertiaFromGeom
{
#if DART_OS_WINDOWS
  IFG_FALSE,
  IFG_TRUE,
  IFG_AUTO,
#else
  FALSE,
  TRUE,
  AUTO,
#endif
};

enum class CollisionType
{
  ALL,
  PREDEFINED,
  DYNAMIC,
};

enum class ConeType
{
  PYRAMIDAL,
  ELLIPTIC,
};

enum class JacobianType
{
  DENSE,
  SPARSE,
  AUTO,
};

enum class SolverType
{
  PGS,
  CG,
  NEWTON,
};

enum class GeomType
{
  PLANE,
  HFIELD,
  /// [Default]
  SPHERE,
  CAPSULE,
  ELLIPSOID,
  CYLINDER,
  BOX,
  MESH,
};

enum class JointType
{
  FREE,
  BALL,
  SLIDE,
  /// [Default]
  HINGE,
};

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart

#endif // #ifndef DART_IO_MJCF_DETAIL_TYPES_HPP_
