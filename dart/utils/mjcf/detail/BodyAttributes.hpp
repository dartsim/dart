/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#ifndef DART_UTILS_MJCF_DETAIL_BODYATTRIBUTES_HPP_
#define DART_UTILS_MJCF_DETAIL_BODYATTRIBUTES_HPP_

#include "dart/common/Optional.hpp"
#include "dart/math/MathTypes.hpp"
#include "dart/utils/mjcf/detail/Error.hpp"
#include "dart/utils/mjcf/detail/Inertial.hpp"

#include <Eigen/Dense>
#include <tinyxml2.h>

#include <string>

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

class Size;

/// Intermediate raw data read from the XML file. For the details, see
/// http://www.mujoco.org/book/XMLreference.html#body
struct BodyAttributes final
{
  /// Name of the body.
  std::optional<std::string> mName;

  /// If this attribute is present, all descendant elements that admit a
  /// defaults class will use the class specified here, unless they specify
  /// their own class or another body with a childclass attribute is
  /// encountered along the chain of nested bodies.
  std::optional<std::string> mChildClass;

  /// If this attribute is "true", the body is labeled as a mocap body.
  bool mMocap{false};

  /// The 3D position of the body frame, in local or global coordinates as
  /// determined by the coordinate attribute of compiler.
  std::optional<Eigen::Vector3d> mPos;

  /// Quaternion
  Eigen::Quaterniond mQuat{Eigen::Quaterniond::Identity()};

  /// These are the quantities (x, y, z, a) mentioned above. The last number
  /// is the angle of rotation, in degrees or radians as specified by the
  /// angle attribute of compiler.
  std::optional<Eigen::Vector4d> mAxisAngle;

  /// Rotation angles around three coordinate axes. The sequence of axes
  /// around which these rotations are applied is determined by the eulerseq
  /// attribute of compiler and is the same for the entire model.
  std::optional<Eigen::Vector3d> mEuler;

  /// The first 3 numbers are the X axis of the frame. The next 3 numbers are
  /// the Y axis of the frame, which is automatically made orthogonal to the X
  /// axis. The Z axis is then defined as the cross-product of the X and Y
  /// axes.
  std::optional<Eigen::Vector6d> mXYAxes;

  /// The Z axis of the frame
  std::optional<Eigen::Vector3d> mZAxis;

  Eigen::VectorXd mUser;

  std::optional<Inertial> mInertial;
};

Errors appendBodyAttributes(
    BodyAttributes& attributes,
    tinyxml2::XMLElement* element,
    const std::optional<Size>& size);

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_MJCF_DETAIL_BODYATTRIBUTES_HPP_
