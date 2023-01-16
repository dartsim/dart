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

#ifndef DART_UTILS_MJCF_DETAIL_INERTIAL_HPP_
#define DART_UTILS_MJCF_DETAIL_INERTIAL_HPP_

#include <dart/io/mjcf/detail/Compiler.hpp>
#include <dart/io/mjcf/detail/Error.hpp>

#include <dart/math/Fwd.hpp>

#include <tinyxml2.h>

#include <optional>

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

class Body;

class Inertial final
{
public:
  Inertial() = default;

  Errors read(tinyxml2::XMLElement* element);

  void setMass(double mass);
  double getMass() const;

  void setDiagInertia(const math::Vector3d& inertia);
  const math::Vector3d& getDiagInertia() const;

  void setOffDiagInertia(const math::Vector3d& inertia);
  const math::Vector3d& getOffDiagInertia() const;

  void setRelativeTransform(const math::Isometry3d& tf);
  const math::Isometry3d& getRelativeTransform() const;

  void setWorldTransform(const math::Isometry3d& tf);
  const math::Isometry3d& getWorldTransform() const;

private:
  // Private members used by Body class
  friend class Body;
  Errors compile(const Compiler& compiler);

private:
  struct Data
  {
    /// Position of the inertial frame.
    math::Vector3d mPos;

    /// Quaternion
    math::Quaterniond mQuat{math::Quaterniond::Identity()};

    /// These are the quantities (x, y, z, a) mentioned above. The last number
    /// is the angle of rotation, in degrees or radians as specified by the
    /// angle attribute of compiler.
    std::optional<math::Vector4d> mAxisAngle;

    /// Rotation angles around three coordinate axes.
    std::optional<math::Vector3d> mEuler;

    /// The first 3 numbers are the X axis of the frame. The next 3 numbers are
    /// the Y axis of the frame, which is automatically made orthogonal to the X
    /// axis. The Z axis is then defined as the cross-product of the X and Y
    /// axes.
    std::optional<math::Vector6d> mXYAxes;

    /// The Z axis of the frame
    std::optional<math::Vector3d> mZAxis;

    math::Isometry3d mRelativeTransform{math::Isometry3d::Identity()};
    math::Isometry3d mWorldTransform{math::Isometry3d::Identity()};

    /// Mass of the body.
    double mMass;

    /// Diagonal inertia matrix, expressing the body inertia relative to the
    /// inertial frame.
    std::optional<math::Vector3d> mDiagInertia;

    /// Full inertia matrix M.
    std::optional<math::Vector6d> mFullInertia;
  };

  Data mData;

  /// Position of the inertial frame.
  math::Vector3d mPos;

  math::Isometry3d mRelativeTransform{math::Isometry3d::Identity()};
  math::Isometry3d mWorldTransform{math::Isometry3d::Identity()};

  /// Mass of the body.
  double mMass;

  math::Vector3d mDiagonalInertia;
  math::Vector3d mOffDiagonalInertia;
};

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart

#endif // #ifndef DART_UTILS_MJCF_DETAIL_INERTIAL_HPP_
