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

#ifndef DART_UTILS_MJCF_DETAIL_SITE_HPP_
#define DART_UTILS_MJCF_DETAIL_SITE_HPP_

#include <string>

#include <tinyxml2.h>

#include "dart/common/Optional.hpp"
#include "dart/math/MathTypes.hpp"
#include "dart/utils/mjcf/detail/Compiler.hpp"
#include "dart/utils/mjcf/detail/Error.hpp"
#include "dart/utils/mjcf/detail/Types.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

class Body;

class Site final
{
public:
  /// Default constructor
  Site() = default;

  /// \{ \name Attributes

  const std::string& getName() const;
  GeomType getType() const;
  int getGroup() const;

  const Eigen::Vector3d& getSize() const;

  double getSphereRadius() const;

  double getCapsuleRadius() const;
  double getCapsuleHalfLength() const;
  double getCapsuleLength() const;

  const Eigen::Vector3d& getEllipsoidRadii() const;
  Eigen::Vector3d getEllipsoidDiameters() const;

  double getCylinderRadius() const;
  double getCylinderHalfLength() const;
  double getCylinderLength() const;

  const Eigen::Vector3d& getBoxHalfSize() const;
  Eigen::Vector3d getBoxSize() const;

  const Eigen::Vector4d& getRGBA() const;
  const Eigen::Vector3d& getFriction() const;

  void setRelativeTransform(const Eigen::Isometry3d& tf);
  const Eigen::Isometry3d& getRelativeTransform() const;

  void setWorldTransform(const Eigen::Isometry3d& tf);
  const Eigen::Isometry3d& getWorldTransform() const;

  /// \}

private:
  // Private members used by Body and WorldBody class
  friend class Body;
  friend class Worldbody;
  Errors read(tinyxml2::XMLElement* element);

  /// Updates attributes and elements that doesn't require any other elements.
  Errors preprocess(const Compiler& compiler);

  /// Updates attributes and elements that require the preprocessed child
  /// elements of this <Site>.
  Errors compile(const Compiler& compiler);

  /// Updates attributes and elements that require the compiled parent element.
  Errors postprocess(const Body* body, const Compiler& compiler);

private:
  double computeVolume() const;
  Eigen::Matrix3d computeInertia() const;

  /// Intermediate raw data read from the XML file. For the details, see
  /// http://www.mujoco.org/book/XMLreference.html#Site
  struct Data
  {
    /// Name of the Site
    std::optional<std::string> mName;

    /// Type of Siteetric shape
    GeomType mType{GeomType::SPHERE};

    int mGroup{0};

    /// Site size parameters
    Eigen::Vector3d mSize{Eigen::Vector3d::Zero()};

    Eigen::Vector4d mRGBA{Eigen::Vector4d(0.5, 0.5, 0.5, 1)};

    /// This attribute can only be used with capsule, cylinder, ellipsoid and
    /// box Sites. It provides an alternative specification of the Site length
    /// as well as the frame position and orientation.
    std::optional<Eigen::Vector6d> mFromTo;

    /// Position of the Site frame, in local or global coordinates as determined
    /// by the coordinate attribute of compiler.
    Eigen::Vector3d mPos{Eigen::Vector3d::Zero()};

    /// Quaternion
    Eigen::Quaterniond mQuat{Eigen::Quaterniond::Identity()};

    /// These are the quantities (x, y, z, a) mentioned above. The last number
    /// is the angle of rotation, in degrees or radians as specified by the
    /// angle attribute of compiler.
    std::optional<Eigen::Vector4d> mAxisAngle;

    /// Rotation angles around three coordinate axes.
    std::optional<Eigen::Vector3d> mEuler;

    /// The first 3 numbers are the X axis of the frame. The next 3 numbers are
    /// the Y axis of the frame, which is automatically made orthogonal to the X
    /// axis. The Z axis is then defined as the cross-product of the X and Y
    /// axes.
    std::optional<Eigen::Vector6d> mXYAxes;

    /// The Z axis of the frame
    std::optional<Eigen::Vector3d> mZAxis;
  };

  Data mData;

  /// Name of the Site
  std::string mName{""};

  /// Type of Siteetric shape
  GeomType mType{GeomType::SPHERE};

  int mGroup{0};

  /// Site size parameters
  Eigen::Vector3d mSize{Eigen::Vector3d::Constant(0.005)};

  Eigen::Vector4d mRGBA{Eigen::Vector4d(0.5, 0.5, 0.5, 1)};

  double mMass;

  Eigen::Isometry3d mRelativeTransform{Eigen::Isometry3d::Identity()};

  Eigen::Isometry3d mWorldTransform{Eigen::Isometry3d::Identity()};
};

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_MJCF_DETAIL_SITE_HPP_
