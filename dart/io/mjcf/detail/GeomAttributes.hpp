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

#ifndef DART_IO_MJCF_DETAIL_GEOMATTRIBUTES_HPP_
#define DART_IO_MJCF_DETAIL_GEOMATTRIBUTES_HPP_

#include <Eigen/Core>

#include <tinyxml2.h>

#include "dart/common/Optional.hpp"
#include "dart/io/mjcf/detail/Error.hpp"
#include "dart/io/mjcf/detail/Types.hpp"
#include "dart/math/MathTypes.hpp"

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

/// Intermediate raw data read from the XML file. For the details, see
/// http://www.mujoco.org/book/XMLreference.html#geom
struct GeomAttributes final
{
  /// Name of the geom
  common::optional<std::string> mName;

  /// Type of geometric shape
  GeomType mType{GeomType::SPHERE};

  int mConType{1};

  int mConAffinity{1};

  int mConDim{3};

  int mGroup{0};

  int mPriority{0};

  /// Geom size parameters
  Eigen::Vector3d mSize{Eigen::Vector3d::Zero()};

  Eigen::Vector4d mRGBA{Eigen::Vector4d(0.5, 0.5, 0.5, 1)};

  Eigen::Vector3d mFriction{Eigen::Vector3d(1, 0.005, 0.0001)};

  /// Mass
  common::optional<double> mMass;

  /// Material density used to compute the geom mass and inertia.
  double mDensity{1000};

  /// Weight used for averaging of contact parameters, and interacts with the
  /// priority attribute.
  double mSolMix{1};

  /// Distance threshold below which contacts are detected and included in the
  /// global array mjData.contact.
  double mMargin{0};

  /// This attribute is used to enable the generation of inactive contacts,
  /// i.e. contacts that are ignored by the constraint solver but are included
  /// in mjData.contact for the purpose of custom computations.
  double mGap{0};

  /// This attribute can only be used with capsule, cylinder, ellipsoid and
  /// box geoms. It provides an alternative specification of the geom length
  /// as well as the frame position and orientation.
  common::optional<Eigen::Vector6d> mFromTo;

  /// Position of the geom frame, in local or global coordinates as determined
  /// by the coordinate attribute of compiler.
  Eigen::Vector3d mPos{Eigen::Vector3d::Zero()};

  /// Quaternion
  Eigen::Quaterniond mQuat{Eigen::Quaterniond::Identity()};

  /// These are the quantities (x, y, z, a) mentioned above. The last number
  /// is the angle of rotation, in degrees or radians as specified by the
  /// angle attribute of compiler.
  common::optional<Eigen::Vector4d> mAxisAngle;

  /// Rotation angles around three coordinate axes.
  common::optional<Eigen::Vector3d> mEuler;

  /// The first 3 numbers are the X axis of the frame. The next 3 numbers are
  /// the Y axis of the frame, which is automatically made orthogonal to the X
  /// axis. The Z axis is then defined as the cross-product of the X and Y
  /// axes.
  common::optional<Eigen::Vector6d> mXYAxes;

  /// The Z axis of the frame
  common::optional<Eigen::Vector3d> mZAxis;

  common::optional<std::string> mHField;

  common::optional<std::string> mMesh;

  double mFitScale{1};
};

Errors appendGeomAttributes(
    GeomAttributes& attributes, tinyxml2::XMLElement* element);

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart

#endif // #ifndef DART_IO_MJCF_DETAIL_GEOMATTRIBUTES_HPP_
