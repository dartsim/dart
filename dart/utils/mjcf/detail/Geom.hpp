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

#ifndef DART_UTILS_MJCF_DETAIL_GEOM_HPP_
#define DART_UTILS_MJCF_DETAIL_GEOM_HPP_

#include <tinyxml2.h>

#include "dart/utils/mjcf/detail/Compiler.hpp"
#include "dart/utils/mjcf/detail/Default.hpp"
#include "dart/utils/mjcf/detail/Error.hpp"
#include "dart/utils/mjcf/detail/GeomAttributes.hpp"
#include "dart/utils/mjcf/detail/Types.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

class Body;

class Geom final
{
public:
  /// Default constructor
  Geom() = default;

  /// \{ \name Attributes

  const std::string& getName() const;
  GeomType getType() const;
  int getConType() const;
  int getConAffinity() const;
  int getConDim() const;
  int getGroup() const;
  int getPriority() const;

  const Eigen::Vector3d& getSize() const;

  Eigen::Vector2d getPlaneHalfSize() const;

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

  double getMass() const;
  double getDensity() const;
  double getVolume() const;
  const Eigen::Matrix3d& getInertia() const;

  double getSolMix() const;
  double getMargine() const;
  double getGap() const;

  void setRelativeTransform(const Eigen::Isometry3d& tf);
  const Eigen::Isometry3d& getRelativeTransform() const;

  void setWorldTransform(const Eigen::Isometry3d& tf);
  const Eigen::Isometry3d& getWorldTransform() const;

  const std::string& getHField() const;
  const std::string& getMesh() const;

  /// \}

private:
  // Private members used by Body and WorldBody class
  friend class Body;
  friend class Worldbody;
  Errors read(
      tinyxml2::XMLElement* element,
      const Defaults& defaults,
      const GeomAttributes& defaultAttributes);

  /// Updates attributes and elements that doesn't require any other elements.
  Errors preprocess(const Compiler& compiler);

  /// Updates attributes and elements that require the preprocessed child
  /// elements of this <geom>.
  Errors compile(const Compiler& compiler);

  /// Updates attributes and elements that require the compiled parent element.
  Errors postprocess(const Body* body, const Compiler& compiler);

private:
  double computeVolume() const;
  Eigen::Matrix3d computeInertia() const;

  GeomAttributes mAttributes;

  /// Name of the geom
  std::string mName{""};

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

  double mMass;

  /// Material density used to compute the geom mass and inertia.
  double mDensity{1000};

  double mVolume{0};

  Eigen::Matrix3d mInertia{Eigen::Matrix3d::Identity()};

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

  Eigen::Isometry3d mRelativeTransform{Eigen::Isometry3d::Identity()};

  Eigen::Isometry3d mWorldTransform{Eigen::Isometry3d::Identity()};

  std::string mHField;
  std::string mMesh;

  double mFitScale{1};
};

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_MJCF_DETAIL_GEOM_HPP_
