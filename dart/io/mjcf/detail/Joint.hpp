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

#ifndef DART_UTILS_MJCF_DETAIL_JOINT_HPP_
#define DART_UTILS_MJCF_DETAIL_JOINT_HPP_

#include <dart/io/mjcf/detail/Compiler.hpp>
#include <dart/io/mjcf/detail/Default.hpp>
#include <dart/io/mjcf/detail/Error.hpp>
#include <dart/io/mjcf/detail/JointAttributes.hpp>
#include <dart/io/mjcf/detail/Types.hpp>

#include <tinyxml2.h>

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

class Body;

class DART_IO_API Joint final
{
public:
  Joint() = default;

  const std::string& getName() const;
  JointType getType() const;
  const math::Vector3d& getPos() const;
  const math::Vector3d& getAxis() const;
  bool isLimited() const;
  const math::Vector2d& getRange() const;
  double getDamping() const;
  double getSpringRef() const;

private:
  // Private members used by Body class
  friend class Body;
  Errors read(
      tinyxml2::XMLElement* element,
      const Defaults& defaults,
      const JointAttributes& defaultAttributes);

  /// Updates attributes and elements that doesn't require any other elements.
  Errors preprocess(const Compiler& compiler);

  /// Updates attributes and elements that require the preprocessed child
  /// elements of this <joint>.
  Errors compile(const Compiler& compiler);

  /// Updates attributes and elements that require the compiled parent element.
  Errors postprocess(const Body* body, const Compiler& compiler);

private:
  JointAttributes mAttributes;

  std::string mName{""};

  JointType mType{JointType::HINGE};

  int mGroup{0};

  /// Position of the joint, specified in local or global coordinates as
  /// determined by the coordinate attribute of compiler. For free joints this
  /// attribute is ignored.
  math::Vector3d mPos{math::Vector3d::Zero()};

  /// This attribute specifies the axis of rotation for hinge joints and the
  /// direction of translation for slide joints. It is ignored for free and ball
  /// joints.
  math::Vector3d mAxis{math::Vector3d::UnitZ()};

  math::Vector2d mSpringDamper{math::Vector2d::Zero()};

  /// This attribute specifies if the joint has limits.
  bool mLimited{false};

  double mStiffness{0};

  /// The joint limits.
  math::Vector2d mRange{math::Vector2d::Zero()};

  double mMargin{0};

  double mRef{0};

  double mSpringRef{0};

  double mArmature{0};

  double mDamping{0};

  double mFrictionLoss{0};

  math::VectorXd mUser;
};

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart

#endif // #ifndef DART_UTILS_MJCF_DETAIL_JOINT_HPP_
