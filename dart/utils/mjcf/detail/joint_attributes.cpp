/*
 * Copyright (c) 2011, The DART development contributors
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

#include "dart/utils/mjcf/detail/joint_attributes.hpp"

#include "dart/utils/mjcf/detail/body.hpp"
#include "dart/utils/xml_helpers.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

//==============================================================================
Errors appendJointAttributes(
    JointAttributes& attributes, tinyxml2::XMLElement* element)
{
  Errors errors;

  if (std::string(element->Name()) != "joint") {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <joint> from the provided element");
    return errors;
  }

  // (optional) name
  if (hasAttribute(element, "name")) {
    const std::string name = getAttributeString(element, "name");
    attributes.mName = name;
  }

  // type
  if (hasAttribute(element, "type")) {
    const std::string type = getAttributeString(element, "type");
    if (type == "free") {
      attributes.mType = JointType::FREE;
    } else if (type == "ball") {
      attributes.mType = JointType::BALL;
    } else if (type == "slide") {
      attributes.mType = JointType::SLIDE;
    } else if (type == "hinge") {
      attributes.mType = JointType::HINGE;
    } else {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID,
          "Invalid attribute for 'type': " + type);
      return errors;
    }
  }

  // pos
  if (hasAttribute(element, "pos")) {
    attributes.mPos = getAttributeVector3d(element, "pos");
  }

  // axis
  if (hasAttribute(element, "axis")) {
    attributes.mAxis = getAttributeVector3d(element, "axis").normalized();
  }

  // range
  if (hasAttribute(element, "range")) {
    attributes.mRange = getAttributeVector2d(element, "range");
  }

  // springref
  if (hasAttribute(element, "springref")) {
    attributes.mSpringRef = getAttributeDouble(element, "springref");
  }

  // damping
  if (hasAttribute(element, "damping")) {
    attributes.mDamping = getAttributeDouble(element, "damping");
  }

  // stiffness
  if (hasAttribute(element, "stiffness")) {
    attributes.mStiffness = getAttributeDouble(element, "stiffness");
  }

  // armature
  if (hasAttribute(element, "armature")) {
    attributes.mArmature = getAttributeDouble(element, "armature");
  }

  // frictionloss
  if (hasAttribute(element, "frictionloss")) {
    attributes.mFrictionLoss = getAttributeDouble(element, "frictionloss");
  }

  // limited
  if (hasAttribute(element, "limited")) {
    const std::string limited = getAttributeString(element, "limited");
    if (limited == "true" || limited == "1") {
      attributes.mLimited = true;
    } else if (limited == "false" || limited == "0") {
      attributes.mLimited = false;
    } else if (limited == "auto") {
      attributes.mLimited = std::nullopt;
    } else {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID,
          "Invalid attribute for 'limited': " + limited);
      return errors;
    }
  }

  return errors;
}

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart
