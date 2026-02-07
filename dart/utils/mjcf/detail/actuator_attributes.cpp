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

#include "dart/utils/mjcf/detail/actuator_attributes.hpp"

#include "dart/utils/xml_helpers.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

//==============================================================================
Errors appendActuatorAttributes(
    ActuatorAttributes& attributes, tinyxml2::XMLElement* element)
{
  Errors errors;

  const std::string elementName = element->Name();
  if (elementName == "motor") {
    attributes.mType = ActuatorType::MOTOR;
  } else if (elementName == "position") {
    attributes.mType = ActuatorType::POSITION;
  } else if (elementName == "velocity") {
    attributes.mType = ActuatorType::VELOCITY;
  } else if (elementName == "general") {
    attributes.mType = ActuatorType::GENERAL;
  } else {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <motor>, <position>, <velocity>, or <general> from the "
        "provided element");
    return errors;
  }

  // (optional) name
  if (hasAttribute(element, "name")) {
    attributes.mName = getAttributeString(element, "name");
  }

  // joint (optional in <default> context, required on actual actuator elements)
  if (hasAttribute(element, "joint")) {
    attributes.mJoint = getAttributeString(element, "joint");
  }

  // ctrllimited
  if (hasAttribute(element, "ctrllimited")) {
    const std::string ctrlLimited = getAttributeString(element, "ctrllimited");
    if (ctrlLimited == "true" || ctrlLimited == "1") {
      attributes.mCtrlLimited = true;
    } else if (ctrlLimited == "false" || ctrlLimited == "0") {
      attributes.mCtrlLimited = false;
    } else if (ctrlLimited == "auto") {
      attributes.mCtrlLimited = std::nullopt;
    } else {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID,
          "Invalid attribute for 'ctrllimited': " + ctrlLimited);
      return errors;
    }
  }

  // ctrlrange
  if (hasAttribute(element, "ctrlrange")) {
    attributes.mCtrlRange = getAttributeVector2d(element, "ctrlrange");
  }

  // forcelimited
  if (hasAttribute(element, "forcelimited")) {
    const std::string forceLimited
        = getAttributeString(element, "forcelimited");
    if (forceLimited == "true" || forceLimited == "1") {
      attributes.mForceLimited = true;
    } else if (forceLimited == "false" || forceLimited == "0") {
      attributes.mForceLimited = false;
    } else if (forceLimited == "auto") {
      attributes.mForceLimited = std::nullopt;
    } else {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID,
          "Invalid attribute for 'forcelimited': " + forceLimited);
      return errors;
    }
  }

  // forcerange
  if (hasAttribute(element, "forcerange")) {
    attributes.mForceRange = getAttributeVector2d(element, "forcerange");
  }

  // gear
  if (hasAttribute(element, "gear")) {
    const Eigen::VectorXd gearVec = getAttributeVectorXd(element, "gear");
    attributes.mGear = Eigen::Vector6d::Zero();
    const Eigen::Index n
        = std::min(gearVec.size(), static_cast<Eigen::Index>(6));
    attributes.mGear.head(n) = gearVec.head(n);
  }

  // gainprm
  if (hasAttribute(element, "gainprm")) {
    attributes.mGainPrm = getAttributeVector3d(element, "gainprm");
  }

  // biasprm
  if (hasAttribute(element, "biasprm")) {
    attributes.mBiasPrm = getAttributeVector3d(element, "biasprm");
  }

  return errors;
}

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart
