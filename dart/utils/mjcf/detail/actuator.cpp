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

#include "dart/utils/mjcf/detail/actuator.hpp"

#include "dart/utils/mjcf/detail/utils.hpp"
#include "dart/utils/xml_helpers.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

//==============================================================================
std::size_t Actuator::getNumEntries() const
{
  return mEntries.size();
}

//==============================================================================
const Actuator::Entry& Actuator::getEntry(std::size_t index) const
{
  return mEntries[index];
}

//==============================================================================
Errors Actuator::read(tinyxml2::XMLElement* element)
{
  Errors errors;

  if (std::string(element->Name()) != "actuator") {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <Actuator> from the provided element");
    return errors;
  }

  auto parseEntry = [&](tinyxml2::XMLElement* child,
                        ActuatorType type,
                        bool allowGainBias) {
    Entry entry;
    entry.mType = type;

    if (hasAttribute(child, "name")) {
      entry.mName = getAttributeString(child, "name");
    }

    if (hasAttribute(child, "joint")) {
      entry.mJoint = getAttributeString(child, "joint");
    } else {
      errors.push_back(Error(
          ErrorCode::ATTRIBUTE_MISSING,
          "Failed to find required attribute 'joint' in actuator element."));
    }

    if (hasAttribute(child, "ctrllimited")) {
      const std::string ctrlLimited = getAttributeString(child, "ctrllimited");
      if (ctrlLimited == "true" || ctrlLimited == "1") {
        entry.mCtrlLimited = true;
      } else if (ctrlLimited == "false" || ctrlLimited == "0") {
        entry.mCtrlLimited = false;
      } else {
        errors.emplace_back(
            ErrorCode::ATTRIBUTE_INVALID,
            "Invalid attribute for 'ctrllimited': " + ctrlLimited);
      }
    }

    if (hasAttribute(child, "ctrlrange")) {
      entry.mCtrlRange = getAttributeVector2d(child, "ctrlrange");
    }

    if (hasAttribute(child, "forcelimited")) {
      const std::string forceLimited
          = getAttributeString(child, "forcelimited");
      if (forceLimited == "true" || forceLimited == "1") {
        entry.mForceLimited = true;
      } else if (forceLimited == "false" || forceLimited == "0") {
        entry.mForceLimited = false;
      } else {
        errors.emplace_back(
            ErrorCode::ATTRIBUTE_INVALID,
            "Invalid attribute for 'forcelimited': " + forceLimited);
      }
    }

    if (hasAttribute(child, "forcerange")) {
      entry.mForceRange = getAttributeVector2d(child, "forcerange");
    }

    if (hasAttribute(child, "gear")) {
      entry.mGear = getAttributeDouble(child, "gear");
    }

    if (allowGainBias) {
      if (hasAttribute(child, "gainprm")) {
        entry.mGainPrm = getAttributeVector3d(child, "gainprm");
      }

      if (hasAttribute(child, "biasprm")) {
        entry.mBiasPrm = getAttributeVector3d(child, "biasprm");
      }
    }

    mEntries.emplace_back(std::move(entry));
  };

  ElementEnumerator motorElements(element, "motor");
  while (motorElements.next()) {
    parseEntry(motorElements.get(), ActuatorType::MOTOR, false);
  }

  ElementEnumerator positionElements(element, "position");
  while (positionElements.next()) {
    parseEntry(positionElements.get(), ActuatorType::POSITION, false);
  }

  ElementEnumerator velocityElements(element, "velocity");
  while (velocityElements.next()) {
    parseEntry(velocityElements.get(), ActuatorType::VELOCITY, false);
  }

  ElementEnumerator generalElements(element, "general");
  while (generalElements.next()) {
    parseEntry(generalElements.get(), ActuatorType::GENERAL, true);
  }

  warnUnknownElements(element, {"motor", "position", "velocity", "general"});

  return errors;
}

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart
