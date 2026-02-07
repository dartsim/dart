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

#include "dart/utils/mjcf/detail/actuator_attributes.hpp"
#include "dart/utils/mjcf/detail/default.hpp"
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
Errors Actuator::read(tinyxml2::XMLElement* element, const Defaults& defaults)
{
  Errors errors;

  if (std::string(element->Name()) != "actuator") {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <Actuator> from the provided element");
    return errors;
  }

  const Default* rootDefault = defaults.getRootDefault();

  auto parseEntry = [&](tinyxml2::XMLElement* child, ActuatorType type) {
    ActuatorAttributes attrs;
    if (hasAttribute(child, "class")) {
      const std::string className = getAttributeString(child, "class");
      const Default* cls = defaults.getDefault(className);
      if (cls != nullptr) {
        attrs = cls->getActuatorAttributes(type);
      } else {
        errors.push_back(Error(
            ErrorCode::ATTRIBUTE_INVALID,
            "Failed to find default with class name '" + className + "'"));
      }
    } else if (rootDefault != nullptr) {
      attrs = rootDefault->getActuatorAttributes(type);
    }

    const Errors appendErrors = appendActuatorAttributes(attrs, child);
    errors.insert(errors.end(), appendErrors.begin(), appendErrors.end());

    if (attrs.mJoint.empty()) {
      errors.push_back(Error(
          ErrorCode::ATTRIBUTE_MISSING,
          "Failed to find required attribute 'joint' in actuator element."));
    }

    Entry entry;
    entry.mName = attrs.mName.value_or("");
    entry.mJoint = attrs.mJoint;
    entry.mType = type;
    entry.mCtrlRange = attrs.mCtrlRange;
    entry.mForceRange = attrs.mForceRange;
    entry.mGear = attrs.mGear;
    entry.mGainPrm = attrs.mGainPrm;
    entry.mBiasPrm = attrs.mBiasPrm;

    entry.mCtrlLimited = attrs.mCtrlLimited.has_value()
                             ? attrs.mCtrlLimited.value()
                             : !entry.mCtrlRange.isZero();

    entry.mForceLimited = attrs.mForceLimited.has_value()
                              ? attrs.mForceLimited.value()
                              : !entry.mForceRange.isZero();

    mEntries.emplace_back(std::move(entry));
  };

  ElementEnumerator motorElements(element, "motor");
  while (motorElements.next()) {
    parseEntry(motorElements.get(), ActuatorType::MOTOR);
  }

  ElementEnumerator positionElements(element, "position");
  while (positionElements.next()) {
    parseEntry(positionElements.get(), ActuatorType::POSITION);
  }

  ElementEnumerator velocityElements(element, "velocity");
  while (velocityElements.next()) {
    parseEntry(velocityElements.get(), ActuatorType::VELOCITY);
  }

  ElementEnumerator generalElements(element, "general");
  while (generalElements.next()) {
    parseEntry(generalElements.get(), ActuatorType::GENERAL);
  }

  warnUnknownElements(element, {"motor", "position", "velocity", "general"});

  return errors;
}

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart
