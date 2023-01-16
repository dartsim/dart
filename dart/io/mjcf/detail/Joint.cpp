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

#include "dart/io/mjcf/detail/Joint.hpp"

#include "dart/io/XmlHelpers.hpp"
#include "dart/io/mjcf/detail/Body.hpp"

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

//==============================================================================
Errors Joint::read(
    tinyxml2::XMLElement* element,
    const Defaults& defaults,
    const JointAttributes& defaultAttributes)
{
  Errors errors;

  if (std::string(element->Name()) != "joint") {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <Joint> from the provided element");
    return errors;
  }

  // Initialize the attributes from proper default
  if (hasAttribute(element, "class")) {
    const std::string className = getAttributeString(element, "class");
    const auto& defaultClass = defaults.getDefault(className);
    if (defaultClass) {
      mAttributes = defaultClass->getJointAttributes();
    } else {
      errors.push_back(Error(
          ErrorCode::ATTRIBUTE_INVALID,
          "Failed to find default with class name '" + className + "'"));
    }
  } else {
    mAttributes = defaultAttributes;
  }

  // Read attributes
  const Errors attrErrors = appendJointAttributes(mAttributes, element);
  errors.insert(errors.end(), attrErrors.begin(), attrErrors.end());

  return errors;
}

//==============================================================================
Errors Joint::preprocess(const Compiler& /*compiler*/)
{
  Errors errors;

  if (mAttributes.mName) {
    mName = *mAttributes.mName;
  }

  mType = mAttributes.mType;
  mPos = mAttributes.mPos;
  mAxis = mAttributes.mAxis;
  mRange = mAttributes.mRange;
  mSpringRef = mAttributes.mSpringRef;
  mDamping = mAttributes.mDamping;

  return errors;
}

//==============================================================================
Errors Joint::compile(const Compiler& /*compiler*/)
{
  Errors errors;
  return errors;
}

//==============================================================================
Errors Joint::postprocess(const Body* parent, const Compiler& compiler)
{
  Errors errors;

  if (compiler.getCoordinate() == Coordinate::LOCAL) {
    // Do nothing
  } else {
    if (parent != nullptr) {
      mPos = parent->getWorldTransform().inverse() * mPos;
      mAxis = parent->getWorldTransform().linear().transpose() * mAxis;
    } else {
      // Do nothing
    }
  }

  return errors;
}

//==============================================================================
const std::string& Joint::getName() const
{
  return mName;
}

//==============================================================================
JointType Joint::getType() const
{
  return mType;
}

//==============================================================================
const math::Vector3d& Joint::getPos() const
{
  return mPos;
}

//==============================================================================
const math::Vector3d& Joint::getAxis() const
{
  return mAxis;
}

//==============================================================================
bool Joint::isLimited() const
{
  return mLimited;
}

//==============================================================================
const math::Vector2d& Joint::getRange() const
{
  return mRange;
}

//==============================================================================
double Joint::getDamping() const
{
  return mDamping;
}

//==============================================================================
double Joint::getSpringRef() const
{
  return mSpringRef;
}

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart
