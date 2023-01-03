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

#include "dart/io/mjcf/detail/Weld.hpp"

#include "dart/io/XmlHelpers.hpp"

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

//==============================================================================
const std::string& Weld::getName() const
{
  return mName;
}

//==============================================================================
bool Weld::getActive() const
{
  return mActive;
}

//==============================================================================
const Eigen::Vector2d& Weld::getSolRef() const
{
  return mSolRef;
}

//==============================================================================
const Eigen::Matrix<double, 5, 1>& Weld::getSolImp() const
{
  return mSolImp;
}

//==============================================================================
const std::string& Weld::getBody1() const
{
  return mBody1;
}

//==============================================================================
const std::string& Weld::getBody2() const
{
  return mBody2;
}

//==============================================================================
const std::optional<Eigen::Isometry3d>& Weld::getRelativeTransform() const
{
  return mRelativeTransfrom;
}

//==============================================================================
Errors Weld::read(tinyxml2::XMLElement* element, const Defaults& defaults)
{
  Errors errors;

  if (std::string(element->Name()) != "weld")
  {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <weld> from the provided element");
    return errors;
  }

  const Default* currentDefault = nullptr;

  // Read 'class' attribute
  if (hasAttribute(element, "class"))
  {
    const std::string className = getAttributeString(element, "class");
    const auto& defaultClass = defaults.getDefault(className);
    if (defaultClass)
    {
      currentDefault = &(*defaultClass);
    }
    else
    {
      errors.push_back(Error(
          ErrorCode::ATTRIBUTE_INVALID,
          "Failed to find default with childclass name '" + className + "'"));
    }
  }
  else
  {
    currentDefault = defaults.getRootDefault();
  }
  assert(currentDefault != nullptr);

  mAttributes = currentDefault->getWeldAttributes();

  // Read attributes
  const Errors attrErrors = appendWeldAttributes(mAttributes, element);
  errors.insert(errors.end(), attrErrors.begin(), attrErrors.end());

  if (mAttributes.mName)
  {
    mName = *mAttributes.mName;
  }
  mActive = mAttributes.mActive;
  mSolRef = mAttributes.mSolRef;
  mSolImp = mAttributes.mSolImp;
  mBody1 = mAttributes.mBody1;
  if (mAttributes.mBody2)
  {
    mBody2 = *mAttributes.mBody2;
  }
  if (!mAttributes.mRelPose.tail<4>().isApprox(Eigen::Vector4d::Zero()))
  {
    mRelativeTransfrom = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond quat;
    quat.w() = mAttributes.mRelPose[3];
    quat.x() = mAttributes.mRelPose[4];
    quat.y() = mAttributes.mRelPose[5];
    quat.z() = mAttributes.mRelPose[6];
    mRelativeTransfrom->linear() = quat.normalized().toRotationMatrix();
    mRelativeTransfrom->translation() = mAttributes.mRelPose.head<3>();
  }

  return errors;
}

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart
