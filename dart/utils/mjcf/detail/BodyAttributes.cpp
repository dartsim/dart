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

#include "dart/utils/mjcf/detail/BodyAttributes.hpp"

#include "dart/utils/XmlHelpers.hpp"
#include "dart/utils/mjcf/detail/Size.hpp"
#include "dart/utils/mjcf/detail/Utils.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

//==============================================================================
Errors appendBodyAttributes(
    BodyAttributes& attributes,
    tinyxml2::XMLElement* element,
    const std::optional<Size>& size)
{
  Errors errors;

  if (std::string(element->Name()) != "body")
  {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <Body> from the provided element");
    return errors;
  }

  // name
  if (hasAttribute(element, "name"))
  {
    attributes.mName = getAttributeString(element, "name");
  }

  // mocap
  if (hasAttribute(element, "mocap"))
  {
    attributes.mMocap = getAttributeBool(element, "mocap");
  }

  // pos
  if (hasAttribute(element, "pos"))
  {
    attributes.mPos = getAttributeVector3d(element, "pos");
  }

  // Check if multiple orientation representations present
  const Errors orientationErrors = checkOrientationValidity(element);
  errors.insert(
      errors.end(), orientationErrors.begin(), orientationErrors.end());

  // quat
  if (hasAttribute(element, "quat"))
  {
    const Eigen::Vector4d vec4d = getAttributeVector4d(element, "quat");
    attributes.mQuat.w() = vec4d[0];
    attributes.mQuat.x() = vec4d[1];
    attributes.mQuat.y() = vec4d[2];
    attributes.mQuat.z() = vec4d[3];
  }

  // axisangle
  if (hasAttribute(element, "axisangle"))
  {
    attributes.mAxisAngle = getAttributeVector4d(element, "axisangle");
  }

  // euler
  if (hasAttribute(element, "euler"))
  {
    attributes.mEuler = getAttributeVector3d(element, "euler");
  }

  // xyaxes
  if (hasAttribute(element, "xyaxes"))
  {
    attributes.mXYAxes = getAttributeVector6d(element, "xyaxes");
  }

  // zaxis
  if (hasAttribute(element, "zaxis"))
  {
    attributes.mZAxis = getAttributeVector3d(element, "zaxis");
  }

  // user
  if (hasAttribute(element, "user"))
  {
    if (!size)
    {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID,
          "Attempt to parse 'user' attribute of <body> when <size> is missing");
      return errors;
    }

    const Eigen::VectorXd user = getAttributeVectorXd(element, "user");

    if (user.size() != size->getNUserBody())
    {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID,
          "The size of 'user' is different from <size nuser_body="
              + std::to_string(size->getNUserBody()) + ">");
      return errors;
    }

    attributes.mUser = user;
  }
  else
  {
    if (size)
      attributes.mUser.setZero(size->getNUserBody());
    else
      attributes.mUser.resize(0);
  }

  // Read <inertial>
  if (hasElement(element, "inertial"))
  {
    auto inertialElement = getElement(element, "inertial");
    assert(inertialElement);
    attributes.mInertial = Inertial();
    const Errors inertialErrors = attributes.mInertial->read(inertialElement);
    errors.insert(errors.end(), inertialErrors.begin(), inertialErrors.end());
  }

  return errors;
}

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart
