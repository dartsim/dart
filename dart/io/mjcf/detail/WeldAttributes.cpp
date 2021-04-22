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

#include "dart/io/mjcf/detail/WeldAttributes.hpp"

#include "dart/io/XmlHelpers.hpp"

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

//==============================================================================
WeldAttributes::WeldAttributes()
{
  mSolImp << 0.9, 0.95, 0.001, 0.5, 2;
  mSolRef << 0.02, 1;
  mRelPose << 0, 1, 0, 0, 0, 0, 0;
}

//==============================================================================
Errors appendWeldAttributes(
    WeldAttributes& attributes, tinyxml2::XMLElement* element)
{
  Errors errors;

  if (std::string(element->Name()) != "weld")
  {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <weld> from the provided element");
    return errors;
  }

  // (optional) name
  if (hasAttribute(element, "name"))
  {
    const std::string name = getAttributeString(element, "name");
    attributes.mName = name;
  }

  // active
  if (hasAttribute(element, "active"))
  {
    attributes.mActive = getAttributeBool(element, "active");
  }

  // solimp
  if (hasAttribute(element, "solimp"))
  {
    const Eigen::VectorXd solimp = getAttributeVectorXd(element, "solimp");
    if (solimp.size() == 0 || solimp.size() > 5)
    {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID,
          "Invalid dimension for 'solimp' attribute");
      return errors;
    }
    attributes.mSolImp.head(solimp.size()) = solimp;
  }

  // solref
  if (hasAttribute(element, "solref"))
  {
    attributes.mSolRef = getAttributeVector2d(element, "solref");
  }

  // body1
  if (hasAttribute(element, "body1"))
  {
    attributes.mBody1 = getAttributeString(element, "body1");
  }
  else
  {
    errors.push_back(Error(
        ErrorCode::ATTRIBUTE_MISSING,
        "Failed to find required attribute 'body1' in <weld>."));
  }

  // body2
  if (hasAttribute(element, "body2"))
  {
    attributes.mBody2 = getAttributeString(element, "body2");
  }

  // relpose
  if (hasAttribute(element, "relpose"))
  {
    attributes.mRelPose = getAttributeVectorNd<7>(element, "relpose");
  }

  return errors;
}

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart
