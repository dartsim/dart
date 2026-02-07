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

#include "dart/utils/mjcf/detail/contact.hpp"

#include "dart/utils/mjcf/detail/utils.hpp"
#include "dart/utils/xml_helpers.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

//==============================================================================
std::size_t Contact::getNumPairs() const
{
  return mPairs.size();
}

//==============================================================================
const Contact::Pair& Contact::getPair(std::size_t index) const
{
  return mPairs[index];
}

//==============================================================================
std::size_t Contact::getNumExcludes() const
{
  return mExcludes.size();
}

//==============================================================================
const Contact::Exclude& Contact::getExclude(std::size_t index) const
{
  return mExcludes[index];
}

//==============================================================================
Errors Contact::read(tinyxml2::XMLElement* element)
{
  Errors errors;

  if (std::string(element->Name()) != "contact") {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <Contact> from the provided element");
    return errors;
  }

  ElementEnumerator pairElements(element, "pair");
  while (pairElements.next()) {
    Pair pair;

    if (hasAttribute(pairElements.get(), "geom1")) {
      pair.mGeom1 = getAttributeString(pairElements.get(), "geom1");
    } else {
      errors.push_back(Error(
          ErrorCode::ATTRIBUTE_MISSING,
          "Failed to find required attribute 'geom1' in <pair>."));
    }

    if (hasAttribute(pairElements.get(), "geom2")) {
      pair.mGeom2 = getAttributeString(pairElements.get(), "geom2");
    } else {
      errors.push_back(Error(
          ErrorCode::ATTRIBUTE_MISSING,
          "Failed to find required attribute 'geom2' in <pair>."));
    }

    if (hasAttribute(pairElements.get(), "condim")) {
      pair.mCondim = getAttributeInt(pairElements.get(), "condim");
    }

    if (hasAttribute(pairElements.get(), "friction")) {
      pair.mFriction = getAttributeVectorXd(pairElements.get(), "friction");
    }

    if (hasAttribute(pairElements.get(), "margin")) {
      pair.mMargin = getAttributeDouble(pairElements.get(), "margin");
    }

    if (hasAttribute(pairElements.get(), "gap")) {
      pair.mGap = getAttributeDouble(pairElements.get(), "gap");
    }

    mPairs.emplace_back(std::move(pair));
  }

  ElementEnumerator excludeElements(element, "exclude");
  while (excludeElements.next()) {
    Exclude exclude;

    if (hasAttribute(excludeElements.get(), "body1")) {
      exclude.mBody1 = getAttributeString(excludeElements.get(), "body1");
    } else {
      errors.push_back(Error(
          ErrorCode::ATTRIBUTE_MISSING,
          "Failed to find required attribute 'body1' in <exclude>."));
    }

    if (hasAttribute(excludeElements.get(), "body2")) {
      exclude.mBody2 = getAttributeString(excludeElements.get(), "body2");
    } else {
      errors.push_back(Error(
          ErrorCode::ATTRIBUTE_MISSING,
          "Failed to find required attribute 'body2' in <exclude>."));
    }

    mExcludes.emplace_back(std::move(exclude));
  }

  warnUnknownElements(element, {"pair", "exclude"});

  return errors;
}

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart
