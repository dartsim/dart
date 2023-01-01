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

#include "dart/io/mjcf/detail/Default.hpp"

#include "dart/io/XmlHelpers.hpp"
#include "dart/io/mjcf/detail/Utils.hpp"

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

//==============================================================================
const GeomAttributes& Default::getGeomAttributes() const
{
  return mGeomAttributes;
}

//==============================================================================
const JointAttributes& Default::getJointAttributes() const
{
  return mJointAttributes;
}

//==============================================================================
const MeshAttributes& Default::getMeshAttributes() const
{
  return mMeshAttributes;
}

//==============================================================================
const WeldAttributes& Default::getWeldAttributes() const
{
  return mWeldAttributes;
}

//==============================================================================
Errors Default::read(tinyxml2::XMLElement* element, const Default* parent)
{
  Errors errors;

  if (std::string(element->Name()) != "default")
  {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <Default> from the provided element");
    return errors;
  }

  // Inherit from the parent
  if (parent != nullptr)
  {
    mGeomAttributes = parent->mGeomAttributes;
    mJointAttributes = parent->mJointAttributes;
    mMeshAttributes = parent->mMeshAttributes;
  }

  // Read <geom>
  if (hasElement(element, "geom"))
  {
    auto geomElement = getElement(element, "geom");
    const Errors geomErrors
        = appendGeomAttributes(mGeomAttributes, geomElement);
    errors.insert(errors.end(), geomErrors.begin(), geomErrors.end());
  }

  // Read <joint>
  if (hasElement(element, "joint"))
  {
    auto jointElement = getElement(element, "joint");
    const Errors jointErrors
        = appendJointAttributes(mJointAttributes, jointElement);
    errors.insert(errors.end(), jointErrors.begin(), jointErrors.end());
  }

  // Read <mesh>
  if (hasElement(element, "mesh"))
  {
    auto meshElement = getElement(element, "mesh");
    const Errors meshErrors
        = appendMeshAttributes(mMeshAttributes, meshElement);
    errors.insert(errors.end(), meshErrors.begin(), meshErrors.end());
  }

  // Read <mesh>
  if (hasElement(element, "equality"))
  {
    auto equalityElement = getElement(element, "equality");

    if (hasElement(equalityElement, "weld"))
    {
      auto weldElement = getElement(equalityElement, "weld");
      const Errors weldErrors
          = appendWeldAttributes(mWeldAttributes, weldElement);
      errors.insert(errors.end(), weldErrors.begin(), weldErrors.end());
    }
  }

  return errors;
}

//==============================================================================
bool Defaults::hasDefault(const std::string& className) const
{
  return static_cast<bool>(getDefault(className));
}

//==============================================================================
const Default* Defaults::getDefault(const std::string& className) const
{
  const auto result = mDefaultMap.find(className);

  if (result == mDefaultMap.end())
  {
    return nullptr;
  }

  return &(result->second);
}

//==============================================================================
const Default* Defaults::getRootDefault() const
{
  assert(hasDefault(mRootClassName));
  return getDefault(mRootClassName);
}

//==============================================================================
Errors Defaults::read(tinyxml2::XMLElement* element, const Default* parent)
{
  Errors errors;

  if (std::string(element->Name()) != "default")
  {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <default> from the provided element");
    return errors;
  }

  std::string className;
  if (hasAttribute(element, "class"))
  {
    className = getAttributeString(element, "class");

    if (parent == nullptr)
    {
      mRootClassName = className;
    }
  }
  else
  {
    if (parent != nullptr)
    {
      errors.push_back(Error(
          ErrorCode::ATTRIBUTE_MISSING,
          "Class name for non-root <default> is not specified."));
      return errors;
    }
  }

  auto newDefault = Default();
  const Errors defaultErrors = newDefault.read(element, parent);
  if (!defaultErrors.empty())
  {
    errors.insert(errors.end(), defaultErrors.begin(), defaultErrors.end());
    return errors;
  }
  mDefaultMap[className] = newDefault;

  // Read multiple <default>
  ElementEnumerator defaultElements(element, "default");
  while (defaultElements.next())
  {
    const Errors defaultErrors = read(defaultElements.get(), &newDefault);
    if (!defaultErrors.empty())
    {
      errors.insert(errors.end(), defaultErrors.begin(), defaultErrors.end());
      return errors;
    }
  }

  return errors;
}

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart
