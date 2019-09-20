/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#include "dart/utils/mjcf/detail/Worldbody.hpp"

#include "dart/utils/XmlHelpers.hpp"
#include "dart/utils/mjcf/detail/Compiler.hpp"
#include "dart/utils/mjcf/detail/Size.hpp"
#include "dart/utils/mjcf/detail/Utils.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

//==============================================================================
Errors Worldbody::read(
    tinyxml2::XMLElement* element,
    const common::optional<Size>& size,
    const Defaults& defaults,
    const Default* currentDefault,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  assert(currentDefault != nullptr);

  Errors errors;

  if (std::string(element->Name()) != "worldbody")
  {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <worldbody> from the provided element");
    return errors;
  }

  // childclass
  if (hasAttribute(element, "childclass"))
  {
    const std::string className = getAttributeString(element, "childclass");
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

  // Handle multiple <include>
  const Errors includeErrors = handleInclude(element, baseUri, retriever);
  errors.insert(errors.end(), includeErrors.begin(), includeErrors.end());

  // Read multiple <geom>
  ElementEnumerator geomElements(element, "geom");
  while (geomElements.next())
  {
    Geom geom = Geom();
    const auto geomErrors = geom.read(
        geomElements.get(), defaults, currentDefault->getGeomAttributes());
    errors.insert(errors.end(), geomErrors.begin(), geomErrors.end());

    if (geomErrors.empty())
    {
      mGeoms.emplace_back(std::move(geom));
    }
  }

  // Read multiple <site>
  ElementEnumerator siteElements(element, "site");
  while (siteElements.next())
  {
    Site site = Site();
    const auto siteErrors = site.read(geomElements.get());
    errors.insert(errors.end(), siteErrors.begin(), siteErrors.end());

    if (siteErrors.empty())
    {
      mSites.emplace_back(std::move(site));
    }
  }

  // Read multiple <body>
  ElementEnumerator bodyElements(element, "body");
  while (bodyElements.next())
  {
    Body rootBody = Body();
    const auto bodyErrors
        = rootBody.read(bodyElements.get(), size, defaults, currentDefault);
    errors.insert(errors.end(), bodyErrors.begin(), bodyErrors.end());

    if (bodyErrors.empty())
    {
      mRootBodies.emplace_back(std::move(rootBody));
    }
  }

  return errors;
}

//==============================================================================
Errors Worldbody::preprocess(const Compiler& compiler)
{
  Errors errors;

  for (Geom& geom : mGeoms)
  {
    const Errors geomErrors = geom.preprocess(compiler);
    errors.insert(errors.end(), geomErrors.begin(), geomErrors.end());
  }

  for (Body& body : mRootBodies)
  {
    const Errors bodyErrors = body.preprocess(compiler);
    errors.insert(errors.end(), bodyErrors.begin(), bodyErrors.end());
  }

  return errors;
}

//==============================================================================
Errors Worldbody::compile(const Compiler& compiler)
{
  Errors errors;

  for (Geom& geom : mGeoms)
  {
    const Errors geomErrors = geom.compile(compiler);
    errors.insert(errors.end(), geomErrors.begin(), geomErrors.end());
  }

  for (Body& body : mRootBodies)
  {
    const Errors bodyErrors = body.compile(compiler);
    errors.insert(errors.end(), bodyErrors.begin(), bodyErrors.end());
  }

  return errors;
}

//==============================================================================
Errors Worldbody::postprocess(const Compiler& compiler)
{
  Errors errors;

  for (Geom& geom : mGeoms)
  {
    const Errors geomErrors = geom.postprocess(nullptr, compiler);
    errors.insert(errors.end(), geomErrors.begin(), geomErrors.end());
  }

  for (Body& body : mRootBodies)
  {
    const Errors bodyErrors = body.postprocess(nullptr, compiler);
    errors.insert(errors.end(), bodyErrors.begin(), bodyErrors.end());
  }

  return errors;
}

//==============================================================================
std::size_t Worldbody::getNumGeoms() const
{
  return mGeoms.size();
}

//==============================================================================
const Geom& Worldbody::getGeom(std::size_t index) const
{
  return mGeoms[index];
}

//==============================================================================
std::size_t Worldbody::getNumSites() const
{
  return mSites.size();
}

//==============================================================================
const Site& Worldbody::getSite(std::size_t index) const
{
  return mSites[index];
}

//==============================================================================
std::size_t Worldbody::getNumRootBodies() const
{
  return mRootBodies.size();
}

//==============================================================================
const Body& Worldbody::getRootBody(std::size_t index) const
{
  return mRootBodies[index];
}

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart
