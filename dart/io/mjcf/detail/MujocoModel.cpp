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

#include "dart/io/mjcf/detail/MujocoModel.hpp"

#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/io/CompositeResourceRetriever.hpp"
#include "dart/io/DartResourceRetriever.hpp"
#include "dart/io/XmlHelpers.hpp"
#include "dart/io/mjcf/detail/Utils.hpp"

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

//==============================================================================
Errors MujocoModel::read(
    tinyxml2::XMLElement* element,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  Errors errors;

  if (std::string(element->Name()) != "mujoco")
  {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <Mujoco> from the provided element");
    return errors;
  }

  // Handle <include>s
  const Errors includeErrors = handleInclude(element, baseUri, retriever);
  errors.insert(errors.end(), includeErrors.begin(), includeErrors.end());

  // Read 'model' attribute
  if (hasAttribute(element, "model"))
  {
    const std::string model = getAttributeString(element, "model");
    mModel = model;
  }

  // Read <compiler>
  if (hasElement(element, "compiler"))
  {
    auto compilerElement = getElement(element, "compiler");
    assert(compilerElement);
    const auto compilerErrors = mCompiler.read(compilerElement);
    errors.insert(errors.end(), compilerErrors.begin(), compilerErrors.end());
  }
  mCompiler.setBaseUri(baseUri);
  mCompiler.setResourceRetriever(retriever);

  // Read <option>
  if (hasElement(element, "option"))
  {
    auto optionElement = getElement(element, "option");
    assert(optionElement);
    const auto optionErrors = mOption.read(optionElement);
    errors.insert(errors.end(), optionErrors.begin(), optionErrors.end());
  }

  // Read <size>
  if (hasElement(element, "size"))
  {
    auto sizeElement = getElement(element, "size");
    assert(sizeElement);
    const auto sizeErrors = mSize.read(sizeElement);
    errors.insert(errors.end(), sizeErrors.begin(), sizeErrors.end());
  }

  // Read <asset>
  if (hasElement(element, "asset"))
  {
    auto assetElement = getElement(element, "asset");
    assert(assetElement);
    const auto assetErrors = mAsset.read(assetElement);
    errors.insert(errors.end(), assetErrors.begin(), assetErrors.end());
  }

  // Read <default>
  if (hasElement(element, "default"))
  {
    auto defaultElement = getElement(element, "default");
    assert(defaultElement);
    const auto defaultErrors = mDefaults.read(defaultElement, nullptr);
    errors.insert(errors.end(), defaultErrors.begin(), defaultErrors.end());
  }

  // Read <worldbody>
  if (hasElement(element, "worldbody"))
  {
    auto worldnodeElement = getElement(element, "worldbody");
    assert(worldnodeElement);
    mWorldbody = Worldbody();
    const auto worldbodyErrors = mWorldbody.read(
        worldnodeElement,
        mSize,
        mDefaults,
        mDefaults.getRootDefault(),
        baseUri,
        retriever);
    errors.insert(errors.end(), worldbodyErrors.begin(), worldbodyErrors.end());
  }

  const Errors assetPreprocessErrors = mAsset.preprocess(mCompiler);
  errors.insert(
      errors.end(), assetPreprocessErrors.begin(), assetPreprocessErrors.end());

  const Errors assetCompileErrors = mAsset.compile(mCompiler);
  errors.insert(
      errors.end(), assetCompileErrors.begin(), assetCompileErrors.end());

  const Errors assetPostprocessErrors = mAsset.postprocess(mCompiler);
  errors.insert(
      errors.end(),
      assetPostprocessErrors.begin(),
      assetPostprocessErrors.end());

  const Errors worldbodyPreprocessErrors = mWorldbody.preprocess(mCompiler);
  errors.insert(
      errors.end(),
      worldbodyPreprocessErrors.begin(),
      worldbodyPreprocessErrors.end());

  const Errors worldbodyCompileErrors = mWorldbody.compile(mCompiler);
  errors.insert(
      errors.end(),
      worldbodyCompileErrors.begin(),
      worldbodyCompileErrors.end());

  const Errors worldbodyPostprocessErrors = mWorldbody.postprocess(mCompiler);
  errors.insert(
      errors.end(),
      worldbodyPostprocessErrors.begin(),
      worldbodyPostprocessErrors.end());

  // Read <equality>
  if (hasElement(element, "equality"))
  {
    auto equalityElement = getElement(element, "equality");
    assert(equalityElement);
    const auto equalityErrors = mEquality.read(equalityElement, mDefaults);
    errors.insert(errors.end(), equalityErrors.begin(), equalityErrors.end());
  }

  return errors;
}

//==============================================================================
Errors MujocoModel::read(
    const common::Uri& uri, const common::ResourceRetrieverPtr& retrieverOrNull)
{
  Errors errors;

  // Create DART resource retriever if not passed
  common::ResourceRetrieverPtr retriever = retrieverOrNull;
  if (retriever == nullptr)
  {
    auto newRetriever = std::make_shared<io::CompositeResourceRetriever>();
    newRetriever->addSchemaRetriever(
        "file", std::make_shared<common::LocalResourceRetriever>());
    newRetriever->addSchemaRetriever("dart", DartResourceRetriever::create());
    retriever = std::move(newRetriever);
  }

  tinyxml2::XMLDocument mjcfDoc;
  if (!readXmlFile(mjcfDoc, uri, retriever))
  {
    errors.emplace_back(
        ErrorCode::FILE_READ, "Failed to load '" + uri.toString() + "'.");
    return errors;
  }

  // Get root <mujoco> element
  tinyxml2::XMLElement* mujocoElement = mjcfDoc.FirstChildElement("mujoco");
  if (mujocoElement == nullptr)
  {
    errors.emplace_back(
        ErrorCode::ELEMENT_MISSING, "Failed to find <mujoco> at the root");
    return errors;
  }

  // Parse <mujoco> element
  const Errors readErrors = read(mujocoElement, uri, retriever);
  errors.insert(errors.end(), readErrors.begin(), readErrors.end());

  return errors;
}

//==============================================================================
const std::string& MujocoModel::getModel() const
{
  return mModel;
}

//==============================================================================
const Compiler& MujocoModel::getCompiler() const
{
  return mCompiler;
}

//==============================================================================
const Option& MujocoModel::getOption() const
{
  return mOption;
}

//==============================================================================
const Size& MujocoModel::getSize() const
{
  return mSize;
}

//==============================================================================
const Asset& MujocoModel::getAsset() const
{
  return mAsset;
}

//==============================================================================
const Worldbody& MujocoModel::getWorldbody() const
{
  return mWorldbody;
}

//==============================================================================
const Equality& MujocoModel::getEquality() const
{
  return mEquality;
}

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart
