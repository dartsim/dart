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

#include "dart/common/macros.hpp"
#include "dart/io/dart_resource_retriever.hpp"
#include "dart/io/urdf/include_urdf.hpp"
#include "dart/io/urdf/urdf_parser.hpp"

#include <utility>

namespace dart {
namespace io {

using ModelInterfacePtr = std::shared_ptr<urdf::ModelInterface>;

//==============================================================================
UrdfParser::Options::Options(
    common::ResourceRetrieverPtr resourceRetriever,
    RootJointType defaultRootJointType,
    const dynamics::Inertia& defaultInertia)
  : mResourceRetriever(std::move(resourceRetriever)),
    mDefaultRootJointType(defaultRootJointType),
    mDefaultInertia(defaultInertia)
{
  // Do nothing
}

//==============================================================================
UrdfParser::UrdfParser(const Options& options)
  : mOptions(options),
    mLocalRetriever(new common::LocalResourceRetriever),
    mPackageRetriever(new io::PackageResourceRetriever(mLocalRetriever)),
    mRetriever(new io::CompositeResourceRetriever)
{
  mRetriever->addSchemaRetriever("file", mLocalRetriever);
  mRetriever->addSchemaRetriever("package", mPackageRetriever);
  mRetriever->addSchemaRetriever("dart", DartResourceRetriever::create());
}

//==============================================================================
UrdfParser::~UrdfParser() = default;

//==============================================================================
void UrdfParser::setOptions(const Options& options)
{
  mOptions = options;
}

//==============================================================================
const UrdfParser::Options& UrdfParser::getOptions() const
{
  return mOptions;
}

//==============================================================================
void UrdfParser::addPackageDirectory(
    std::string_view packageName, std::string_view packageDirectory)
{
  mPackageRetriever->addPackageDirectory(packageName, packageDirectory);
}

//==============================================================================
dynamics::SkeletonPtr UrdfParser::parseSkeleton(const common::Uri& uri)
{
  const common::ResourceRetrieverPtr resourceRetriever
      = getResourceRetriever(mOptions.mResourceRetriever);

  std::string content;
  if (!readFileToString(resourceRetriever, uri, content)) {
    return nullptr;
  }

  const ModelInterfacePtr urdfInterface = urdf::parseURDF(content);
  if (!urdfInterface) {
    DART_WARN("Failed loading URDF file '{}'.", uri.toString());
    return nullptr;
  }

  ParseContext context;
  context.mTransmissions = parseTransmissions(content);

  return modelInterfaceToSkeleton(
      urdfInterface.get(), uri, resourceRetriever, mOptions, &context);
}

//==============================================================================
dynamics::SkeletonPtr UrdfParser::parseSkeletonString(
    std::string_view urdfString, const common::Uri& baseUri)
{
  if (urdfString.empty()) {
    DART_WARN(
        "A blank string cannot be parsed into a Skeleton. Returning a nullptr");
    return nullptr;
  }

  const std::string urdfStringCopy(urdfString);
  ModelInterfacePtr urdfInterface = urdf::parseURDF(urdfStringCopy);
  if (!urdfInterface) {
    DART_WARN("Failed loading URDF.");
    return nullptr;
  }

  ParseContext context;
  context.mTransmissions = parseTransmissions(urdfStringCopy);

  return modelInterfaceToSkeleton(
      urdfInterface.get(),
      baseUri,
      getResourceRetriever(mOptions.mResourceRetriever),
      mOptions,
      &context);
}

} // namespace io
} // namespace dart
