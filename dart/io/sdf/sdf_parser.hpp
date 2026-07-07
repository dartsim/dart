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

#ifndef DART_IO_SDFPARSER_HPP_
#define DART_IO_SDFPARSER_HPP_

#include <dart/dynamics/skeleton.hpp>

#include <dart/common/deprecated.hpp>
#include <dart/common/resource_retriever.hpp>

#include <dart/io/export.hpp>

#include <string>

namespace dart {
namespace io {

namespace SdfParser {

/// Root joint type to be used when the parent joint of the root link is not
/// specified in the URDF file.
enum class RootJointType
{
  /// Floating joint type of URDF.
  Floating = 0,

  /// Fixed joint type of URDF.
  Fixed = 1,
};

struct DART_IO_API Options
{
  /// Resource retriever. LocalResourceRetriever is used if it's nullptr.
  common::ResourceRetrieverPtr mResourceRetriever;

  /// Default root joint type to be used when the parent joint of the root
  /// link is not specified in the SDF model.
  RootJointType mDefaultRootJointType;

  /// Optional model name to load from a world document.
  ///
  /// Leave empty to keep the legacy behavior of loading a top-level model, or
  /// the first model in the first world when the document has no top-level
  /// model.
  std::string mModelName;

  /// Default constructor
  Options(
      common::ResourceRetrieverPtr resourceRetriever = nullptr,
      RootJointType defaultRootJointType = RootJointType::Floating,
      std::string modelName = {});
};

dynamics::SkeletonPtr DART_IO_API
readSkeleton(const common::Uri& uri, const Options& options = Options());

DART_DEPRECATED(6.12)
dynamics::SkeletonPtr DART_IO_API readSkeleton(
    const common::Uri& uri, const common::ResourceRetrieverPtr& retrievers);

} // namespace SdfParser

} // namespace io
} // namespace dart

#endif // #ifndef DART_IO_SDFPARSER_HPP_
