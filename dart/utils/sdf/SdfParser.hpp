/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#ifndef DART_UTILS_SDFPARSER_HPP_
#define DART_UTILS_SDFPARSER_HPP_

#include <dart/simulation/World.hpp>

#include <dart/dynamics/Skeleton.hpp>

#include <dart/common/Deprecated.hpp>
#include <dart/common/ResourceRetriever.hpp>

namespace dart {
namespace utils {

namespace SdfParser {

/// Root joint type to be used when the parent joint of the root link is not
/// specified in the URDF file.
enum class RootJointType
{
  /// Floating joint type of URDF.
  FLOATING = 0,

  /// Fixed joint type of URDF.
  FIXED = 1,
};

struct Options
{
  /// Resource retriever. LocalResourceRetriever is used if it's nullptr.
  common::ResourceRetrieverPtr mResourceRetriever;

  /// Default root joint type to be used when the parent joint of the root
  /// link is not specified in the URDF file.
  RootJointType mDefaultRootJointType;

  /// Default constructor
  Options(
      common::ResourceRetrieverPtr resourceRetriever = nullptr,
      RootJointType defaultRootJointType = RootJointType::FLOATING);
};

DART_DEPRECATED(6.0)
simulation::WorldPtr readSdfFile(
    const common::Uri& uri,
    const common::ResourceRetrieverPtr& retriever = nullptr);

simulation::WorldPtr readWorld(
    const common::Uri& uri, const Options& options = Options());

DART_DEPRECATED(6.12)
simulation::WorldPtr readWorld(
    const common::Uri& uri, const common::ResourceRetrieverPtr& retriever);

dynamics::SkeletonPtr readSkeleton(
    const common::Uri& uri, const Options& options = Options());

DART_DEPRECATED(6.12)
dynamics::SkeletonPtr readSkeleton(
    const common::Uri& uri, const common::ResourceRetrieverPtr& retrievers);

} // namespace SdfParser

} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_SDFPARSER_HPP_
