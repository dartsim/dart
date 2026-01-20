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

#ifndef DART_IO_READ_HPP_
#define DART_IO_READ_HPP_

#include <dart/simulation/world.hpp>

#include <dart/dynamics/skeleton.hpp>

#include <dart/common/resource_retriever.hpp>
#include <dart/common/result.hpp>
#include <dart/common/uri.hpp>

#include <dart/io/export.hpp>

#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace dart {
namespace io {

/// Supported model formats for reading Skeletons and Worlds.
enum class ModelFormat
{
  /// Infer from URI (extension / XML root element).
  Auto = 0,

  /// DART native XML format (.skel).
  Skel,

  /// SDF format (.sdf / .world).
  Sdf,

  /// URDF format (.urdf / <robot>).
  Urdf,

  /// MJCF format (.xml / <mujoco>).
  Mjcf,

};

/// Root joint type used when a model does not explicitly specify its root
/// joint.
enum class RootJointType
{
  /// Floating joint type.
  Floating = 0,

  /// Fixed joint type.
  Fixed,
};

/// Options for reading a model file.
struct DART_IO_API ReadOptions
{
  /// Model format. If Auto, infer from URI.
  ModelFormat format{ModelFormat::Auto};

  /// Resource retriever used for reading referenced resources. If nullptr, a
  /// default CompositeResourceRetriever is used (file:// and dart://).
  common::ResourceRetrieverPtr resourceRetriever{nullptr};

  /// Default root joint type for SDF models when it is not explicitly
  /// specified.
  RootJointType sdfDefaultRootJointType{RootJointType::Floating};

  /// Package directories for resolving package:// URIs in URDF files.
  ///
  /// This is equivalent to calling
  /// dart::utils::UrdfParser::addPackageDirectory. You can provide multiple
  /// directories per package name; they will be tried in the same order in
  /// which they are added.
  std::map<std::string, std::vector<std::string>> urdfPackageDirectories;

  /// Add a package directory for resolving package:// URIs in URDF files.
  void addPackageDirectory(
      std::string_view packageName, std::string_view packageDirectory)
  {
    urdfPackageDirectories[std::string(packageName)].push_back(
        std::string(packageDirectory));
  }
};

/// Read World from a model file.
simulation::WorldPtr DART_IO_API
readWorld(const common::Uri& uri, const ReadOptions& options = ReadOptions());

/// Read Skeleton from a model file.
dynamics::SkeletonPtr DART_IO_API readSkeleton(
    const common::Uri& uri, const ReadOptions& options = ReadOptions());

/// Try to read World from a model file, returning Result instead of nullptr.
common::Result<simulation::WorldPtr, common::Error> DART_IO_API tryReadWorld(
    const common::Uri& uri, const ReadOptions& options = ReadOptions());

/// Try to read Skeleton from a model file, returning Result instead of nullptr.
common::Result<dynamics::SkeletonPtr, common::Error> DART_IO_API
tryReadSkeleton(
    const common::Uri& uri, const ReadOptions& options = ReadOptions());

} // namespace io
} // namespace dart

#endif // DART_IO_READ_HPP_
