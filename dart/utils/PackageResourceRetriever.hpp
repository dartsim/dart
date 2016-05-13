/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_UTILS_PACKAGERESOURCERETRIEVER_HPP_
#define DART_UTILS_PACKAGERESOURCERETRIEVER_HPP_

#include <unordered_map>
#include <vector>
#include "dart/common/ResourceRetriever.hpp"

namespace dart {
namespace utils {

/// Retrieve local resources specified by package:// URIs by: (1) resolving 
/// the package path and (2) passing the resolved URI to another
/// \ref ResourceRetriever. This class uses requires you to manually provide the
/// base URI of every package that you wish to resolve using the
/// \ref addPackageDirectory method.
class PackageResourceRetriever : public virtual common::ResourceRetriever
{
public:
  /// Construct a PackageResourceRetriever that uses the specified \a
  /// _localRetriever to load resolved URIs.
  explicit PackageResourceRetriever(
    const common::ResourceRetrieverPtr& _localRetriever = nullptr);

  virtual ~PackageResourceRetriever() = default;

  /// Specify the directory of a ROS package. In your URDF files, you may see
  /// strings with a package URI pattern such as:
  ///
  /// @code
  /// "package://my_robot/meshes/mesh_for_my_robot.stl"
  ///  \______/  \______/\___________________________/
  ///      |        |                 |
  ///   package  package   file path with respect to
  ///   keyword   name       the package directory
  /// @endcode
  ///
  /// For us to successfully parse a URDF, we need to be told what the path
  /// to the package directory is, using addPackageDirectory(). In this case,
  /// suppose the path to the my_robot package is /path/to/my_robot. Then you
  /// should use addPackageDirectory("my_robot", "/path/to/my_robot").
  /// Altogether, this implies that a file named
  /// "/path/to/my_robot/meshes/mesh_for_my_robot.stl" exists. Whatever you
  /// specify as the package directory will end up replacing the 'package
  /// keyword' and 'package name' components of the URI string.
  ///
  /// You can call this method multiple times with the same \a _packageName to
  /// provide multiple candidates for resolution. This is necessarry if your
  /// resources are split between the Catkin devel and source spaces. Multiple
  /// candidates will be tested in the same order in which they were added.
  ///
  /// This class supports arbitrary URIs for \a _packageDirectory, as long as
  /// they are supported by the \a _localRetriever passed to the constructor.
  void addPackageDirectory(const std::string& _packageName,
                           const std::string& _packageDirectory);

  // Documentation inherited.
  bool exists(const common::Uri& _uri) override;

  // Documentation inherited.
  common::ResourcePtr retrieve(const common::Uri& _uri) override;

private:
  common::ResourceRetrieverPtr mLocalRetriever;
  std::unordered_map<std::string, std::vector<std::string> > mPackageMap;

  const std::vector<std::string>& getPackagePaths(
    const std::string& _packageName) const;
  bool resolvePackageUri(const common::Uri& _uri,
    std::string& _packageName, std::string& _relativePath) const;
};

using PackageResourceRetrieverPtr = std::shared_ptr<PackageResourceRetriever>;

} // namespace utils
} // namespace dart

#endif // ifndef DART_UTILS_PACKAGERESOURCERETRIEVER_HPP_
