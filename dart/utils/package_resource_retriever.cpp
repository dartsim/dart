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

#include "dart/utils/package_resource_retriever.hpp"

#include "dart/common/diagnostics.hpp"
#include "dart/common/local_resource_retriever.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/uri.hpp"

#include <iostream>
#include <sstream>

#include <cassert>

namespace dart {
namespace utils {

//==============================================================================
PackageResourceRetriever::PackageResourceRetriever(
    const common::ResourceRetrieverPtr& localRetriever)
{
  if (localRetriever)
    mLocalRetriever = localRetriever;
  else
    mLocalRetriever = std::make_shared<common::LocalResourceRetriever>();
}

//==============================================================================
void PackageResourceRetriever::addPackageDirectory(
    std::string_view packageName, std::string_view packageDirectory)
{
  // Strip a trailing slash.
  std::string_view normalizedPackageDirectory = packageDirectory;
  if (!normalizedPackageDirectory.empty()
      && normalizedPackageDirectory.back() == '/') {
    normalizedPackageDirectory = normalizedPackageDirectory.substr(
        0, normalizedPackageDirectory.size() - 1);
  }

  mPackageMap[std::string(packageName)].push_back(
      std::string(normalizedPackageDirectory));
}

//==============================================================================
bool PackageResourceRetriever::exists(const common::Uri& uri)
{
  std::string packageName, relativePath;
  if (!resolvePackageUri(uri, packageName, relativePath))
    return false;

  for (const std::string& packagePath : getPackagePaths(packageName)) {
    const std::string resolvedUri = packagePath + relativePath;
    common::Uri fileUri;
    if (!fileUri.fromStringOrPath(resolvedUri)) {
      DART_WARN(
          "Failed to parse resolved URI '{}' for package '{}'.",
          resolvedUri,
          packageName);
      continue;
    }

    if (mLocalRetriever->exists(fileUri))
      return true;
  }

  return false;
}

//==============================================================================
common::ResourcePtr PackageResourceRetriever::retrieve(const common::Uri& uri)
{
  std::string packageName, relativePath;
  if (!resolvePackageUri(uri, packageName, relativePath))
    return nullptr;

  for (const std::string& packagePath : getPackagePaths(packageName)) {
    const std::string resolvedUri = packagePath + relativePath;
    common::Uri fileUri;
    if (!fileUri.fromStringOrPath(resolvedUri)) {
      DART_WARN(
          "Failed to parse resolved URI '{}' for package '{}'.",
          resolvedUri,
          packageName);
      continue;
    }

    if (const auto resource = mLocalRetriever->retrieve(fileUri))
      return resource;
  }
  return nullptr;
}

//==============================================================================
DART_SUPPRESS_DEPRECATED_BEGIN
std::string PackageResourceRetriever::getFilePath(const common::Uri& uri)
{
  std::string packageName, relativePath;
  if (!resolvePackageUri(uri, packageName, relativePath))
    return "";

  for (const std::string& packagePath : getPackagePaths(packageName)) {
    const std::string resolvedUri = packagePath + relativePath;
    common::Uri fileUri;
    if (!fileUri.fromStringOrPath(resolvedUri)) {
      DART_WARN(
          "Failed to parse resolved URI '{}' for package '{}'.",
          resolvedUri,
          packageName);
      continue;
    }

    const auto path = mLocalRetriever->getFilePath(fileUri);

    // path is empty if the file specified by fileUri doesn't exist.
    if (!path.empty())
      return path;
  }

  return "";
}
DART_SUPPRESS_DEPRECATED_END

//==============================================================================
std::span<const std::string> PackageResourceRetriever::getPackagePaths(
    std::string_view packageName) const
{
  static const std::vector<std::string> empty_placeholder;
  const std::string packageNameString(packageName);

  // Lookup the corresponding package path.
  const auto it = mPackageMap.find(packageNameString);
  if (it != std::end(mPackageMap))
    return std::span<const std::string>(it->second);
  else {
    DART_WARN(
        "{}{}{}",
        "Unable to resolvepath to package '",
        packageNameString,
        "'. Did you call addPackageDirectory(~) for this package name?\n");
    return std::span<const std::string>(empty_placeholder);
  }
}

//==============================================================================
bool PackageResourceRetriever::resolvePackageUri(
    const common::Uri& uri,
    std::string& packageName,
    std::string& relativePath) const
{
  if (uri.mScheme.get_value_or("file") != "package")
    return false;

  if (!uri.mAuthority) {
    DART_WARN(
        "{}{}'.", "Failed extracting package name from URI '", uri.toString());
    return false;
  }
  packageName = *uri.mAuthority;

  if (!uri.mPath) {
    DART_WARN(
        "{}{}'.", "Failed extracting relative path from URI '", uri.toString());
    return false;
  }
  relativePath = uri.mPath.get_value_or("");

  return true;
}

} // namespace utils
} // namespace dart
