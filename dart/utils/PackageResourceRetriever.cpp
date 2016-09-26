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

#include <cassert>
#include <sstream>
#include <iostream>
#include "dart/common/Console.hpp"
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/utils/PackageResourceRetriever.hpp"

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
  const std::string& _packageName, const std::string& _packageDirectory)
{
  // Strip a trailing slash.
  std::string normalizedPackageDirectory;
  if(!_packageDirectory.empty() && _packageDirectory.back() == '/')
    normalizedPackageDirectory
      = _packageDirectory.substr(0, _packageDirectory.size() - 1);
  else
    normalizedPackageDirectory = _packageDirectory;

  mPackageMap[_packageName].push_back(normalizedPackageDirectory);
}

//==============================================================================
bool PackageResourceRetriever::exists(const common::Uri& _uri)
{
  std::string packageName, relativePath;
  if (!resolvePackageUri(_uri, packageName, relativePath))
    return false;

  for(const std::string& packagePath : getPackagePaths(packageName))
  {
    common::Uri fileUri;
    fileUri.fromPath(packagePath + relativePath);

    if (mLocalRetriever->exists(fileUri))
      return true;
  }
  return false;
}

//==============================================================================
common::ResourcePtr PackageResourceRetriever::retrieve(const common::Uri& _uri)
{
  std::string packageName, relativePath;
  if (!resolvePackageUri(_uri, packageName, relativePath))
    return nullptr;

  for(const std::string& packagePath : getPackagePaths(packageName))
  {
    common::Uri fileUri;
    fileUri.fromPath(packagePath + relativePath);

    if(const auto resource = mLocalRetriever->retrieve(fileUri))
      return resource;
  }
  return nullptr;
}

//==============================================================================
const std::vector<std::string>& PackageResourceRetriever::getPackagePaths(
  const std::string& _packageName) const
{
  static const std::vector<std::string> empty_placeholder;

  // Lookup the corresponding package path.
  const auto it = mPackageMap.find(_packageName);
  if(it != std::end(mPackageMap))
    return it->second;
  else
  {
    dtwarn << "[PackageResourceResolver::getPackagePaths] Unable to resolve"
              "path to package '" << _packageName << "'. Did you call"
              " addPackageDirectory(~) for this package name?\n";
    return empty_placeholder;
  }
}

//==============================================================================
bool PackageResourceRetriever::resolvePackageUri(
  const common::Uri& _uri, std::string& _packageName,
  std::string& _relativePath) const
{
  if(_uri.mScheme.get_value_or("file") != "package")
    return false;

  if(!_uri.mAuthority)
  {
    dtwarn << "[PackageResourceRetriever::resolvePackageUri] Failed extracting"
              " package name from URI '" << _uri.toString() << "'.\n";
    return false;
  }
  _packageName = *_uri.mAuthority;

  if(!_uri.mPath)
  {
    dtwarn << "[PackageResourceRetriever::resolvePackageUri] Failed extracting"
              " relative path from URI '" << _uri.toString() << "'.\n";
    return false;
  }
  _relativePath = _uri.mPath.get_value_or("");

  return true;
}

} // namespace utils
} // namespace dart
