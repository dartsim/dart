/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
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

#include "dart/utils/DartResourceRetriever.hpp"

#include <iostream>
#include <fstream>
#include "dart/config.hpp"
#include "dart/common/Console.hpp"
#include "dart/common/LocalResourceRetriever.hpp"

namespace dart {
namespace utils {

//==============================================================================
DartResourceRetriever::DartResourceRetriever()
  : mLocalRetriever(std::make_shared<common::LocalResourceRetriever>())
{
  addDataDirectory(DART_DATA_LOCAL_PATH);
  addDataDirectory(DART_DATA_GLOBAL_PATH);
}

//==============================================================================
bool DartResourceRetriever::exists(const common::Uri& uri)
{
  std::string relativePath;
  if (!resolveDataUri(uri, relativePath))
    return false;

  if (uri.mAuthority.get() == "sample")
  {
    for (const auto& dataPath : mDataDirectories)
    {
      common::Uri fileUri;
      fileUri.fromPath(dataPath + relativePath);

      if (mLocalRetriever->exists(fileUri))
        return true;
    }
  }
  else
  {
    if (mLocalRetriever->exists(uri))
      return true;
  }

  return false;
}

//==============================================================================
common::ResourcePtr DartResourceRetriever::retrieve(const common::Uri& uri)
{
  std::string relativePath;
  if (!resolveDataUri(uri, relativePath))
    return nullptr;

  if (uri.mAuthority.get() == "sample")
  {
    for (const auto& dataPath : mDataDirectories)
    {
      common::Uri fileUri;
      fileUri.fromPath(dataPath + relativePath);

      if (const auto resource = mLocalRetriever->retrieve(fileUri))
        return resource;
    }
  }
  else
  {
    if (const auto resource = mLocalRetriever->retrieve(uri))
      return resource;
  }

  return nullptr;
}

//==============================================================================
void DartResourceRetriever::addDataDirectory(
    const std::string& dataDirectory)
{
  // Strip a trailing slash.
  std::string normalizedDataDirectory;
  if (!dataDirectory.empty() && dataDirectory.back() == '/')
  {
    normalizedDataDirectory
      = dataDirectory.substr(0, dataDirectory.size() - 1);
  }
  else
  {
    normalizedDataDirectory = dataDirectory;
  }

  mDataDirectories.push_back(normalizedDataDirectory);
}

//==============================================================================
bool DartResourceRetriever::resolveDataUri(
  const common::Uri& uri,
  std::string& relativePath) const
{
  if (uri.mScheme.get_value_or("dart") != "dart")
    return false;

  if (!uri.mPath)
  {
    dtwarn << "[DartResourceRetriever::resolveDataUri] Failed extracting"
              " relative path from URI '" << uri.toString() << "'.\n";
    return false;
  }
  relativePath = uri.mPath.get_value_or("");

  return true;
}

} // namespace utils
} // namespace dart
