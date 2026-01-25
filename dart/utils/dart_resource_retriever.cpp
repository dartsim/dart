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

#include "dart/utils/dart_resource_retriever.hpp"

#include "dart/common/diagnostics.hpp"
#include "dart/common/local_resource_retriever.hpp"
#include "dart/common/logging.hpp"
#include "dart/config.hpp"

#include <fstream>
#include <iostream>

#include <cstdlib>

namespace dart {
namespace utils {

//==============================================================================
DartResourceRetriever::DartResourceRetriever()
  : mLocalRetriever(std::make_shared<common::LocalResourceRetriever>())
{
  // 1. Search the local build directory
  addDataDirectory(dart::config::dataLocalPath());

  // 2. Search the designated install directory. This should work if you build
  // and install DART from source.
  addDataDirectory(dart::config::dataGlobalPath());

  // 3. Search the directory set by the environment variable, DART_DATA_PATH.
  // Method 2 can fail because some package manager use temporary install
  // directory (e.g., Launchpad PPA).
  const char* dartDataPathEnv = std::getenv("DART_DATA_PATH");
  if (dartDataPathEnv) {
    addDataDirectory(dartDataPathEnv);
  }
}

//==============================================================================
bool DartResourceRetriever::exists(const common::Uri& uri)
{
  std::string relativePath;
  if (!resolveDataUri(uri, relativePath)) {
    return false;
  }

  if (uri.mAuthority.get() == "sample") {
    for (const auto& dataPath : mDataDirectories) {
      common::Uri fileUri;
      fileUri.fromPath(dataPath + relativePath);

      if (mLocalRetriever->exists(fileUri)) {
        return true;
      }

      DART_WARN(
          "Failed to retrieve a resource from '{}'. Please make sure you set "
          "the environment variable for DART data path. For example:\\n  $ "
          "export DART_DATA_PATH=/usr/local/share/doc/dart/data/",
          uri.toString());
    }
  } else {
    if (mLocalRetriever->exists(uri)) {
      return true;
    }
  }

  return false;
}

//==============================================================================
common::ResourcePtr DartResourceRetriever::retrieve(const common::Uri& uri)
{
  std::string relativePath;
  if (!resolveDataUri(uri, relativePath)) {
    return nullptr;
  }

  if (uri.mAuthority.get() == "sample") {
    for (const auto& dataPath : mDataDirectories) {
      common::Uri fileUri;
      fileUri.fromPath(dataPath + relativePath);

      if (const auto resource = mLocalRetriever->retrieve(fileUri)) {
        return resource;
      }
    }

    DART_WARN(
        "Failed to retrieve a resource from '{}'. Please make sure you set the "
        "environment variable for DART data path. For example:\\n  $ export "
        "DART_DATA_PATH=/usr/local/share/doc/dart/data/",
        uri.toString());
  } else {
    if (const auto resource = mLocalRetriever->retrieve(uri)) {
      return resource;
    }
  }

  return nullptr;
}

//==============================================================================
DART_SUPPRESS_DEPRECATED_BEGIN
std::string DartResourceRetriever::getFilePath(const common::Uri& uri)
{
  std::string relativePath;
  if (!resolveDataUri(uri, relativePath)) {
    return "";
  }

  if (uri.mAuthority.get() == "sample") {
    for (const auto& dataPath : mDataDirectories) {
      common::Uri fileUri;
      fileUri.fromPath(dataPath + relativePath);

      const auto path = mLocalRetriever->getFilePath(fileUri);

      // path is empty if the file specified by fileUri doesn't exist.
      if (!path.empty()) {
        return path;
      }
    }

    DART_WARN(
        "Failed to retrieve a resource from '{}'. Please make sure you set the "
        "environment variable for DART data path. For example:\n  $ export "
        "DART_DATA_PATH=/usr/local/share/doc/dart/data/",
        uri.toString());
  } else {
    const auto path = mLocalRetriever->getFilePath(uri);

    // path is empty if the file specified by fileUri doesn't exist.
    if (!path.empty()) {
      return path;
    }
  }

  return "";
}
DART_SUPPRESS_DEPRECATED_END

//==============================================================================
void DartResourceRetriever::addDataDirectory(std::string_view dataDirectory)
{
  // Strip a trailing slash.
  std::string normalizedDataDirectory;
  if (!dataDirectory.empty() && dataDirectory.back() == '/') {
    normalizedDataDirectory
        = std::string(dataDirectory.substr(0, dataDirectory.size() - 1));
  } else {
    normalizedDataDirectory = std::string(dataDirectory);
  }

  mDataDirectories.push_back(normalizedDataDirectory);
}

//==============================================================================
bool DartResourceRetriever::resolveDataUri(
    const common::Uri& uri, std::string& relativePath) const
{
  if (uri.mScheme.get_value_or("dart") != "dart") {
    return false;
  }

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
