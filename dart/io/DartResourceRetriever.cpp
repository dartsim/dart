/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include "dart/io/DartResourceRetriever.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/MemoryResource.hpp"
#include "dart/config.hpp"

#if DART_HAS_CURL
  #include <curl/curl.h>
#endif

#include <fstream>
#include <iostream>

#include <cstdlib>

namespace dart {
namespace io {

namespace {

#if DART_HAS_CURL

size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp)
{
  reinterpret_cast<std::string*>(userp)->append(
      static_cast<char*>(contents), size * nmemb);
  return size * nmemb;
}

bool httpGet(const std::string& url, std::string& response)
{
  CURL* curl = curl_easy_init();
  if (!curl)
    return false;

  CURLcode res;
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

  res = curl_easy_perform(curl);

  curl_easy_cleanup(curl);

  return res == CURLE_OK;
}

#endif

} // namespace

//==============================================================================
DartResourceRetriever::DartResourceRetriever()
  : mLocalRetriever(std::make_shared<common::LocalResourceRetriever>())
{
  // 1. Search the local build directory
  addDataDirectory(DART_DATA_LOCAL_PATH);

  // 2. Search the designated install directory. This should work if you build
  // and install DART from source.
  addDataDirectory(DART_DATA_GLOBAL_PATH);

  // 3. Search the directory set by the environment variable,
  // DART_DATA_LOCAL_PATH. Method 2 can fail because some package manager use
  // temporary install directory (e.g., Launchpad PPA).
  const char* dartDataPathEnv = std::getenv("DART_DATA_LOCAL_PATH");
  if (dartDataPathEnv)
    addDataDirectory(dartDataPathEnv);
}

//==============================================================================
bool DartResourceRetriever::exists(const common::Uri& uri)
{
  std::string relativePath;
  if (!resolveDataUri(uri, relativePath))
    return false;

  if (uri.mAuthority.get() == "sample") {
    for (const auto& dataPath : mDataDirectories) {
      common::Uri fileUri;
      fileUri.fromPath(dataPath + relativePath);
      if (mLocalRetriever->exists(fileUri))
        return true;
    }

#if DART_HAS_CURL
    // Check if the URI is a GitHub URL
    std::string response;
    const auto fullUrl
        = "https://raw.githubusercontent.com/dartsim/dart/main/data/"
          + relativePath;
    if (httpGet(fullUrl, response)) {
      return true;
    }
#endif
  } else {
    if (mLocalRetriever->exists(uri))
      return true;
  }

  dtwarn << "Failed to retrieve a resource from '" << uri.toString()
         << "'. Please make sure you set the environment variable for DART "
         << "data path. For example:\n"
         << "  $ export "
         << "DART_DATA_LOCAL_PATH=/usr/local/share/doc/dart/data/\n";

  return false;
}

//==============================================================================
common::ResourcePtr DartResourceRetriever::retrieve(const common::Uri& uri)
{
  std::string relativePath;
  if (!resolveDataUri(uri, relativePath))
    return nullptr;

  if (uri.mAuthority.get() == "sample") {
    for (const auto& dataPath : mDataDirectories) {
      common::Uri fileUri;
      fileUri.fromPath(dataPath + relativePath);

      if (const auto resource = mLocalRetriever->retrieve(fileUri)) {
        return resource;
      }
    }

#if DART_HAS_CURL
    common::Uri githubUri;
    githubUri.fromRelativeUri(
        common::Uri("https://raw.githubusercontent.com/dartsim/dart/main/data"),
        relativePath);
    std::string response;
    if (httpGet(uri.toString(), response)) {
      return std::make_shared<common::MemoryResource>(response);
    }
#endif
  } else {
    if (const auto resource = mLocalRetriever->retrieve(uri))
      return resource;
  }

  dtwarn << "Failed to retrieve a resource from '" << uri.toString()
         << "'. Please make sure you set the environment variable for DART "
         << "data path. For example:\n"
         << "  $ export DART_DATA_LOCAL_PATH=/usr/local/share/doc/dart/data/\n";

  return nullptr;
}

//==============================================================================
std::string DartResourceRetriever::getFilePath(const common::Uri& uri)
{
  std::string relativePath;
  if (!resolveDataUri(uri, relativePath))
    return "";

  if (uri.mAuthority.get() == "sample") {
    for (const auto& dataPath : mDataDirectories) {
      common::Uri fileUri;
      fileUri.fromPath(dataPath + relativePath);

      const auto path = mLocalRetriever->getFilePath(fileUri);

      // path is empty if the file specified by fileUri doesn't exist.
      if (!path.empty())
        return path;
    }
  } else {
    const auto path = mLocalRetriever->getFilePath(uri);

    // path is empty if the file specified by fileUri doesn't exist.
    if (!path.empty())
      return path;
  }

  dtwarn << "Failed to retrieve a resource from '" << uri.toString()
         << "'. Please make sure you set the environment variable for "
         << "DART data path. For example:\n"
         << "  $ export DART_DATA_LOCAL_PATH=/usr/local/share/doc/dart/data/\n";

  return "";
}

//==============================================================================
void DartResourceRetriever::addDataDirectory(const std::string& dataDirectory)
{
  // Strip a trailing slash.
  std::string normalizedDataDirectory;
  if (!dataDirectory.empty() && dataDirectory.back() == '/') {
    normalizedDataDirectory = dataDirectory.substr(0, dataDirectory.size() - 1);
  } else {
    normalizedDataDirectory = dataDirectory;
  }

  mDataDirectories.push_back(normalizedDataDirectory);
}

//==============================================================================
bool DartResourceRetriever::resolveDataUri(
    const common::Uri& uri, std::string& relativePath) const
{
  if (uri.mScheme.get_value_or("dart") != "dart")
    return false;

  if (!uri.mPath) {
    dtwarn << "[DartResourceRetriever::resolveDataUri] Failed extracting"
              " relative path from URI '"
           << uri.toString() << "'.\n";
    return false;
  }
  relativePath = uri.mPath.get_value_or("");

  return true;
}

} // namespace io
} // namespace dart
