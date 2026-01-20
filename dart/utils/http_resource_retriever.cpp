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

#include "dart/utils/http_resource_retriever.hpp"

#include <dart/common/diagnostics.hpp>
#include <dart/common/logging.hpp>
#include <dart/common/macros.hpp>

#include <iomanip>
#include <memory>
#include <sstream>
#include <string_view>
#include <utility>

#include <cstdint>
#include <cstdio>
#include <cstring>

#if DART_HAS_CURL
  #include <curl/curl.h>
#endif

namespace dart {
namespace utils {

namespace {

std::string sanitizeFileName(std::string_view name)
{
  if (name.empty())
    return "resource";

  std::string sanitized;
  sanitized.reserve(name.size());
  for (char c : name) {
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
        || (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '_')
      sanitized.push_back(c);
    else
      sanitized.push_back('_');
  }

  if (sanitized.empty())
    return "resource";

  return sanitized;
}

std::string computeHash(std::string_view input)
{
  std::uint64_t hash = 1469598103934665603ULL; // FNV-1a offset basis
  for (unsigned char c : input) {
    hash ^= c;
    hash *= 1099511628211ULL; // FNV-1a prime
  }

  std::ostringstream oss;
  oss << std::hex << std::setw(16) << std::setfill('0') << hash;
  return oss.str();
}

#if DART_HAS_CURL
struct DownloadContext
{
  FILE* file = nullptr;
  std::size_t maxBytes = 0;
  std::size_t received = 0;
};

std::size_t writeCallback(
    void* contents, std::size_t size, std::size_t nmemb, void* userp)
{
  auto* ctx = static_cast<DownloadContext*>(userp);
  const std::size_t bytes = size * nmemb;

  if (ctx->maxBytes > 0 && ctx->received + bytes > ctx->maxBytes)
    return 0;

  const std::size_t written = std::fwrite(contents, size, nmemb, ctx->file);
  ctx->received += written * size;
  return written;
}

#endif // if DART_HAS_CURL

} // namespace

//==============================================================================
HttpResourceRetriever::HttpResourceRetriever()
  : HttpResourceRetriever(Options{})
{
}

//==============================================================================
HttpResourceRetriever::HttpResourceRetriever(const Options& options)
  : mOptions(options),
    mLocalRetriever(std::make_shared<common::LocalResourceRetriever>())
{
  try {
    mCacheDirectory
        = common::filesystem::temp_directory_path() / "dart_http_cache";
  } catch (const std::exception& e) {
    DART_WARN(
        "Failed to determine a temporary directory for the HTTP cache: {}",
        e.what());
    mCacheDirectory = "dart_http_cache";
  }

  ensureCacheDirectory();
}

//==============================================================================
bool HttpResourceRetriever::exists(const common::Uri& uri)
{
  if (!isSupported(uri))
    return false;

  const auto cachePath = buildCachePath(uri);
  common::error_code ec;
  if (common::filesystem::exists(cachePath, ec) && !ec)
    return true;

  return performExistsRequest(uri);
}

//==============================================================================
common::ResourcePtr HttpResourceRetriever::retrieve(const common::Uri& uri)
{
  if (!isSupported(uri))
    return nullptr;

  if (!ensureCacheDirectory())
    return nullptr;

  const auto cachePath = buildCachePath(uri);

  common::error_code ec;
  if (!common::filesystem::exists(cachePath, ec) || ec) {
    if (!download(uri, cachePath.string()))
      return nullptr;
  }

  common::Uri fileUri;
  if (!fileUri.fromPath(cachePath.string())) {
    DART_WARN(
        "Failed creating a file URI for cached resource '{}'.",
        cachePath.string());
    return nullptr;
  }

  return mLocalRetriever->retrieve(fileUri);
}

//==============================================================================
DART_SUPPRESS_DEPRECATED_BEGIN
std::string HttpResourceRetriever::getFilePath(const common::Uri& uri)
{
  if (!isSupported(uri))
    return "";

  const auto cachePath = buildCachePath(uri);
  common::error_code ec;
  if (common::filesystem::exists(cachePath, ec) && !ec)
    return cachePath.string();

  if (download(uri, cachePath.string()))
    return cachePath.string();
  else
    return "";
}
DART_SUPPRESS_DEPRECATED_END

//==============================================================================
void HttpResourceRetriever::setOptions(const Options& options)
{
  mOptions = options;
}

//==============================================================================
const HttpResourceRetriever::Options& HttpResourceRetriever::getOptions() const
{
  return mOptions;
}

//==============================================================================
void HttpResourceRetriever::setCacheDirectory(
    const common::filesystem::path& directory)
{
  mCacheDirectory = directory;
  ensureCacheDirectory();
}

//==============================================================================
const common::filesystem::path& HttpResourceRetriever::getCacheDirectory() const
{
  return mCacheDirectory;
}

//==============================================================================
bool HttpResourceRetriever::isSupported(const common::Uri& uri) const
{
  const std::string scheme = uri.mScheme.get_value_or("");
  return scheme == "http" || scheme == "https";
}

//==============================================================================
common::filesystem::path HttpResourceRetriever::buildCachePath(
    const common::Uri& uri) const
{
  std::string url = uri.toString();

  // Drop fragments when building the cache key.
  if (uri.mFragment) {
    const auto fragment = "#" + uri.mFragment.get_value_or("");
    if (!fragment.empty() && url.size() >= fragment.size()) {
      if (url.compare(url.size() - fragment.size(), fragment.size(), fragment)
          == 0)
        url.erase(url.size() - fragment.size());
    }
  }

  const std::string hash = computeHash(url);

  std::string leaf = uri.mPath.get_value_or("");
  if (!leaf.empty()) {
    const auto pos = leaf.find_last_of('/');
    if (pos != std::string::npos)
      leaf = leaf.substr(pos + 1);
  }

  // Remove query parameters from the filename component.
  const auto queryPos = leaf.find_first_of("?&");
  if (queryPos != std::string::npos)
    leaf = leaf.substr(0, queryPos);

  if (leaf.empty())
    leaf = "resource";

  const auto sanitizedLeaf = sanitizeFileName(leaf);

  common::filesystem::path path = mCacheDirectory;
  path /= hash + "_" + sanitizedLeaf;
  return path;
}

//==============================================================================
bool HttpResourceRetriever::ensureCacheDirectory() const
{
  common::error_code ec;
  if (common::filesystem::exists(mCacheDirectory, ec) && !ec)
    return true;

  common::filesystem::create_directories(mCacheDirectory, ec);
  if (ec) {
    DART_WARN(
        "Failed creating HTTP cache directory '{}': {}",
        mCacheDirectory.string(),
        ec.message());
    return false;
  }

  return true;
}

//==============================================================================
bool HttpResourceRetriever::download(
    const common::Uri& uri, const std::string& destination) const
{
#if DART_HAS_CURL
  auto* curl = curl_easy_init();
  if (!curl) {
    DART_WARN("Failed to initialize libcurl.");
    return false;
  }

  std::unique_ptr<FILE, int (*)(FILE*)> file(
      std::fopen(destination.c_str(), "wb"), std::fclose);
  if (!file) {
    DART_WARN(
        "Failed opening '{}' for writing downloaded content.", destination);
    curl_easy_cleanup(curl);
    return false;
  }

  DownloadContext context;
  context.file = file.get();
  context.maxBytes = mOptions.maxDownloadBytes;

  const std::string url = uri.toString();
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(
      curl, CURLOPT_FOLLOWLOCATION, mOptions.followRedirects ? 1L : 0L);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &writeCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &context);
  curl_easy_setopt(curl, CURLOPT_USERAGENT, mOptions.userAgent.c_str());
  curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
  #if LIBCURL_VERSION_NUM >= 0x075500
  curl_easy_setopt(curl, CURLOPT_PROTOCOLS_STR, "http,https");
  curl_easy_setopt(curl, CURLOPT_REDIR_PROTOCOLS_STR, "http,https");
  #else
  curl_easy_setopt(curl, CURLOPT_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
  curl_easy_setopt(
      curl, CURLOPT_REDIR_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
  #endif

  if (mOptions.connectTimeout.count() > 0) {
    curl_easy_setopt(
        curl, CURLOPT_CONNECTTIMEOUT_MS, mOptions.connectTimeout.count());
  }

  if (mOptions.transferTimeout.count() > 0) {
    curl_easy_setopt(
        curl, CURLOPT_TIMEOUT_MS, mOptions.transferTimeout.count());
  }

  const CURLcode code = curl_easy_perform(curl);
  curl_easy_cleanup(curl);

  if (code != CURLE_OK) {
    DART_WARN("Failed downloading '{}': {}", url, curl_easy_strerror(code));
    common::error_code ec;
    common::filesystem::remove(destination, ec);
    return false;
  }

  return true;
#else
  DART_UNUSED(uri);
  DART_UNUSED(destination);
  DART_ERROR(
      "HttpResourceRetriever is unavailable because DART was built without "
      "libcurl.");
  return false;
#endif
}

//==============================================================================
bool HttpResourceRetriever::performExistsRequest(const common::Uri& uri) const
{
#if DART_HAS_CURL
  auto* curl = curl_easy_init();
  if (!curl) {
    DART_WARN("Failed to initialize libcurl.");
    return false;
  }

  const std::string url = uri.toString();
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_NOBODY, 1L);
  curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
  curl_easy_setopt(
      curl, CURLOPT_FOLLOWLOCATION, mOptions.followRedirects ? 1L : 0L);
  curl_easy_setopt(curl, CURLOPT_USERAGENT, mOptions.userAgent.c_str());
  #if LIBCURL_VERSION_NUM >= 0x075500
  curl_easy_setopt(curl, CURLOPT_PROTOCOLS_STR, "http,https");
  curl_easy_setopt(curl, CURLOPT_REDIR_PROTOCOLS_STR, "http,https");
  #else
  curl_easy_setopt(curl, CURLOPT_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
  curl_easy_setopt(
      curl, CURLOPT_REDIR_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
  #endif

  if (mOptions.connectTimeout.count() > 0) {
    curl_easy_setopt(
        curl, CURLOPT_CONNECTTIMEOUT_MS, mOptions.connectTimeout.count());
  }
  if (mOptions.transferTimeout.count() > 0) {
    curl_easy_setopt(
        curl, CURLOPT_TIMEOUT_MS, mOptions.transferTimeout.count());
  }

  const CURLcode code = curl_easy_perform(curl);
  curl_easy_cleanup(curl);

  return code == CURLE_OK;
#else
  DART_UNUSED(uri);
  return false;
#endif
}

} // namespace utils
} // namespace dart
