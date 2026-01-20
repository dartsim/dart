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

#ifndef DART_UTILS_HTTPRESOURCERETRIEVER_HPP_
#define DART_UTILS_HTTPRESOURCERETRIEVER_HPP_

#include <dart/utils/export.hpp>

#include <dart/common/filesystem.hpp>
#include <dart/common/local_resource_retriever.hpp>
#include <dart/common/resource_retriever.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#ifndef DART_HAS_CURL
  #define DART_HAS_CURL 0
#endif

namespace dart {
namespace utils {

/// Retrieve resources located at http(s) URIs by downloading them to a local
/// on-disk cache and delegating to a LocalResourceRetriever.
///
/// The retriever always uses disk caching (to avoid keeping large files in
/// memory and to allow subsequent calls to reuse already downloaded assets).
/// Cached files are keyed by URL and stored inside the configured cache
/// directory (defaults to $TMP/dart_http_cache).
class DART_UTILS_API HttpResourceRetriever
  : public virtual common::ResourceRetriever
{
public:
  struct Options
  {
    /// Whether to follow redirects returned by the remote server.
    bool followRedirects = true;

    /// Timeout for establishing the TCP connection.
    std::chrono::milliseconds connectTimeout{5000};

    /// Timeout for an entire transfer (0 disables the limit).
    std::chrono::milliseconds transferTimeout{0};

    /// Maximum number of bytes to download per request (0 disables the limit).
    std::size_t maxDownloadBytes = 0;

    /// User agent string presented to servers.
    std::string userAgent = "dart-http-resource-retriever";
  };

  HttpResourceRetriever();
  explicit HttpResourceRetriever(const Options& options);
  ~HttpResourceRetriever() override = default;

  // Documentation inherited.
  bool exists(const common::Uri& uri) override;

  // Documentation inherited.
  common::ResourcePtr retrieve(const common::Uri& uri) override;

  // Documentation inherited.
  DART_DEPRECATED(7.0)
  std::string getFilePath(const common::Uri& uri) override;

  /// Set the runtime options.
  void setOptions(const Options& options);

  /// Retrieve the runtime options.
  const Options& getOptions() const;

  /// Override the cache directory. The directory will be created on demand.
  void setCacheDirectory(const common::filesystem::path& directory);

  /// Return the currently configured cache directory.
  const common::filesystem::path& getCacheDirectory() const;

private:
  bool isSupported(const common::Uri& uri) const;
  common::filesystem::path buildCachePath(const common::Uri& uri) const;
  bool ensureCacheDirectory() const;
  bool download(const common::Uri& uri, const std::string& destination) const;
  bool performExistsRequest(const common::Uri& uri) const;

  Options mOptions;
  common::filesystem::path mCacheDirectory;
  std::shared_ptr<common::LocalResourceRetriever> mLocalRetriever;
};

using HttpResourceRetrieverPtr = std::shared_ptr<HttpResourceRetriever>;

} // namespace utils
} // namespace dart

#endif // ifndef DART_UTILS_HTTPRESOURCERETRIEVER_HPP_
