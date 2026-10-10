/*
 * Copyright (c) 2026, The DART development contributors
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

#ifndef DART_UTILS_ASSETS_MODELRESOURCERETRIEVER_HPP_
#define DART_UTILS_ASSETS_MODELRESOURCERETRIEVER_HPP_

#include <dart/common/ResourceRetriever.hpp>

#include <memory>
#include <string>

namespace dart {
namespace utils {

/// Retrieve model://id/revision/path resources from verified HTTPS bundles.
///
/// A local XML manifest lists every file, SHA-256 digest, and byte count. First
/// access materializes the complete bundle in a persistent cache, preserving
/// relative mesh and texture paths. A cached bundle is verified on its first
/// use, and each subsequently requested file is verified again. Files read
/// directly by a renderer are outside those subsequent verification checks.
class ModelResourceRetriever : public common::ResourceRetriever
{
public:
  /// Use the platform user cache when cacheDirectory is empty. Offline mode
  /// verifies existing bundles and never performs network requests.
  explicit ModelResourceRetriever(
      const std::string& cacheDirectory = "", bool offline = false);

  ~ModelResourceRetriever() override;

  /// Register a trusted file:// or dart://sample/ XML manifest without fetching
  /// model files. Identical registrations are idempotent; conflicting manifests
  /// for the same ID and revision are rejected. Returns false on invalid input.
  bool addManifest(const common::Uri& localManifestUri);

  /// Like getFilePath(), this may download the bundle in online mode.
  bool exists(const common::Uri& uri) override;

  common::ResourcePtr retrieve(const common::Uri& uri) override;

  /// Return a verified local path, or an empty string on failure. A corrupt
  /// published cache is rejected and must be explicitly removed to refetch.
  std::string getFilePath(const common::Uri& uri) override;

private:
  struct Impl;
  std::unique_ptr<Impl> mImpl;
};

using ModelResourceRetrieverPtr = std::shared_ptr<ModelResourceRetriever>;

} // namespace utils
} // namespace dart

#endif // DART_UTILS_ASSETS_MODELRESOURCERETRIEVER_HPP_
