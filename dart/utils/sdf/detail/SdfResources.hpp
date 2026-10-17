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

#ifndef DART_UTILS_SDF_DETAIL_SDFRESOURCES_HPP_
#define DART_UTILS_SDF_DETAIL_SDFRESOURCES_HPP_

#include "dart/common/ResourceRetriever.hpp"

#include <sdf/Element.hh>
#include <sdf/ParserConfig.hh>
#include <sdf/Root.hh>

#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace dart {
namespace utils {
namespace SdfParser {
namespace detail {

class SdfResources
{
public:
  SdfResources(
      common::ResourceRetrieverPtr retriever, bool useNativeFileLookup);
  ~SdfResources();

  SdfResources(const SdfResources&) = delete;
  SdfResources& operator=(const SdfResources&) = delete;

  bool load(const common::Uri& uri, sdf::Root& root);
  common::Uri resolveUri(
      const sdf::ElementPtr& element, const std::string& uri) const;

private:
  struct Include
  {
    common::Uri owner;
    std::string uri;
    std::vector<std::string> ancestors;
  };

  std::string stage(
      const common::Uri& uri, const std::vector<std::string>& ancestors);
  std::string findInclude(const std::string& alias);
  void prepare(
      const sdf::ElementPtr& element,
      const common::Uri& source,
      const std::vector<std::string>& ancestors,
      bool softGeometry = false);

  common::ResourceRetrieverPtr mRetriever;
  bool mUseNativeFileLookup;
  sdf::ParserConfig mConfig;
  std::filesystem::path mDirectory;
  common::Uri mRootUri;
  std::map<std::string, Include> mIncludes;
  std::map<std::string, common::Uri> mOrigins;
  std::size_t mNextId = 0;
  bool mFailed = false;
};

} // namespace detail
} // namespace SdfParser
} // namespace utils
} // namespace dart

#endif // DART_UTILS_SDF_DETAIL_SDFRESOURCES_HPP_
