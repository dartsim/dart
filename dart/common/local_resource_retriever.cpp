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

#include "dart/common/local_resource_retriever.hpp"

#include "dart/common/diagnostics.hpp"
#include "dart/common/local_resource.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/uri.hpp"

#include <fstream>
#include <iostream>

namespace dart {
namespace common {

//==============================================================================
bool LocalResourceRetriever::exists(const Uri& _uri)
{
  if (_uri.mScheme.get_value_or("file") != "file") {
    return false;
  }

  if (!_uri.mPath) {
    return false;
  }

  const auto path = _uri.getFilesystemPath();

  return std::ifstream(path, std::ios::binary).good();
}

//==============================================================================
common::ResourcePtr LocalResourceRetriever::retrieve(const Uri& _uri)
{
  if (_uri.mScheme.get_value_or("file") != "file") {
    return nullptr;
  } else if (!_uri.mPath) {
    return nullptr;
  }

  const auto resource
      = std::make_shared<LocalResource>(_uri.getFilesystemPath());

  if (resource->isGood()) {
    return resource;
  } else {
    return nullptr;
  }
}

//==============================================================================
DART_SUPPRESS_DEPRECATED_BEGIN
std::string LocalResourceRetriever::getFilePath(const Uri& uri)
{
  if (uri.mScheme.get_value_or("file") != "file") {
    return "";
  } else if (!uri.mPath) {
    return "";
  }

  const auto path = uri.getFilesystemPath();

  if (std::ifstream(path, std::ios::binary).good()) {
    return path;
  } else {
    return "";
  }
}
DART_SUPPRESS_DEPRECATED_END

} // namespace common
} // namespace dart
