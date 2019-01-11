/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_COMMON_LOCALRESOURCERETRIEVER_HPP_
#define DART_COMMON_LOCALRESOURCERETRIEVER_HPP_

#include "dart/common/ResourceRetriever.hpp"

namespace dart {
namespace common {

/// LocalResourceRetriever provides access to local resources specified by
/// file:// URIs by wrapping the standard C and C++ file manipulation routines.
class LocalResourceRetriever : public virtual ResourceRetriever
{
public:
  virtual ~LocalResourceRetriever() = default;

  // Documentation inherited.
  bool exists(const Uri& _uri) override;

  // Documentation inherited.
  ResourcePtr retrieve(const Uri& _uri) override;

  // Documentation inherited.
  std::string getFilePath(const Uri& uri) override;
};

using LocalResourceRetrieverPtr = std::shared_ptr<LocalResourceRetriever>;

} // namespace common
} // namespace dart

#endif // ifndef DART_COMMON_LOCALRESOURCERETRIEVER_HPP_
