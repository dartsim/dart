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

#ifndef DART_COMMON_RESOURCERETRIEVER_HPP_
#define DART_COMMON_RESOURCERETRIEVER_HPP_

#include <memory>
#include <string>
#include "dart/common/Resource.hpp"
#include "dart/common/Uri.hpp"

namespace dart {
namespace common {

/// ResourceRetriever provides methods for testing for the existance of and
/// accessing the content of a resource specified by URI.
class ResourceRetriever
{
public:
  virtual ~ResourceRetriever() = default;

  /// \brief Return whether the resource specified by a URI exists.
  virtual bool exists(const Uri& _uri) = 0;

  /// \brief Return the resource specified by a URI or nullptr on failure.
  virtual ResourcePtr retrieve(const Uri& _uri) = 0;

  // We don't const-qualify for exists and retrieve here. Derived classes of
  // ResourceRetriever will be interacting with external resources that you
  // don't necessarily have control over so we cannot guarantee that you get the
  // same result every time with the same input Uri. Indeed, const-qualification
  // for those functions is pointless.
};

using ResourceRetrieverPtr = std::shared_ptr<ResourceRetriever>;

} // namespace common
} // namespace dart

#endif // ifndef DART_COMMON_RESOURCERETRIEVER_HPP_
