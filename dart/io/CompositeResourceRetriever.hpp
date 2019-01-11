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

#ifndef DART_IO_COMPOSITERESOURCERETRIEVER_HPP_
#define DART_IO_COMPOSITERESOURCERETRIEVER_HPP_

#include <unordered_map>
#include <vector>
#include "dart/common/ResourceRetriever.hpp"

namespace dart {
namespace io {

/// CompositeResourceRetriever allows multiple \ref ResourceRetriever to be
/// used interchangably by: (1) associating each \ref ResourceRetriever with a
/// particular URI schema and/or (2) providing a precedence order for trying
/// multiple retrievers.
class CompositeResourceRetriever : public virtual common::ResourceRetriever
{
public:
  virtual ~CompositeResourceRetriever() = default;

  /// \brief Add a default \ref ResourceRetriever for all URIs.
  /// This \ref ResourceRetriever will be called after all schema-specific
  /// ResourceRetrievers, if any, have failed. This method may be called
  /// multiple times. In that case, the ResourceRetrievers will be queried
  /// in the same order in which they were added.
  void addDefaultRetriever(
    const common::ResourceRetrieverPtr& _resourceRetriever);

  /// \brief Add a default \ref ResourceRetriever for \a _schema
  /// This \ref ResourceRetriever will be called after URIs that match the
  /// specified schema. This method may be called multiple times. In that
  /// case, the ResourceRetrievers will be queried in the same order in which
  /// they were added.
  bool addSchemaRetriever(
    const std::string& _schema,
    const common::ResourceRetrieverPtr& _resourceRetriever);

  // Documentation inherited.
  bool exists(const common::Uri& _uri) override;

  // Documentation inherited.
  common::ResourcePtr retrieve(const common::Uri& _uri) override;

  // Documentation inherited.
  std::string getFilePath(const common::Uri& uri) override;

private:
  std::vector<common::ResourceRetrieverPtr> getRetrievers(
    const common::Uri& _uri) const;

  std::unordered_map<std::string,
    std::vector<common::ResourceRetrieverPtr> > mResourceRetrievers;
  std::vector<common::ResourceRetrieverPtr> mDefaultResourceRetrievers;
};

using CompositeResourceRetrieverPtr
  = std::shared_ptr<CompositeResourceRetriever>;

} // namespace io
} // namespace dart

#endif // ifndef DART_IO_COMPOSITERESOURCERETRIEVER_HPP_
