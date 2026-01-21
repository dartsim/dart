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

#include "dart/utils/composite_resource_retriever.hpp"

#include "dart/common/diagnostics.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/uri.hpp"

#include <iostream>

namespace dart {
namespace utils {

//==============================================================================
void CompositeResourceRetriever::addDefaultRetriever(
    const common::ResourceRetrieverPtr& resourceRetriever)
{
  mDefaultResourceRetrievers.push_back(resourceRetriever);
}

//==============================================================================
bool CompositeResourceRetriever::addSchemaRetriever(
    std::string_view schema,
    const common::ResourceRetrieverPtr& resourceRetriever)
{
  if (!resourceRetriever) {
    DART_ERROR(
        "{}", "Received nullptr ResourceRetriever; skipping this entry.\n");
    return false;
  }

  if (schema.find("://") != std::string_view::npos) {
    DART_ERROR(
        "Schema '{}{}",
        schema,
        "' contains '://'. Did you mistakenly include the '://' in the input "
        "of this function?\n");
    return false;
  }

  const std::string schemaString(schema);
  mResourceRetrievers[schemaString].push_back(resourceRetriever);
  return true;
}

//==============================================================================
bool CompositeResourceRetriever::exists(const common::Uri& uri)
{
  for (const common::ResourceRetrieverPtr& resourceRetriever :
       getRetrievers(uri)) {
    if (resourceRetriever->exists(uri))
      return true;
  }
  return false;
}

//==============================================================================
common::ResourcePtr CompositeResourceRetriever::retrieve(const common::Uri& uri)
{
  const std::vector<common::ResourceRetrieverPtr>& retrievers
      = getRetrievers(uri);
  for (const common::ResourceRetrieverPtr& resourceRetriever : retrievers) {
    if (common::ResourcePtr resource = resourceRetriever->retrieve(uri))
      return resource;
  }

  DART_WARN(
      "{}{}' (tried {}).",
      "All ResourceRetrievers registered for this schema failed to retrieve "
      "the URI '",
      uri.toString(),
      retrievers.size());

  return nullptr;
}

//==============================================================================
DART_SUPPRESS_DEPRECATED_BEGIN
std::string CompositeResourceRetriever::getFilePath(const common::Uri& uri)
{
  for (const auto& resourceRetriever : getRetrievers(uri)) {
    const auto path = resourceRetriever->getFilePath(uri);
    if (!path.empty())
      return path;
  }

  return "";
}
DART_SUPPRESS_DEPRECATED_END

//==============================================================================
std::vector<common::ResourceRetrieverPtr>
CompositeResourceRetriever::getRetrievers(const common::Uri& uri) const
{
  const std::string schema = uri.mScheme.get_value_or("file");

  std::vector<common::ResourceRetrieverPtr> retrievers;

  const auto it = mResourceRetrievers.find(schema);
  if (it != std::end(mResourceRetrievers))
    retrievers.insert(
        std::end(retrievers), std::begin(it->second), std::end(it->second));

  retrievers.insert(
      std::end(retrievers),
      std::begin(mDefaultResourceRetrievers),
      std::end(mDefaultResourceRetrievers));

  DART_WARN_IF(
      retrievers.empty(),
      "{}{}{}{}'.",
      "There are no resource retrievers registered for the schema '",
      schema,
      "' that is necessary to retrieve URI '",
      uri.toString());

  return retrievers;
}

} // namespace utils
} // namespace dart
