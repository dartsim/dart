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

#include <iostream>
#include "dart/common/Console.hpp"
#include "dart/common/Uri.hpp"
#include "dart/utils/CompositeResourceRetriever.hpp"

namespace dart {
namespace utils {

//==============================================================================
void CompositeResourceRetriever::addDefaultRetriever(
  const common::ResourceRetrieverPtr& _resourceRetriever)
{
  mDefaultResourceRetrievers.push_back(_resourceRetriever);
}

//==============================================================================
bool CompositeResourceRetriever::addSchemaRetriever(
  const std::string& _schema,
  const common::ResourceRetrieverPtr& _resourceRetriever)
{
  if(!_resourceRetriever)
  {
    dterr << "[CompositeResourceRetriever::addSchemaRetriever] Receieved"
             " nullptr ResourceRetriever; skipping this entry.\n";
    return false;
  }

  if(_schema.find("://") != std::string::npos)
  {
    dterr << "[CompositeResourceRetriever::addSchemaRetriever] Schema '"
          << _schema << "' contains '://'. Did you mistakenly include the"
             " '://' in the input of this function?\n";
    return false;
  }

  mResourceRetrievers[_schema].push_back(_resourceRetriever);
  return true;
}

//==============================================================================
bool CompositeResourceRetriever::exists(const common::Uri& _uri)
{
  for(const common::ResourceRetrieverPtr& resourceRetriever
      : getRetrievers(_uri))
  {
    if(resourceRetriever->exists(_uri))
      return true;
  }
  return false;
}

//==============================================================================
common::ResourcePtr CompositeResourceRetriever::retrieve(
  const common::Uri& _uri)
{
  const std::vector<common::ResourceRetrieverPtr> &retrievers
    = getRetrievers(_uri);
  for(const common::ResourceRetrieverPtr& resourceRetriever : retrievers)
  {
    if(common::ResourcePtr resource = resourceRetriever->retrieve(_uri))
      return resource;
  }

  dtwarn << "[CompositeResourceRetriever::retrieve] All ResourceRetrievers"
            " registered for this schema failed to retrieve the URI '"
            << _uri.toString() << "' (tried " << retrievers.size() << ").\n";

  return nullptr;
}

//==============================================================================
std::vector<common::ResourceRetrieverPtr>
  CompositeResourceRetriever::getRetrievers(const common::Uri& _uri) const
{
  const std::string schema = _uri.mScheme.get_value_or("file");

  std::vector<common::ResourceRetrieverPtr> retrievers;

  const auto it = mResourceRetrievers.find(schema);
  if(it != std::end(mResourceRetrievers))
    retrievers.insert(
      std::end(retrievers),
      std::begin(it->second),
      std::end(it->second));

  retrievers.insert(
    std::end(retrievers),
    std::begin(mDefaultResourceRetrievers),
    std::end(mDefaultResourceRetrievers));

  if(retrievers.empty())
  {
    dtwarn << "[CompositeResourceRetriever::retrieve] There are no resource"
              " retrievers registered for the schema '" << schema << "'"
              " that is necessary to retrieve URI '" << _uri.toString() <<
              "'.\n";
  }

  return retrievers;
}

} // namespace utils
} // namespace dart
