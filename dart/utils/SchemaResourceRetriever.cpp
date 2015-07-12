#include <iostream>
#include "dart/common/Console.h"
#include "dart/utils/SchemaResourceRetriever.h"

namespace dart {
namespace utils {

bool SchemaResourceRetriever::addSchemaRetriever(
  const std::string& _schema, const ResourceRetrieverPtr& _resourceRetriever)
{
  if(!_resourceRetriever)
  {
    dterr << "[SchemaResourceRetriever::addSchemaRetriever] Recieved nullptr"
             " ResourceRetriever; skipping this entry.\n"; 
    return false;
  }

  if(_schema.find("://") != std::string::npos)
  {
    dterr << "[SchemaResourceRetriever::addSchemaRetriever] Schema '"
          << _schema << "' contains '://'. Did you mistakenly include the"
             " '://' in the input of this function?\n";
    return false;
  }

  mResourceRetrievers[_schema].push_back(_resourceRetriever);
  return true;
}

ConstMemoryResourcePtr SchemaResourceRetriever::retrieve(const std::string& _uri)
{
  const auto schemaIndex = _uri.find("://");
  if(schemaIndex == std::string::npos)
  {
    dterr << "[SchemaResourceRetriever::retrieve] Failed to extract schema from"
             " URI '" << _uri << "'. Is this a valid URI?\n";
    return nullptr;
  }

  const std::string schema = _uri.substr(0, schemaIndex);
  const auto resourceRetrieversIt = mResourceRetrievers.find(schema);
  if (resourceRetrieversIt == std::end(mResourceRetrievers))
  {
    dtwarn << "[SchemaResourceRetriever::retrieve] There are no resource"
              " retrievers registered for the schema '" << schema << "'"
              " that is necessary to retrieve URI '" << _uri << "'.\n";
    return nullptr;
  }

  // Sequentially try each ResourceRetriever until one returns success.
  const std::vector<ResourceRetrieverPtr> &retrievers
    = resourceRetrieversIt->second;

  for (const ResourceRetrieverPtr resourceRetriever : retrievers)
  {
    if (ConstMemoryResourcePtr resource = resourceRetriever->retrieve(_uri))
      return resource;
  }

  dtwarn << "[SchemaResourceRetriever::retrieve] All ResourceRetrievers"
            " registered for schema '" << schema << "'" " failed to retrieve"
            " URI '" << _uri << "' (tried " << retrievers.size() << ").\n";

  return nullptr;
}

} // namespace utils
} // namespace dart
