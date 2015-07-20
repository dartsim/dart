#include <iostream>
#include "dart/common/Console.h"
#include "dart/utils/SchemaResourceRetriever.h"
#include "dart/utils/UriUtils.h"

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

bool SchemaResourceRetriever::exists(const std::string& _uri)
{
  for(const ResourceRetrieverPtr& resourceRetriever : getRetrievers(_uri))
  {
    if(resourceRetriever->exists(_uri))
      return true;
  }
  return false;
}

ResourcePtr SchemaResourceRetriever::retrieve(const std::string& _uri)
{
  const std::vector<ResourceRetrieverPtr> &retrievers = getRetrievers(_uri);
  for(const ResourceRetrieverPtr& resourceRetriever : retrievers)
  {
    if(ResourcePtr resource = resourceRetriever->retrieve(_uri))
      return resource;
  }

  dtwarn << "[SchemaResourceRetriever::retrieve] All ResourceRetrievers"
            " registered for this schema failed to retrieve the URI '"
            << _uri << "' (tried " << retrievers.size() << ").\n";

  return nullptr;
}

const std::vector<ResourceRetrieverPtr>& SchemaResourceRetriever::getRetrievers(
  const std::string& _uri)
{
  static const std::vector<ResourceRetrieverPtr> empty_placeholder;

  const std::string schema = getSchema(_uri);

  const auto it = mResourceRetrievers.find(schema);
  if(it != std::end(mResourceRetrievers))
    return it->second;
  else
  {
    dtwarn << "[SchemaResourceRetriever::retrieve] There are no resource"
              " retrievers registered for the schema '" << schema << "'"
              " that is necessary to retrieve URI '" << _uri << "'.\n";
    return empty_placeholder;
  }
}

std::string SchemaResourceRetriever::getSchema(const std::string& _uri)
{
  Uri uri;
  if(!uri.fromString(_uri))
  {
    dtwarn << "[SchemaResourceRetriever::retrieve] Failed parsing URI:"
           << _uri << "\n";
    return "";
  }

  return uri.mScheme.get_value_or("file");
}

} // namespace utils
} // namespace dart
