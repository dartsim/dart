#include <iostream>
#include <fstream>
#include "dart/common/Console.h"
#include "dart/utils/LocalResourceRetriever.h"
#include "LocalResource.h"

static std::string getPath(const std::string& _uriOrPath)
{
  static const std::string file_schema = "file://";

  // There is no schema, so we'll interpret the input as a path.
  if(_uriOrPath.find_first_of(':') == std::string::npos)
    return _uriOrPath;
  // This is a file:// URI, so we'll strip off the schema.
  else if(_uriOrPath.size() > file_schema.size()
        && _uriOrPath.substr(0, file_schema.size()) == file_schema)
    return _uriOrPath.substr(file_schema.size());
  else
    return "";
}


namespace dart {
namespace utils {

bool LocalResourceRetriever::exists(const std::string& _uri)
{
  // Open and close the file to check if it exists. It would be more efficient
  // to stat() it, but that is not portable.
  const std::string path = getPath(_uri);
  return !path.empty() && std::ifstream(path, std::ios::binary).good();
}

ResourcePtr LocalResourceRetriever::retrieve(const std::string& _uri)
{
  const std::string path = getPath(_uri);
  if(path.empty())
    return nullptr;

  const auto resource = std::make_shared<LocalResource>(path);
  if(resource->isGood())
    return resource;
  else
    return nullptr;
}

} // namespace utils
} // namespace dart
