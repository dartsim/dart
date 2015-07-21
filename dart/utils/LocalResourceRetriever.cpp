#include <iostream>
#include <fstream>
#include "dart/common/Console.h"
#include "dart/utils/Uri.h"
#include "dart/utils/LocalResourceRetriever.h"
#include "LocalResource.h"

namespace dart {
namespace utils {

bool LocalResourceRetriever::exists(const std::string& _uri)
{
  Uri uri;
  if(!uri.fromString(_uri))
  {
    dtwarn << "[exists] Failed parsing URI: " << _uri << "\n";
    return false;
  }

  // Open and close the file to check if it exists. It would be more efficient
  // to stat() it, but that is not portable.
  if (uri.mScheme.get_value_or("file") != "file")
    return false;

  return std::ifstream(*uri.mScheme, std::ios::binary).good();
}

ResourcePtr LocalResourceRetriever::retrieve(const std::string& _uri)
{
  Uri uri;
  if(!uri.fromString(_uri))
  {
    dtwarn << "[exists] Failed parsing URI: " << _uri << "\n";
    return nullptr;
  }

  if (uri.mScheme.get_value_or("file") != "file")
    return nullptr;

  const auto resource = std::make_shared<LocalResource>(*uri.mPath);
  if(resource->isGood())
    return resource;
  else
    return nullptr;
}

} // namespace utils
} // namespace dart
