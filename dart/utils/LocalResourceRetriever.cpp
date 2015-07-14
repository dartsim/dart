#include <fstream>
#include <boost/filesystem/operations.hpp>
#include "dart/common/Console.h"
#include "dart/utils/LocalResourceRetriever.h"
#include "LocalResource.h"

static const std::string FILE_SCHEMA = "file://";

namespace dart {
namespace utils {

bool LocalResourceRetriever::exists(const std::string& _uri)
{
  if(_uri.size() >= FILE_SCHEMA.size())
    return boost::filesystem::exists(_uri.substr(FILE_SCHEMA.size()));
  else
    return false;
}

ResourcePtr LocalResourceRetriever::retrieve(const std::string& _uri)
{
  // Only file:// URIs are local.
  if(_uri.find(FILE_SCHEMA) != 0)
    return nullptr;

  const std::string localPath = _uri.substr(FILE_SCHEMA.size());
  std::ifstream stream(localPath.c_str(), std::ios::binary);

  return std::make_shared<LocalResource>(localPath);
}

} // namespace utils
} // namespace dart
