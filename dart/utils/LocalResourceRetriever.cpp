#include <fstream>
#include <boost/filesystem/operations.hpp>
#include "dart/common/Console.h"
#include "dart/utils/LocalResourceRetriever.h"

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

ConstMemoryResourcePtr LocalResourceRetriever::retrieve(
  const std::string& _uri)
{
  // Only file:// URIs are local.
  if(_uri.find(FILE_SCHEMA) != 0)
    return nullptr;

  const std::string localPath = _uri.substr(FILE_SCHEMA.size());
  std::ifstream stream(localPath.c_str(), std::ios::binary);

  // Compute the length of the file.
  std::streampos size = stream.tellg();
  stream.seekg(0, std::ios::end);
  size = stream.tellg() - size;
  stream.seekg(0, std::ios::beg);

  // Read the full contents of the file.
  const auto resource = std::make_shared<MemoryResource>(
    new uint8_t[size], size, true);
  stream.read(reinterpret_cast<char *>(resource->getData()), size);

  if(!stream.good())
  {
    dtwarn << "[LocalResourceRetriever::retrieve] Failed to read file '"
           << localPath << "'.\n";
    return nullptr;
  }
  return resource;
}

} // namespace utils
} // namespace dart
