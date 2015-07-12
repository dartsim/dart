#include <fstream>
#include "dart/common/Console.h"
#include "dart/utils/LocalResourceRetriever.h"

namespace dart {
namespace utils {

ConstMemoryResourcePtr LocalResourceRetriever::retrieve(
  const std::string& _uri)
{
  static const std::string prefix = "file://";

  // Only file:// URIs are local.
  if(_uri.find(prefix) != 0)
    return nullptr;

  const std::string localPath = _uri.substr(prefix.size());
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
