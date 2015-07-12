#include <cassert>
#include <iostream>
#include "dart/common/Console.h"
#include "dart/utils/LocalResourceRetriever.h"
#include "dart/utils/ResourceRetriever.h"
#include "dart/utils/PackageResourceRetriever.h"

namespace dart {
namespace utils {

PackageResourceRetriever::PackageResourceRetriever(
  const ResourceRetrieverPtr& localRetriever)
{
  if (localRetriever)
    mLocalRetriever = localRetriever;
  else
    mLocalRetriever = std::make_shared<LocalResourceRetriever>();
}

void PackageResourceRetriever::addPackageDirectory(
  const std::string& _packageName, const std::string& _packageDirectory)
{
  // Strip a trailing slash.
  std::string normalizedPackageDirectory;
  if(!_packageDirectory.empty() && _packageDirectory.back() == '/')
    normalizedPackageDirectory
      = _packageDirectory.substr(0, _packageDirectory.size() - 1);
  else
    normalizedPackageDirectory = _packageDirectory;

  mPackageMap[_packageName].push_back(normalizedPackageDirectory);
}

ConstMemoryResourcePtr PackageResourceRetriever::retrieve(
  const std::string& _uri)
{
  static const std::string schema = "package://";

  if (_uri.find(schema) != 0)
    return nullptr;

  // Split the URI into package and relative path components.
  const size_t packageIndex = schema.size();
  const size_t pathIndex = _uri.find_first_of('/', packageIndex);

  if(pathIndex == std::string::npos)
  {
    dtwarn << "[PackageResourceRetriever::retrieve] Unable to find package"
              " name in URI '" << _uri << "'.\n";
    return nullptr;
  }

  assert(pathIndex >= packageIndex);
  const std::string packageName = _uri.substr(packageIndex, pathIndex - packageIndex);
  const std::string relativePath = _uri.substr(pathIndex);

  // Lookup the corresponding package path.
  const auto it = mPackageMap.find(packageName);
  if(it == std::end(mPackageMap))
  {
    dtwarn << "[PackageResourceResolver::retrieve] Unable to resolve path to"
              " package '" << packageName << "' while retrieving '" << _uri
           << "'. Did you call addPackageDirectory(~) with this package"
              " name?\n";
    return nullptr;
  }

  // Try each path, in sequence.
  for(const std::string &packagePath : it->second)
  {
    const std::string localUri = "file://" + packagePath + relativePath;
    if(const auto resource = mLocalRetriever->retrieve(localUri))
      return resource;
  }
  return nullptr;
}

} // namespace utils
} // namespace dart
