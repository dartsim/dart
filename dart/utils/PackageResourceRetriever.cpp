#include <cassert>
#include <sstream>
#include <iostream>
#include "dart/common/Console.h"
#include "dart/common/LocalResourceRetriever.h"
#include "dart/common/Uri.h"
#include "dart/utils/PackageResourceRetriever.h"

namespace dart {
namespace utils {

PackageResourceRetriever::PackageResourceRetriever(
  const common::ResourceRetrieverPtr& localRetriever)
{
  if (localRetriever)
    mLocalRetriever = localRetriever;
  else
    mLocalRetriever = std::make_shared<common::LocalResourceRetriever>();
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

bool PackageResourceRetriever::exists(const std::string& _uri)
{
  std::string packageName, relativePath;
  if (!resolvePackageUri(_uri, packageName, relativePath))
    return false;

  for(const std::string &packagePath : getPackagePaths(packageName))
  {
    const std::string localUri = "file://" + packagePath + relativePath;
    if (mLocalRetriever->exists(localUri))
      return true;
  }
  return false;
}

common::ResourcePtr PackageResourceRetriever::retrieve(const std::string& _uri)
{
  std::string packageName, relativePath;
  if (!resolvePackageUri(_uri, packageName, relativePath))
    return nullptr;

  for(const std::string &packagePath : getPackagePaths(packageName))
  {
    const std::string localUri = "file://" + packagePath + relativePath;
    if(const auto resource = mLocalRetriever->retrieve(localUri))
      return resource;
  }
  return nullptr;
}

const std::vector<std::string>& PackageResourceRetriever::getPackagePaths(
  const std::string& _packageName)
{
  static const std::vector<std::string> empty_placeholder;

  // Lookup the corresponding package path.
  const auto it = mPackageMap.find(_packageName);
  if(it != std::end(mPackageMap))
    return it->second;
  else
  {
    dtwarn << "[PackageResourceResolver::getPackagePaths] Unable to resolve"
              "path to package '" << _packageName << "'. Did you call"
              " addPackageDirectory(~) for this package name?\n";
    return empty_placeholder;
  }
}

bool PackageResourceRetriever::resolvePackageUri(
  const std::string& _uri, std::string& _packageName,
  std::string& _relativePath)
{
  Uri uri;
  if(!uri.fromString(_uri))
  {
    dtwarn << "[PackageResourceRetriever::resolvePackageUri] Failed parsing
              " URI '" << _uri << "'.\n";
    return false;
  }

  if(uri.mSchema.get_or_default("file") != "package")
    return false;

  if(!uri.mAuthority)
  {
    dtwarn << "[PackageResourceRetriever::resolvePackageUri] Failed extracting"
              " package name from URI '" << _uri << "'.\n";
    return false;
  }
  _packageName = *uri.mAuthority;

  if(!uri.mPath)
  {
    dtwarn << "[PackageResourceRetriever::resolvePackageUri] Failed extracting"
              " relative path from URI '" << _uri << "'.\n";
    return false;
  }
  _relativePath = uri.mPath.get_or_default("");

  return true;
}

} // namespace utils
} // namespace dart
