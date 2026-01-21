// Copyright (c) 2011-2025, The DART development contributors

#include <dart/utils/http_resource_retriever.hpp>

#include <dart/common/diagnostics.hpp>
#include <dart/common/macros.hpp>
#include <dart/common/uri.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#include <cstdint>

using namespace dart;
using namespace dart::utils;

namespace dart::test {
namespace {

std::string sanitizeFileName(std::string_view name)
{
  if (name.empty())
    return "resource";

  std::string sanitized;
  sanitized.reserve(name.size());
  for (char c : name) {
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
        || (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '_') {
      sanitized.push_back(c);
    } else {
      sanitized.push_back('_');
    }
  }

  if (sanitized.empty())
    return "resource";

  return sanitized;
}

std::string computeHash(std::string_view input)
{
  std::uint64_t hash = 1469598103934665603ULL;
  for (unsigned char c : input) {
    hash ^= c;
    hash *= 1099511628211ULL;
  }

  std::ostringstream oss;
  oss << std::hex << std::setw(16) << std::setfill('0') << hash;
  return oss.str();
}

std::filesystem::path computeCachePath(
    const std::filesystem::path& cacheDir, const common::Uri& uri)
{
  std::string url = uri.toString();
  if (uri.mFragment) {
    const auto fragment = "#" + uri.mFragment.get_value_or("");
    if (!fragment.empty() && url.size() >= fragment.size()) {
      if (url.compare(url.size() - fragment.size(), fragment.size(), fragment)
          == 0) {
        url.erase(url.size() - fragment.size());
      }
    }
  }

  const std::string hash = computeHash(url);

  std::string leaf = uri.mPath.get_value_or("");
  if (!leaf.empty()) {
    const auto pos = leaf.find_last_of('/');
    if (pos != std::string::npos)
      leaf = leaf.substr(pos + 1);
  }

  const auto queryPos = leaf.find_first_of("?&");
  if (queryPos != std::string::npos)
    leaf = leaf.substr(0, queryPos);

  if (leaf.empty())
    leaf = "resource";

  const auto sanitizedLeaf = sanitizeFileName(leaf);
  return cacheDir / (hash + "_" + sanitizedLeaf);
}

std::filesystem::path makeTempDir(const std::string& suffix)
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path()
         / ("dart_http_cache_test_" + std::to_string(now) + suffix);
}

} // namespace

TEST(HttpResourceRetriever, CacheHitAndOptions)
{
  HttpResourceRetriever::Options options;
  options.followRedirects = false;
  options.connectTimeout = std::chrono::milliseconds(5);
  options.transferTimeout = std::chrono::milliseconds(5);
  options.maxDownloadBytes = 1024;
  options.userAgent = "dart-http-test";

  HttpResourceRetriever retriever(options);
  EXPECT_EQ(retriever.getOptions().userAgent, options.userAgent);

  const auto cacheDir = makeTempDir("_cache");
  retriever.setCacheDirectory(cacheDir);

  common::Uri uri;
  ASSERT_TRUE(uri.fromString(
      "http://example.com/path/Name With Space.txt?query=1#frag"));

  const auto cachePath = computeCachePath(cacheDir, uri);
  std::filesystem::create_directories(cachePath.parent_path());

  const std::string payload = "cached data";
  {
    std::ofstream out(cachePath, std::ios::binary);
    out << payload;
  }

  EXPECT_TRUE(retriever.exists(uri));
  auto resource = retriever.retrieve(uri);
  ASSERT_NE(resource, nullptr);
  EXPECT_EQ(resource->readAll(), payload);

  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_EQ(retriever.getFilePath(uri), cachePath.string());
  DART_SUPPRESS_DEPRECATED_END

  std::filesystem::remove_all(cacheDir);
}

TEST(HttpResourceRetriever, UnsupportedAndRemoteFailures)
{
  HttpResourceRetriever::Options options;
  options.connectTimeout = std::chrono::milliseconds(5);
  options.transferTimeout = std::chrono::milliseconds(5);
  HttpResourceRetriever retriever(options);

  common::Uri fileUri;
  ASSERT_TRUE(fileUri.fromString("file:///tmp/does-not-exist.txt"));
  EXPECT_FALSE(retriever.exists(fileUri));
  EXPECT_EQ(retriever.retrieve(fileUri), nullptr);

  common::Uri remoteUri;
  ASSERT_TRUE(remoteUri.fromString("http://127.0.0.1:9/missing"));
  EXPECT_FALSE(retriever.exists(remoteUri));
  EXPECT_EQ(retriever.retrieve(remoteUri), nullptr);
}

} // namespace dart::test
