// Copyright (c) 2011, The DART development contributors

#include <dart/common/diagnostics.hpp>
#include <dart/common/macros.hpp>
#include <dart/common/uri.hpp>

#include <dart/io/http_resource_retriever.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#include <cstdint>

using namespace dart;
using namespace dart::io;

namespace dart::test {
namespace {

std::string sanitizeFileName(std::string_view name)
{
  if (name.empty()) {
    return "resource";
  }

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

  if (sanitized.empty()) {
    return "resource";
  }

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
    if (!fragment.empty() && url.ends_with(fragment)) {
      url.erase(url.size() - fragment.size());
    }
  }

  const std::string hash = computeHash(url);

  std::string leaf = uri.mPath.get_value_or("");
  if (!leaf.empty()) {
    const auto pos = leaf.find_last_of('/');
    if (pos != std::string::npos) {
      leaf = leaf.substr(pos + 1);
    }
  }

  const auto queryPos = leaf.find_first_of("?&");
  if (queryPos != std::string::npos) {
    leaf = leaf.substr(0, queryPos);
  }

  if (leaf.empty()) {
    leaf = "resource";
  }

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

  std::string filePath;
  {
    auto resource = retriever.retrieve(uri);
    ASSERT_NE(resource, nullptr);
    EXPECT_EQ(resource->readAll(), payload);

    DART_SUPPRESS_DEPRECATED_BEGIN
    filePath = retriever.getFilePath(uri);
    DART_SUPPRESS_DEPRECATED_END
  } // resource released before cleanup

  EXPECT_EQ(filePath, cachePath.string());

  std::error_code ec;
  std::filesystem::remove_all(cacheDir, ec);
  // Ignore cleanup errors on Windows where file locks can linger
}

TEST(HttpResourceRetriever, DefaultOptionsAndRootCacheHit)
{
  HttpResourceRetriever retriever;
  EXPECT_EQ(retriever.getOptions().userAgent, "dart-http-resource-retriever");

  HttpResourceRetriever::Options updatedOptions = retriever.getOptions();
  updatedOptions.followRedirects = false;
  updatedOptions.userAgent = "dart-http-updated";
  retriever.setOptions(updatedOptions);
  EXPECT_FALSE(retriever.getOptions().followRedirects);
  EXPECT_EQ(retriever.getOptions().userAgent, updatedOptions.userAgent);

  const auto cacheDir = makeTempDir("_root_cache");
  retriever.setCacheDirectory(cacheDir);
  EXPECT_EQ(retriever.getCacheDirectory(), cacheDir);

  common::Uri uri;
  ASSERT_TRUE(uri.fromString("https://example.com"));

  const auto cachePath = computeCachePath(cacheDir, uri);
  EXPECT_TRUE(cachePath.filename().string().ends_with("_resource"));
  std::filesystem::create_directories(cachePath.parent_path());

  const std::string payload = "root cached data";
  {
    std::ofstream out(cachePath, std::ios::binary);
    out << payload;
  }

  EXPECT_TRUE(retriever.exists(uri));
  auto resource = retriever.retrieve(uri);
  ASSERT_NE(resource, nullptr);
  EXPECT_EQ(resource->readAll(), payload);

  common::Uri unsupportedUri;
  ASSERT_TRUE(unsupportedUri.fromString("ftp://example.com/resource.txt"));
  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_TRUE(retriever.getFilePath(unsupportedUri).empty());
  DART_SUPPRESS_DEPRECATED_END

  std::error_code ec;
  std::filesystem::remove_all(cacheDir, ec);
}

TEST(HttpResourceRetriever, UnsupportedAndRemoteFailures)
{
  HttpResourceRetriever::Options options;
  options.connectTimeout = std::chrono::milliseconds(5);
  options.transferTimeout = std::chrono::milliseconds(5);
  HttpResourceRetriever retriever(options);

  const auto cacheDir = makeTempDir("_remote_failure_cache");
  retriever.setCacheDirectory(cacheDir);

  common::Uri fileUri;
  ASSERT_TRUE(fileUri.fromString("file:///tmp/does-not-exist.txt"));
  EXPECT_FALSE(retriever.exists(fileUri));
  EXPECT_EQ(retriever.retrieve(fileUri), nullptr);

  common::Uri remoteUri;
  ASSERT_TRUE(remoteUri.fromString("http://127.0.0.1:9/missing"));
  const auto cachePath = computeCachePath(cacheDir, remoteUri);

  EXPECT_FALSE(retriever.exists(remoteUri));
  EXPECT_EQ(retriever.retrieve(remoteUri), nullptr);
  EXPECT_FALSE(std::filesystem::exists(cachePath));

  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_TRUE(retriever.getFilePath(remoteUri).empty());
  DART_SUPPRESS_DEPRECATED_END
  EXPECT_FALSE(std::filesystem::exists(cachePath));

  std::error_code ec;
  std::filesystem::remove_all(cacheDir, ec);
}

TEST(HttpResourceRetriever, InvalidCacheDirectoryShortCircuitsRetrieval)
{
  HttpResourceRetriever retriever;
  const auto tempDir = makeTempDir("_blocked_cache");
  std::filesystem::create_directories(tempDir);

  const auto blocker = tempDir / "not_a_directory";
  {
    std::ofstream out(blocker, std::ios::binary);
    out << "blocking file";
  }

  retriever.setCacheDirectory(blocker / "child");

  common::Uri uri;
  ASSERT_TRUE(uri.fromString("http://example.com/resource.txt"));
  EXPECT_EQ(retriever.retrieve(uri), nullptr);

  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_TRUE(retriever.getFilePath(uri).empty());
  DART_SUPPRESS_DEPRECATED_END

  std::error_code ec;
  std::filesystem::remove_all(tempDir, ec);
}

} // namespace dart::test
