/*
 * Copyright (c) 2026, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/utils/assets/ModelResourceRetriever.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/IncludeWindows.hpp"
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/utils/DartResourceRetriever.hpp"

#include <curl/curl.h>
#include <openssl/evp.h>
#include <openssl/rand.h>
#include <tinyxml2.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>
#include <utility>

#include <cstdint>
#include <cstdlib>

namespace dart {
namespace utils {
namespace {
namespace model_assets {

namespace fs = std::filesystem;

struct File
{
  std::string path;
  std::string url;
  std::string sha256;
  std::uint64_t size;
};

struct Manifest
{
  std::string id;
  std::string revision;
  std::string digest;
  std::map<std::string, File> files;
  bool acquired = false;
};

std::string hex(const unsigned char* bytes, std::size_t size)
{
  static const char digits[] = "0123456789abcdef";
  std::string result(size * 2, '0');
  for (std::size_t i = 0; i < size; ++i) {
    result[i * 2] = digits[bytes[i] >> 4];
    result[i * 2 + 1] = digits[bytes[i] & 15];
  }
  return result;
}

std::string digest(const std::string& bytes)
{
  std::array<unsigned char, EVP_MAX_MD_SIZE> value;
  unsigned int size = 0;
  if (EVP_Digest(
          bytes.data(),
          bytes.size(),
          value.data(),
          &size,
          EVP_sha256(),
          nullptr)
      != 1)
    throw std::runtime_error("SHA-256 initialization failed");
  return hex(value.data(), size);
}

bool isIdentifier(const std::string& value)
{
  if (value.empty() || value == "." || value == "..")
    return false;
  for (const char c : value) {
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
          || (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.'))
      return false;
  }
  return true;
}

std::string relativePath(const std::string& value, bool allowDot = false)
{
  if (value.empty() || value.find_first_of("\\:") != std::string::npos
      || value.find('\0') != std::string::npos)
    throw std::runtime_error("Invalid relative bundle path: " + value);

  const fs::path path(value);
  if (path.has_root_path())
    throw std::runtime_error("Absolute bundle path is forbidden: " + value);

  const auto normalized = path.lexically_normal();
  for (const auto& component : normalized) {
    if (component == "..")
      throw std::runtime_error("Bundle path escapes its root: " + value);
  }
  const auto result = normalized.generic_string();
  if (result.empty() || (!allowDot && result == ".")
      || (!allowDot && result.back() == '/'))
    throw std::runtime_error("Bundle path must name a file: " + value);
  return result;
}

std::string attribute(const tinyxml2::XMLElement* element, const char* name)
{
  const auto value = element ? element->Attribute(name) : nullptr;
  if (!value || !*value)
    throw std::runtime_error(
        std::string("Missing manifest attribute: ") + name);
  return value;
}

void requireHttps(const std::string& url)
{
  common::Uri uri;
  if (!uri.fromString(url) || uri.mScheme.get_value_or("") != "https"
      || uri.mAuthority.get_value_or("").empty()
      || uri.mAuthority.get().find('@') != std::string::npos || uri.mFragment)
    throw std::runtime_error("Manifest source must be an HTTPS URL: " + url);
}

std::uint64_t byteCount(const std::string& value)
{
  std::uint64_t result = 0;
  for (const char c : value) {
    if (c < '0' || c > '9'
        || result
               > (std::numeric_limits<std::uint64_t>::max() - (c - '0')) / 10)
      throw std::runtime_error("Invalid manifest byte count: " + value);
    result = result * 10 + (c - '0');
  }
  return result;
}

Manifest parseManifest(const std::string& bytes)
{
  tinyxml2::XMLDocument document;
  if (document.Parse(bytes.data(), bytes.size()) != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("Invalid model manifest XML");
  const auto root = document.RootElement();
  if (!root || std::string(root->Name()) != "model"
      || attribute(root, "schemaVersion") != "1")
    throw std::runtime_error("Expected a model manifest with schemaVersion=1");

  Manifest manifest;
  manifest.id = attribute(root, "id");
  manifest.revision = attribute(root, "revision");
  if (!isIdentifier(manifest.id) || !isIdentifier(manifest.revision))
    throw std::runtime_error("Invalid model ID or revision");
  const auto entrypoint = relativePath(attribute(root, "entrypoint"));
  requireHttps(attribute(root->FirstChildElement("source"), "url"));
  attribute(root->FirstChildElement("license"), "name");

  std::map<std::string, std::string> packages;
  for (auto package = root->FirstChildElement("package"); package;
       package = package->NextSiblingElement("package")) {
    const auto name = attribute(package, "name");
    if (!isIdentifier(name)
        || !packages
                .emplace(name, relativePath(attribute(package, "path"), true))
                .second)
      throw std::runtime_error("Invalid or duplicate model package: " + name);
  }

  for (auto element = root->FirstChildElement("file"); element;
       element = element->NextSiblingElement("file")) {
    File file;
    file.path = relativePath(attribute(element, "path"));
    file.url = attribute(element, "url");
    requireHttps(file.url);
    file.sha256 = attribute(element, "sha256");
    if (file.sha256.size() != 64
        || file.sha256.find_first_not_of("0123456789abcdef")
               != std::string::npos)
      throw std::runtime_error("Invalid SHA-256 for " + file.path);
    file.size = byteCount(attribute(element, "size"));
    if (file.size
        > static_cast<std::uint64_t>(std::numeric_limits<curl_off_t>::max()))
      throw std::runtime_error(
          "File exceeds the supported transfer size: " + file.path);
    if (!manifest.files.emplace(file.path, file).second)
      throw std::runtime_error("Duplicate manifest file: " + file.path);
  }
  if (manifest.files.find(entrypoint) == manifest.files.end())
    throw std::runtime_error("Manifest entrypoint is not a registered file");
  for (const auto& entry : manifest.files) {
    auto parent = fs::path(entry.first).parent_path();
    while (!parent.empty()) {
      if (manifest.files.count(parent.generic_string()))
        throw std::runtime_error("A manifest file is also used as a directory");
      parent = parent.parent_path();
    }
  }
  manifest.digest = digest(bytes);
  return manifest;
}

fs::path defaultCache()
{
#ifdef _WIN32
  if (const auto directory = std::getenv("LOCALAPPDATA")) {
    if (*directory)
      return fs::path(directory) / "dart" / "models";
  }
#elif defined(__APPLE__)
  if (const auto directory = std::getenv("HOME")) {
    if (*directory)
      return fs::path(directory) / "Library" / "Caches" / "dart" / "models";
  }
#else
  if (const auto directory = std::getenv("XDG_CACHE_HOME")) {
    if (*directory && fs::path(directory).is_absolute())
      return fs::path(directory) / "dart" / "models";
  }
  if (const auto directory = std::getenv("HOME")) {
    if (*directory)
      return fs::path(directory) / ".cache" / "dart" / "models";
  }
#endif
  throw std::runtime_error("Cannot locate user cache; provide cacheDirectory");
}

void rejectSymlinks(const fs::path& root, const fs::path& relative)
{
  auto path = root;
  if (fs::is_symlink(fs::symlink_status(path)))
    throw std::runtime_error(
        "Symlink in model cache: " + path.string()
        + ". Remove this symlink before retrying.");
  for (const auto& component : relative) {
    path /= component;
    if (fs::is_symlink(fs::symlink_status(path)))
      throw std::runtime_error(
          "Symlink in model cache: " + path.string()
          + ". Remove this symlink before retrying.");
    if (!fs::exists(path))
      return;
  }
}

void ensureDirectory(const fs::path& root, const fs::path& relative)
{
  rejectSymlinks(root, relative);
  fs::create_directories(root);
  auto path = root;
  for (const auto& component : relative) {
    path /= component;
    fs::create_directory(path);
    if (fs::is_symlink(fs::symlink_status(path)) || !fs::is_directory(path))
      throw std::runtime_error("Invalid cache directory: " + path.string());
  }
}

void verifyFile(const fs::path& root, const File& file)
{
  rejectSymlinks(root, fs::path(file.path));
  const auto path = root / file.path;
  if (!fs::is_regular_file(path) || fs::file_size(path) != file.size)
    throw std::runtime_error(
        "Missing or incorrect-size file: " + path.string());
  std::ifstream input(path, std::ios::binary);
  if (!input)
    throw std::runtime_error("Cannot read model file: " + path.string());
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(
      EVP_MD_CTX_new(), &EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("SHA-256 initialization failed");
  std::array<char, 65536> buffer;
  while (input) {
    input.read(buffer.data(), buffer.size());
    if (EVP_DigestUpdate(context.get(), buffer.data(), input.gcount()) != 1)
      throw std::runtime_error("SHA-256 calculation failed");
  }
  if (!input.eof())
    throw std::runtime_error("Failed reading model file: " + path.string());
  std::array<unsigned char, EVP_MAX_MD_SIZE> value;
  unsigned int size = 0;
  if (EVP_DigestFinal_ex(context.get(), value.data(), &size) != 1
      || hex(value.data(), size) != file.sha256)
    throw std::runtime_error("SHA-256 mismatch: " + path.string());
}

void verifyBundle(const fs::path& root, const Manifest& manifest)
{
  if (fs::is_symlink(fs::symlink_status(root)) || !fs::is_directory(root))
    throw std::runtime_error(
        "Invalid model bundle directory: " + root.string());
  for (const auto& entry : manifest.files)
    verifyFile(root, entry.second);
}

struct Transfer
{
  std::ofstream output;
  std::uint64_t expected;
  std::uint64_t received = 0;
};

std::size_t writeDownload(
    char* bytes, std::size_t size, std::size_t count, void* userdata)
{
  auto& transfer = *static_cast<Transfer*>(userdata);
  if (size && count > std::numeric_limits<std::size_t>::max() / size)
    return 0;
  const auto length = size * count;
  if (length > transfer.expected - transfer.received)
    return 0;
  transfer.output.write(bytes, length);
  if (!transfer.output)
    return 0;
  transfer.received += length;
  return length;
}

void downloadFile(const fs::path& root, const File& file)
{
  static const auto initialized = curl_global_init(CURL_GLOBAL_DEFAULT);
  if (initialized != CURLE_OK)
    throw std::runtime_error("HTTPS initialization failed");
  std::unique_ptr<CURL, decltype(&curl_easy_cleanup)> handle(
      curl_easy_init(), &curl_easy_cleanup);
  if (!handle)
    throw std::runtime_error("HTTPS transfer allocation failed");

  ensureDirectory(root, fs::path(file.path).parent_path());
  Transfer transfer{
      std::ofstream(root / file.path, std::ios::binary), file.size};
  if (!transfer.output)
    throw std::runtime_error("Cannot write model file: " + file.path);
  std::array<char, CURL_ERROR_SIZE> error{};

  const auto setOption = [&](CURLoption option, auto value) {
    if (curl_easy_setopt(handle.get(), option, value) != CURLE_OK)
      throw std::runtime_error("Failed configuring HTTPS transfer");
  };
  setOption(CURLOPT_URL, file.url.c_str());
  setOption(CURLOPT_PROTOCOLS_STR, "https");
  setOption(CURLOPT_REDIR_PROTOCOLS_STR, "https");
  setOption(CURLOPT_FOLLOWLOCATION, 1L);
  setOption(CURLOPT_MAXREDIRS, 5L);
  setOption(CURLOPT_SSL_VERIFYPEER, 1L);
  setOption(CURLOPT_SSL_VERIFYHOST, 2L);
  setOption(CURLOPT_FAILONERROR, 1L);
  setOption(CURLOPT_NOSIGNAL, 1L);
  setOption(CURLOPT_CONNECTTIMEOUT, 5L);
  setOption(CURLOPT_TIMEOUT, 60L);
  setOption(CURLOPT_LOW_SPEED_LIMIT, 1024L);
  setOption(CURLOPT_LOW_SPEED_TIME, 30L);
  setOption(CURLOPT_MAXFILESIZE_LARGE, static_cast<curl_off_t>(file.size));
  setOption(CURLOPT_ERRORBUFFER, error.data());
  setOption(CURLOPT_WRITEFUNCTION, &writeDownload);
  setOption(CURLOPT_WRITEDATA, &transfer);
  const char* trustRoot = std::getenv("CURL_CA_BUNDLE");
  if (!trustRoot || !*trustRoot)
    trustRoot = std::getenv("SSL_CERT_FILE");
  if (trustRoot && *trustRoot)
    setOption(CURLOPT_CAINFO, trustRoot);

  const auto result = curl_easy_perform(handle.get());
  transfer.output.close();
  long response = 0;
  curl_easy_getinfo(handle.get(), CURLINFO_RESPONSE_CODE, &response);
  if (result != CURLE_OK || response != 200 || !transfer.output)
    throw std::runtime_error(
        "Failed HTTPS download of " + file.url + ": "
        + (error[0] ? error.data() : curl_easy_strerror(result)));
  if (transfer.received != file.size)
    throw std::runtime_error("Incorrect download size for " + file.path);
  verifyFile(root, file);
}

struct StagingDirectory
{
  fs::path path;

  ~StagingDirectory()
  {
    std::error_code error;
    fs::remove_all(path, error);
  }
};

fs::path makeStagingDirectory(const fs::path& parent, const std::string& digest)
{
  for (int attempt = 0; attempt < 10; ++attempt) {
    std::array<unsigned char, 16> random;
    if (RAND_bytes(random.data(), random.size()) != 1)
      throw std::runtime_error("Cannot create a unique staging directory");
    const auto path
        = parent / (".tmp-" + digest + "-" + hex(random.data(), random.size()));
    if (fs::create_directory(path))
      return path;
  }
  throw std::runtime_error("Cannot reserve a unique staging directory");
}

void report(const char* operation, const std::exception& error)
{
  dtwarn << "[ModelResourceRetriever::" << operation << "] " << error.what()
         << '\n';
}

} // namespace model_assets
} // namespace

struct ModelResourceRetriever::Impl
{
  model_assets::fs::path cache;
  bool offline;
  std::mutex mutex;
  std::map<std::pair<std::string, std::string>, model_assets::Manifest>
      manifests;
  common::LocalResourceRetriever local;

  Impl(const std::string& directory, bool offlineMode)
    : cache(model_assets::fs::absolute(
                directory.empty() ? model_assets::defaultCache()
                                  : model_assets::fs::path(directory))
                .lexically_normal()),
      offline(offlineMode)
  {
  }

  std::string getFilePath(const common::Uri& uri)
  {
    namespace fs = model_assets::fs;
    if (uri.mScheme.get_value_or("") != "model" || !uri.mAuthority || !uri.mPath
        || uri.mQuery || uri.mFragment)
      throw std::runtime_error("Expected model://id/revision/path URI");
    const auto& id = uri.mAuthority.get();
    const auto& path = uri.mPath.get();
    const auto separator = path.find('/', 1);
    if (path.empty() || path.front() != '/' || separator == std::string::npos)
      throw std::runtime_error("Model URI must include revision and file path");
    const auto revision = path.substr(1, separator - 1);
    const auto relative
        = model_assets::relativePath(path.substr(separator + 1));
    const auto entry = manifests.find({id, revision});
    if (entry == manifests.end())
      throw std::runtime_error(
          "No manifest registered for " + id + "/" + revision);
    auto& manifest = entry->second;
    const auto file = manifest.files.find(relative);
    if (file == manifest.files.end())
      throw std::runtime_error(
          "File is not registered in the manifest: " + relative);

    const auto relativeRoot = fs::path(id) / revision / manifest.digest;
    const auto root = cache / relativeRoot;
    model_assets::rejectSymlinks(cache, relativeRoot);
    if (fs::exists(root)) {
      try {
        if (!manifest.acquired)
          model_assets::verifyBundle(root, manifest);
        model_assets::verifyFile(root, file->second);
      } catch (const std::exception& error) {
        throw std::runtime_error(
            std::string(error.what()) + ". Remove the corrupt bundle directory "
            + root.string() + " explicitly before refetching.");
      }
    } else {
      if (offline)
        throw std::runtime_error(
            "Offline cache miss: " + root.string()
            + ". Fetch this model with an online retriever first.");
      model_assets::ensureDirectory(cache, fs::path(id) / revision);
      model_assets::StagingDirectory staging{model_assets::makeStagingDirectory(
          root.parent_path(), manifest.digest)};
      for (const auto& bundleFile : manifest.files)
        model_assets::downloadFile(staging.path, bundleFile.second);
      model_assets::verifyBundle(staging.path, manifest);
      std::error_code error;
      fs::rename(staging.path, root, error);
      if (error) {
        if (!fs::exists(root))
          throw std::runtime_error(
              "Cannot publish model bundle: " + error.message());
        model_assets::rejectSymlinks(cache, relativeRoot);
        try {
          model_assets::verifyBundle(root, manifest);
        } catch (const std::exception& winnerError) {
          throw std::runtime_error(
              std::string(winnerError.what())
              + ". Remove the corrupt bundle directory " + root.string()
              + " explicitly before refetching.");
        }
      }
    }
    manifest.acquired = true;
    return (root / relative).string();
  }
};

//==============================================================================
ModelResourceRetriever::ModelResourceRetriever(
    const std::string& cacheDirectory, bool offline)
  : mImpl(std::make_unique<Impl>(cacheDirectory, offline))
{
  addManifest("dart://sample/robot_models/atlas-v5.xml");
  addManifest("dart://sample/robot_models/unitree-g1.xml");
}

//==============================================================================
ModelResourceRetriever::~ModelResourceRetriever() = default;

//==============================================================================
bool ModelResourceRetriever::addManifest(const common::Uri& localManifestUri)
{
  std::lock_guard<std::mutex> lock(mImpl->mutex);
  try {
    common::ResourcePtr resource;
    const auto scheme = localManifestUri.mScheme.get_value_or("file");
    if (scheme == "file" && !localManifestUri.mQuery
        && !localManifestUri.mFragment
        && localManifestUri.mAuthority.get_value_or("").empty()) {
      resource = mImpl->local.retrieve(localManifestUri);
    } else if (
        scheme == "dart"
        && localManifestUri.mAuthority.get_value_or("") == "sample"
        && !localManifestUri.mQuery && !localManifestUri.mFragment) {
      DartResourceRetriever retriever;
      resource = retriever.retrieve(localManifestUri);
    } else {
      throw std::runtime_error(
          "Manifest must be a local file or dart://sample/ URI");
    }
    const auto size = resource ? resource->getSize() : 0;
    if (!resource || size > 16 * 1024 * 1024)
      throw std::runtime_error("Manifest is missing or exceeds 16 MiB");
    std::string bytes(size, '\0');
    if (!bytes.empty()
        && resource->read(bytes.data(), 1, bytes.size()) != bytes.size())
      throw std::runtime_error("Cannot read complete manifest");
    auto manifest = model_assets::parseManifest(bytes);
    const auto key = std::make_pair(manifest.id, manifest.revision);
    const auto existing = mImpl->manifests.find(key);
    if (existing != mImpl->manifests.end()) {
      if (existing->second.digest != manifest.digest)
        throw std::runtime_error(
            "Conflicting manifest for " + manifest.id + "/"
            + manifest.revision);
      return true;
    }
    mImpl->manifests.emplace(key, std::move(manifest));
    return true;
  } catch (const std::exception& error) {
    model_assets::report("addManifest", error);
    return false;
  }
}

//==============================================================================
bool ModelResourceRetriever::exists(const common::Uri& uri)
{
  return !getFilePath(uri).empty();
}

//==============================================================================
common::ResourcePtr ModelResourceRetriever::retrieve(const common::Uri& uri)
{
  const auto path = getFilePath(uri);
  return path.empty()
             ? nullptr
             : mImpl->local.retrieve(common::Uri::createFromPath(path));
}

//==============================================================================
std::string ModelResourceRetriever::getFilePath(const common::Uri& uri)
{
  std::lock_guard<std::mutex> lock(mImpl->mutex);
  try {
    return mImpl->getFilePath(uri);
  } catch (const std::exception& error) {
    model_assets::report("getFilePath", error);
    return "";
  }
}

} // namespace utils
} // namespace dart
