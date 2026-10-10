/*
 * Copyright (c) 2026, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD license in the root LICENSE file.
 */

#include <dart/utils/PackageResourceRetriever.hpp>
#include <dart/utils/assets/ModelResourceRetriever.hpp>

#include <dart/common/Uri.hpp>

#include <gtest/gtest.h>
#include <openssl/evp.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

namespace fs = std::filesystem;
using dart::common::Uri;
using dart::utils::ModelResourceRetriever;

std::string sha256(const std::string& content)
{
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int length = 0;
  if (EVP_Digest(
          content.data(),
          content.size(),
          digest,
          &length,
          EVP_sha256(),
          nullptr)
      != 1)
    throw std::runtime_error("Could not hash test fixture");

  std::ostringstream result;
  for (unsigned int index = 0; index < length; ++index)
    result << std::hex << std::setfill('0') << std::setw(2)
           << static_cast<unsigned int>(digest[index]);
  return result.str();
}

void writeFile(const fs::path& path, const std::string& content)
{
  fs::create_directories(path.parent_path());
  std::ofstream output(path, std::ios::binary);
  output << content;
  if (!output)
    throw std::runtime_error("Could not write test fixture");
}

class ModelResourceRetrieverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    static std::atomic<unsigned int> sequence{0};
    const auto timestamp
        = std::chrono::steady_clock::now().time_since_epoch().count();
    directory = fs::temp_directory_path()
                / ("dart-model-retriever-" + std::to_string(timestamp) + "-"
                   + std::to_string(sequence++));
    cache = directory / "cache";
    fs::create_directories(directory);
  }

  void TearDown() override
  {
    std::error_code error;
    fs::remove_all(directory, error);
  }

  std::string manifest(
      const std::string& revision = "v1",
      const std::string& packageRoot = ".") const
  {
    std::ostringstream xml;
    xml << "<model schemaVersion=\"1\" id=\"testbot\" revision=\"" << revision
        << "\" entrypoint=\"robot.urdf\">"
        << "<source url=\"https://example.invalid/pinned-source\"/>"
        << "<license name=\"MIT\"/>"
        << "<package name=\"testbot_description\" path=\"" << packageRoot
        << "\"/>";
    for (const auto& file : files)
      xml << "<file path=\"" << file.first
          << "\" url=\"https://example.invalid/" << file.first << "\" sha256=\""
          << sha256(file.second) << "\" size=\"" << file.second.size()
          << "\"/>";
    xml << "</model>";
    return xml.str();
  }

  Uri registerManifest(
      ModelResourceRetriever& retriever,
      const std::string& xml,
      const std::string& filename = "manifest.xml")
  {
    const fs::path path = directory / filename;
    writeFile(path, xml);
    const Uri uri = Uri::createFromPath(path.string());
    EXPECT_TRUE(retriever.addManifest(uri));
    return uri;
  }

  fs::path populateCache(
      const std::string& xml, const std::string& revision = "v1")
  {
    const fs::path bundle = cache / "testbot" / revision / sha256(xml);
    for (const auto& file : files)
      writeFile(bundle / file.first, file.second);
    return bundle;
  }

  fs::path directory;
  fs::path cache;
  const std::map<std::string, std::string> files{
      {"robot.urdf", "<robot name=\"testbot\"><link name=\"base\"/></robot>"},
      {"meshes/link.obj",
       "mtllib link.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"},
      {"meshes/link.mtl", "newmtl body\nmap_Kd textures/body.png\n"},
      {"meshes/textures/body.png", "test-only-texture"}};
};

TEST_F(ModelResourceRetrieverTest, VerifiedOfflineBundlePreservesRelativeFiles)
{
  const auto xml = manifest();
  const auto bundle = populateCache(xml);
  ModelResourceRetriever retriever(cache.string(), true);
  registerManifest(retriever, xml);

  EXPECT_TRUE(retriever.exists("model://testbot/v1/robot.urdf"));
  EXPECT_EQ(
      retriever.getFilePath("model://testbot/v1/meshes/link.obj"),
      (bundle / "meshes/link.obj").string());
  auto resource = retriever.retrieve("model://testbot/v1/meshes/link.mtl");
  ASSERT_NE(resource, nullptr);
  EXPECT_EQ(resource->readAll(), files.at("meshes/link.mtl"));
  EXPECT_TRUE(fs::is_regular_file(bundle / "meshes/textures/body.png"));

  auto modelRetriever
      = std::make_shared<ModelResourceRetriever>(cache.string(), true);
  registerManifest(*modelRetriever, xml);
  dart::utils::PackageResourceRetriever packageRetriever(modelRetriever);
  packageRetriever.addPackageDirectory(
      "testbot_description", "model://testbot/v1");
  EXPECT_EQ(
      packageRetriever.getFilePath(
          "package://testbot_description/meshes/link.obj"),
      (bundle / "meshes/link.obj").string());
}

TEST_F(ModelResourceRetrieverTest, OfflineMissDoesNotPublishPartialBundle)
{
  const auto xml = manifest();
  ModelResourceRetriever retriever(cache.string(), true);
  registerManifest(retriever, xml);

  EXPECT_FALSE(retriever.exists("model://testbot/v1/robot.urdf"));
  EXPECT_TRUE(retriever.getFilePath("model://testbot/v1/robot.urdf").empty());
  EXPECT_EQ(retriever.retrieve("model://testbot/v1/robot.urdf"), nullptr);
  EXPECT_FALSE(fs::exists(cache / "testbot" / "v1" / sha256(xml)));
}

TEST_F(ModelResourceRetrieverTest, DistinctRevisionsCoexist)
{
  ModelResourceRetriever retriever(cache.string(), true);
  const auto first = manifest();
  const auto second = manifest("v2");
  const auto firstBundle = populateCache(first);
  const auto secondBundle = populateCache(second, "v2");
  registerManifest(retriever, first);
  registerManifest(retriever, second, "second.xml");

  EXPECT_EQ(
      retriever.getFilePath("model://testbot/v1/robot.urdf"),
      (firstBundle / "robot.urdf").string());
  EXPECT_EQ(
      retriever.getFilePath("model://testbot/v2/robot.urdf"),
      (secondBundle / "robot.urdf").string());
  EXPECT_NE(firstBundle, secondBundle);
}

TEST_F(ModelResourceRetrieverTest, RejectsWholeBundleCorruptionOnFirstUse)
{
  const auto xml = manifest();
  const auto bundle = populateCache(xml);
  writeFile(bundle / "meshes/textures/body.png", "corrupt-texture");
  ModelResourceRetriever retriever(cache.string(), true);
  registerManifest(retriever, xml);

  EXPECT_FALSE(retriever.exists("model://testbot/v1/robot.urdf"));
  EXPECT_TRUE(retriever.getFilePath("model://testbot/v1/robot.urdf").empty());
  EXPECT_TRUE(fs::exists(bundle));
}

TEST_F(ModelResourceRetrieverTest, RechecksRequestedFilesAfterAcquisition)
{
  const auto xml = manifest();
  const auto bundle = populateCache(xml);
  ModelResourceRetriever retriever(cache.string(), true);
  registerManifest(retriever, xml);
  ASSERT_TRUE(retriever.exists("model://testbot/v1/robot.urdf"));
  writeFile(bundle / "meshes/link.obj", "corrupt");

  EXPECT_TRUE(
      retriever.getFilePath("model://testbot/v1/meshes/link.obj").empty());
  EXPECT_EQ(retriever.retrieve("model://testbot/v1/meshes/link.obj"), nullptr);
}

TEST_F(ModelResourceRetrieverTest, IgnoresInterruptedStagingDirectories)
{
  const auto xml = manifest();
  const auto staging
      = cache / "testbot" / "v1" / (".tmp-" + sha256(xml) + "-interrupted");
  writeFile(staging / "robot.urdf", files.at("robot.urdf"));
  ModelResourceRetriever retriever(cache.string(), true);
  registerManifest(retriever, xml);

  EXPECT_FALSE(retriever.exists("model://testbot/v1/robot.urdf"));
  EXPECT_TRUE(fs::exists(staging / "robot.urdf"));
}

TEST_F(ModelResourceRetrieverTest, RegistrationsAreIdempotentAndRejectConflicts)
{
  const auto xml = manifest();
  ModelResourceRetriever retriever(cache.string(), true);
  const auto original = registerManifest(retriever, xml);
  EXPECT_TRUE(retriever.addManifest(original));
  const auto conflict = directory / "conflict.xml";
  writeFile(conflict, xml + "\n");
  EXPECT_FALSE(retriever.addManifest(Uri::createFromPath(conflict.string())));
}

TEST_F(ModelResourceRetrieverTest, RejectsUnsafeManifestPathsAndMissingMetadata)
{
  ModelResourceRetriever retriever(cache.string(), true);
  const auto good = manifest();
  for (const std::string unsafe :
       {"../robot.urdf", "/robot.urdf", "..\\robot.urdf"}) {
    auto xml = good;
    const auto at = xml.find("path=\"robot.urdf\"");
    ASSERT_NE(at, std::string::npos);
    xml.replace(
        at,
        std::string("path=\"robot.urdf\"").size(),
        "path=\"" + unsafe + "\"");
    writeFile(directory / "bad.xml", xml);
    EXPECT_FALSE(retriever.addManifest(
        Uri::createFromPath((directory / "bad.xml").string())));
  }
  writeFile(directory / "bad.xml", manifest("v1", "../outside"));
  EXPECT_FALSE(retriever.addManifest(
      Uri::createFromPath((directory / "bad.xml").string())));
  auto noLicense = good;
  noLicense.erase(
      noLicense.find("<license"),
      std::string("<license name=\"MIT\"/>").size());
  writeFile(directory / "bad.xml", noLicense);
  EXPECT_FALSE(retriever.addManifest(
      Uri::createFromPath((directory / "bad.xml").string())));
  EXPECT_FALSE(retriever.addManifest("https://example.invalid/manifest.xml"));
}

TEST_F(ModelResourceRetrieverTest, RejectsUnlistedAndEscapingResourceRequests)
{
  const auto xml = manifest();
  populateCache(xml);
  ModelResourceRetriever retriever(cache.string(), true);
  registerManifest(retriever, xml);

  for (const std::string uri :
       {"model://testbot/v1/../robot.urdf",
        "model://testbot/v1/%2e%2e/robot.urdf",
        "model://testbot/v1/unlisted.urdf",
        "model://testbot/v1/robot.urdf?query=1",
        "model://unknown/v1/robot.urdf",
        "file:///robot.urdf"}) {
    EXPECT_FALSE(retriever.exists(uri)) << uri;
    EXPECT_TRUE(retriever.getFilePath(uri).empty()) << uri;
  }
}

TEST_F(ModelResourceRetrieverTest, RejectsSymlinkedCacheFiles)
{
  const auto xml = manifest();
  const auto bundle = populateCache(xml);
  const auto outside = directory / "outside.obj";
  writeFile(outside, files.at("meshes/link.obj"));
  fs::remove(bundle / "meshes/link.obj");
  std::error_code error;
  fs::create_symlink(outside, bundle / "meshes/link.obj", error);
  if (error)
    GTEST_SKIP() << "Host cannot create test symlinks: " << error.message();
  ModelResourceRetriever retriever(cache.string(), true);
  registerManifest(retriever, xml);

  EXPECT_FALSE(retriever.exists("model://testbot/v1/robot.urdf"));
}

TEST_F(ModelResourceRetrieverTest, RejectsSymlinkedCacheAncestor)
{
  const auto xml = manifest();
  fs::create_directories(cache);
  fs::create_directories(directory / "outside");
  std::error_code error;
  fs::create_directory_symlink(directory / "outside", cache / "testbot", error);
  if (error)
    GTEST_SKIP() << "Host cannot create test symlinks: " << error.message();
  populateCache(xml);
  ModelResourceRetriever retriever(cache.string(), true);
  registerManifest(retriever, xml);

  EXPECT_FALSE(retriever.exists("model://testbot/v1/robot.urdf"));
}

} // namespace
