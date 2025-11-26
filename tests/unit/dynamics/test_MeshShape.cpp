#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/ArrowShape.hpp"
#include "dart/dynamics/AssimpInputResourceAdaptor.hpp"
#include "dart/dynamics/MeshShape.hpp"

#include <Eigen/Core>
#include <assimp/cimport.h>
#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <gtest/gtest.h>

#include <atomic>
#include <fstream>
#include <memory>
#include <string>

using namespace dart;

namespace {

class AliasUriResourceRetriever final : public common::ResourceRetriever
{
public:
  AliasUriResourceRetriever(
      const std::string& aliasUri, const std::string& targetFilePath)
    : mAliasUri(aliasUri),
      mTargetUri(common::Uri::createFromPath(targetFilePath)),
      mDelegate(std::make_shared<common::LocalResourceRetriever>())
  {
  }

  bool exists(const common::Uri& uri) override
  {
    if (isAlias(uri))
      return mDelegate->exists(mTargetUri);
    return mDelegate->exists(uri);
  }

  common::ResourcePtr retrieve(const common::Uri& uri) override
  {
    if (isAlias(uri))
      return mDelegate->retrieve(mTargetUri);
    return mDelegate->retrieve(uri);
  }

private:
  bool isAlias(const common::Uri& uri) const
  {
    return uri.toString() == mAliasUri;
  }

  std::string mAliasUri;
  common::Uri mTargetUri;
  common::LocalResourceRetrieverPtr mDelegate;
};

class RecordingRetriever final : public common::ResourceRetriever
{
public:
  bool exists(const common::Uri&) override
  {
    return true;
  }

  common::ResourcePtr retrieve(const common::Uri&) override
  {
    return nullptr;
  }

  std::string getFilePath(const common::Uri& uri) override
  {
    lastUri = uri.toString();
    return "/virtual/path/from/retriever";
  }

  std::string lastUri;
};

const aiScene* loadMeshWithOverrides(
    const std::string& uri,
    const common::ResourceRetrieverPtr& retriever,
    bool ignoreUnitSize)
{
  aiPropertyStore* propertyStore = aiCreatePropertyStore();
  aiSetImportPropertyInteger(
      propertyStore,
      AI_CONFIG_PP_SBP_REMOVE,
      aiPrimitiveType_POINT | aiPrimitiveType_LINE);

#ifdef AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION
  aiSetImportPropertyInteger(
      propertyStore, AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, 1);
#endif

#ifdef AI_CONFIG_IMPORT_COLLADA_IGNORE_UNIT_SIZE
  if (ignoreUnitSize) {
    aiSetImportPropertyInteger(
        propertyStore, AI_CONFIG_IMPORT_COLLADA_IGNORE_UNIT_SIZE, 1);
  }
#else
  if (ignoreUnitSize) {
    aiReleasePropertyStore(propertyStore);
    return nullptr;
  }
#endif

  dynamics::AssimpInputResourceRetrieverAdaptor systemIO(retriever);
  aiFileIO fileIO = dynamics::createFileIO(&systemIO);

  const aiScene* scene = aiImportFileExWithProperties(
      uri.c_str(),
      aiProcess_GenNormals | aiProcess_Triangulate
          | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType
          | aiProcess_OptimizeMeshes,
      &fileIO,
      propertyStore);

  if (!scene) {
    aiReleasePropertyStore(propertyStore);
    return nullptr;
  }

  scene = aiApplyPostProcessing(scene, aiProcess_PreTransformVertices);
  aiReleasePropertyStore(propertyStore);
  return scene;
}

double readColladaUnitScale(const std::string& path)
{
  std::ifstream file(path);
  if (!file.is_open())
    return 1.0;

  const std::string token = "meter=\"";
  std::string buffer(
      (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  const std::size_t pos = buffer.find(token);
  if (pos == std::string::npos)
    return 1.0;

  const std::size_t start = pos + token.size();
  const std::size_t end = buffer.find('"', start);
  if (end == std::string::npos)
    return 1.0;

  try {
    return std::stod(buffer.substr(start, end - start));
  } catch (...) {
    return 1.0;
  }
}

} // namespace

TEST(MeshShapeTest, CloneCreatesIndependentScene)
{
  const std::string filePath = dart::config::dataPath("skel/kima/l-foot.dae");
  const std::string fileUri = common::Uri::createFromPath(filePath).toString();
  ASSERT_FALSE(fileUri.empty());

  auto retriever = std::make_shared<common::LocalResourceRetriever>();
  const aiScene* scene = dynamics::MeshShape::loadMesh(fileUri, retriever);
  ASSERT_NE(scene, nullptr);

  auto original = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), scene, fileUri, retriever);
  const Eigen::Vector3d originalExtents
      = original->getBoundingBox().computeFullExtents();

  auto cloned
      = std::dynamic_pointer_cast<dynamics::MeshShape>(original->clone());
  ASSERT_NE(cloned, nullptr);
  EXPECT_NE(original->getMesh(), cloned->getMesh());
  EXPECT_TRUE(cloned->getBoundingBox().computeFullExtents().isApprox(
      originalExtents, 1e-12));

  cloned->setScale(Eigen::Vector3d::Constant(2.0));
  EXPECT_TRUE(cloned->getBoundingBox().computeFullExtents().isApprox(
      originalExtents * 2.0, 1e-12));
  EXPECT_TRUE(original->getBoundingBox().computeFullExtents().isApprox(
      originalExtents, 1e-12));

  original.reset(); // releasing imported scene should not break clone
  EXPECT_GT(cloned->getVolume(), 0.0);
}

TEST(MeshShapeTest, ColladaUnitMetadataApplied)
{
#ifndef AI_CONFIG_IMPORT_COLLADA_IGNORE_UNIT_SIZE
  GTEST_SKIP() << "Assimp build does not expose unit-size control property.";
#endif

  const std::string filePath = dart::config::dataPath("skel/kima/l-foot.dae");
  const std::string fileUri = common::Uri::createFromPath(filePath).toString();
  ASSERT_FALSE(fileUri.empty());

  auto retriever = std::make_shared<common::LocalResourceRetriever>();

  const aiScene* sceneWithUnits
      = dynamics::MeshShape::loadMesh(fileUri, retriever);
  ASSERT_NE(sceneWithUnits, nullptr);
  const auto shapeWithUnits = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), sceneWithUnits);
  const Eigen::Vector3d extentsWithUnits
      = shapeWithUnits->getBoundingBox().computeFullExtents();

  const aiScene* sceneIgnoringUnits
      = loadMeshWithOverrides(fileUri, retriever, true);
  ASSERT_NE(sceneIgnoringUnits, nullptr);
  const auto shapeIgnoringUnits = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), sceneIgnoringUnits);
  const Eigen::Vector3d extentsIgnoringUnits
      = shapeIgnoringUnits->getBoundingBox().computeFullExtents();

  const double unitScale = readColladaUnitScale(filePath);
  ASSERT_GT(unitScale, 0.0);

  EXPECT_TRUE(extentsWithUnits.isApprox(extentsIgnoringUnits * unitScale, 1e-6))
      << "extentsWithUnits=" << extentsWithUnits.transpose()
      << ", extentsIgnoringUnits=" << extentsIgnoringUnits.transpose()
      << ", unitScale=" << unitScale;
}

TEST(MeshShapeTest, ColladaUriWithoutExtensionStillLoads)
{
  const std::string filePath = dart::config::dataPath("skel/kima/l-foot.dae");
  const std::string aliasUri = "collada-nodot://meshshape/lfoot";

  auto aliasRetriever
      = std::make_shared<AliasUriResourceRetriever>(aliasUri, filePath);
  const aiScene* aliasScene
      = dynamics::MeshShape::loadMesh(aliasUri, aliasRetriever);
  ASSERT_NE(aliasScene, nullptr);
  const auto aliasShape = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), aliasScene);
  const Eigen::Vector3d aliasExtents
      = aliasShape->getBoundingBox().computeFullExtents();

  auto canonicalRetriever = std::make_shared<common::LocalResourceRetriever>();
  const aiScene* canonicalScene = dynamics::MeshShape::loadMesh(
      common::Uri::createFromPath(filePath).toString(), canonicalRetriever);
  ASSERT_NE(canonicalScene, nullptr);
  const auto canonicalShape = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), canonicalScene);
  const Eigen::Vector3d canonicalExtents
      = canonicalShape->getBoundingBox().computeFullExtents();

  EXPECT_TRUE(aliasExtents.isApprox(canonicalExtents, 1e-6))
      << "aliasExtents=" << aliasExtents.transpose()
      << ", canonicalExtents=" << canonicalExtents.transpose();
}

TEST(MeshShapeTest, RespectsCustomMeshDeleter)
{
  std::atomic<int> deleted{0};

  {
    auto scene = std::shared_ptr<const aiScene>(
        new aiScene, [&deleted](const aiScene* mesh) {
          ++deleted;
          delete const_cast<aiScene*>(mesh);
        });

    dynamics::MeshShape shape(Eigen::Vector3d::Ones(), scene);
    EXPECT_EQ(shape.getMesh(), scene.get());
  }

  EXPECT_EQ(deleted.load(), 1);
}

TEST(MeshShapeTest, TracksOwnershipAndUriMetadata)
{
  auto retriever = std::make_shared<RecordingRetriever>();
  const common::Uri fileUri
      = common::Uri::createFromStringOrPath("/tmp/manual-mesh.dae");

  auto* manualScene = new aiScene;
  dynamics::MeshShape shape(
      Eigen::Vector3d::Ones(),
      manualScene,
      fileUri,
      retriever,
      dynamics::MeshShape::MeshOwnership::Manual);
  EXPECT_EQ(shape.getMesh(), manualScene);
  EXPECT_EQ(shape.getMeshPath(), fileUri.getFilesystemPath());
  EXPECT_EQ(shape.getMeshUri(), fileUri.toString());

  const common::Uri retrieverUri("package://example/mesh.dae");
  auto* retrieverScene = new aiScene;
  shape.setMesh(
      retrieverScene,
      dynamics::MeshShape::MeshOwnership::Manual,
      retrieverUri,
      retriever);
  EXPECT_EQ(retriever->lastUri, retrieverUri.toString());
  EXPECT_EQ(shape.getMeshPath(), "/virtual/path/from/retriever");
  EXPECT_EQ(shape.getMesh(), retrieverScene);

  // No-op when the mesh pointer and ownership are unchanged.
  shape.setMesh(
      shape.getMesh(),
      dynamics::MeshShape::MeshOwnership::Manual,
      retrieverUri,
      retriever);

  // Clearing the mesh resets related metadata.
  shape.setMesh(
      nullptr,
      dynamics::MeshShape::MeshOwnership::Manual,
      common::Uri(),
      nullptr);
  EXPECT_EQ(shape.getMesh(), nullptr);
  EXPECT_TRUE(shape.getMeshPath().empty());
  EXPECT_TRUE(shape.getMeshUri().empty());
}

TEST(ArrowShapeTest, CloneUsesMeshOwnershipSemantics)
{
  dynamics::ArrowShape arrow(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitX(),
      dynamics::ArrowShape::Properties(),
      Eigen::Vector4d::Ones(),
      4);

  auto cloned = std::dynamic_pointer_cast<dynamics::ArrowShape>(arrow.clone());
  ASSERT_TRUE(cloned);
  ASSERT_NE(cloned->getMesh(), nullptr);
  EXPECT_NE(cloned->getMesh(), arrow.getMesh());
}
