#include "dart/common/Deprecated.hpp"
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/ArrowShape.hpp"
#include "dart/dynamics/AssimpInputResourceAdaptor.hpp"
#include "dart/dynamics/MeshShape.hpp"

#include <assimp/cimport.h>
#include <assimp/config.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <limits>
#include <memory>
#include <string>

using namespace dart;

DART_SUPPRESS_DEPRECATED_BEGIN

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

  std::string getFilePath(const common::Uri& uri) override
  {
    if (isAlias(uri))
      return mDelegate->getFilePath(mTargetUri);
    return mDelegate->getFilePath(uri);
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

double maxVertexZ(const math::TriMesh<double>& mesh)
{
  double maxZ = -std::numeric_limits<double>::infinity();
  for (const auto& vertex : mesh.getVertices())
    maxZ = std::max(maxZ, vertex.z());

  return maxZ;
}

} // namespace

TEST(MeshShapeTest, TriMeshConstructorUpdatesBoundsAndAssimpBridge)
{
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->reserveVertices(3);
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(2.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 3.0, 4.0);
  triMesh->addTriangle(0, 1, 2);
  triMesh->computeVertexNormals();

  dynamics::MeshShape shape(Eigen::Vector3d(2.0, 3.0, 4.0), triMesh);
  EXPECT_EQ(shape.getTriMesh(), triMesh);
  EXPECT_TRUE(shape.getBoundingBox().computeFullExtents().isApprox(
      Eigen::Vector3d(4.0, 9.0, 16.0), 1e-12));

  const aiScene* scene = shape.getMesh();
  ASSERT_NE(scene, nullptr);
  ASSERT_GT(scene->mNumMeshes, 0u);
  EXPECT_EQ(shape.getMesh(), scene);

  auto clone = std::dynamic_pointer_cast<dynamics::MeshShape>(shape.clone());
  ASSERT_NE(clone, nullptr);
  EXPECT_NE(clone->getTriMesh(), triMesh);
  EXPECT_TRUE(clone->getBoundingBox().computeFullExtents().isApprox(
      shape.getBoundingBox().computeFullExtents(), 1e-12));
}

TEST(MeshShapeTest, CloneCreatesIndependentScene)
{
  const std::string filePath = DART_DATA_LOCAL_PATH "sdf/atlas/r_scap.dae";
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
  const aiScene* clonedScene = cloned->getMesh();
  ASSERT_NE(clonedScene, nullptr);
  EXPECT_NE(original->getMesh(), clonedScene);
  EXPECT_TRUE(cloned->getBoundingBox().computeFullExtents().isApprox(
      originalExtents, 1e-12));

  ASSERT_GT(scene->mNumMaterials, 0u);
  ASSERT_GT(clonedScene->mNumMaterials, 0u);
  aiString originalTexture;
  aiString clonedTexture;
  ASSERT_EQ(
      scene->mMaterials[0]->GetTexture(
          aiTextureType_DIFFUSE, 0, &originalTexture),
      AI_SUCCESS);
  ASSERT_EQ(
      clonedScene->mMaterials[0]->GetTexture(
          aiTextureType_DIFFUSE, 0, &clonedTexture),
      AI_SUCCESS);
  EXPECT_STREQ(originalTexture.C_Str(), clonedTexture.C_Str());

  cloned->setScale(Eigen::Vector3d::Constant(2.0));
  EXPECT_TRUE(cloned->getBoundingBox().computeFullExtents().isApprox(
      originalExtents * 2.0, 1e-12));
  EXPECT_TRUE(original->getBoundingBox().computeFullExtents().isApprox(
      originalExtents, 1e-12));

  original.reset(); // releasing imported scene should not break clone
  EXPECT_GT(cloned->getVolume(), 0.0);
}

TEST(MeshShapeTest, ReusingMeshPointerRefreshesMetadata)
{
  const std::string filePath = DART_DATA_LOCAL_PATH "skel/kima/l-foot.dae";
  const std::string fileUri = common::Uri::createFromPath(filePath).toString();
  ASSERT_FALSE(fileUri.empty());

  auto retriever = std::make_shared<common::LocalResourceRetriever>();
  const aiScene* scene = dynamics::MeshShape::loadMesh(fileUri, retriever);
  ASSERT_NE(scene, nullptr);

  auto mesh = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), scene, fileUri, retriever);
  EXPECT_EQ(mesh->getMeshUri(), fileUri);

  // Reuse the SAME aiScene* but supply a different uri. The mesh pointer must
  // be kept (not freed/replaced), and the public metadata must still be
  // refreshed so a metadata-only update is not dropped.
  const std::string otherUri
      = common::Uri::createFromPath(DART_DATA_LOCAL_PATH "skel/kima/r-foot.dae")
            .toString();
  ASSERT_FALSE(otherUri.empty());
  ASSERT_NE(otherUri, fileUri);

  mesh->setMesh(scene, otherUri, retriever);
  EXPECT_EQ(mesh->getMesh(), scene);
  EXPECT_EQ(mesh->getMeshUri(), otherUri);
  EXPECT_EQ(mesh->getMeshPath(), retriever->getFilePath(common::Uri(otherUri)));
}

TEST(MeshShapeTest, ArrowShapeRefreshesTriMeshAfterPositionUpdates)
{
  dynamics::ArrowShape arrow(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ());

  const auto originalMesh = arrow.getTriMesh();
  ASSERT_NE(originalMesh, nullptr);
  const double originalMaxZ = maxVertexZ(*originalMesh);

  arrow.setPositions(Eigen::Vector3d::Zero(), 2.0 * Eigen::Vector3d::UnitZ());

  const auto updatedMesh = arrow.getTriMesh();
  ASSERT_NE(updatedMesh, nullptr);
  EXPECT_NE(updatedMesh, originalMesh);
  EXPECT_GT(maxVertexZ(*updatedMesh), originalMaxZ * 1.5);

  auto clone = std::dynamic_pointer_cast<dynamics::ArrowShape>(arrow.clone());
  ASSERT_NE(clone, nullptr);
  EXPECT_GT(clone->getBoundingBox().computeFullExtents().z(), 1.5);
}

TEST(MeshShapeTest, ColladaUnitMetadataApplied)
{
#ifndef AI_CONFIG_IMPORT_COLLADA_IGNORE_UNIT_SIZE
  GTEST_SKIP() << "Assimp build does not expose unit-size control property.";
#endif

  const std::string filePath = DART_DATA_LOCAL_PATH "skel/kima/l-foot.dae";
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
  const std::string filePath = DART_DATA_LOCAL_PATH "skel/kima/l-foot.dae";
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

DART_SUPPRESS_DEPRECATED_END
