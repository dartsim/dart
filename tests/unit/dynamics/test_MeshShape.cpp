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

std::unique_ptr<aiScene> createCustomMeshScene()
{
  auto scene = std::make_unique<aiScene>();
  scene->mRootNode = new aiNode;
  scene->mRootNode->mNumMeshes = 2;
  scene->mRootNode->mMeshes = new unsigned int[2]{0, 1};
  scene->mNumMeshes = 2;
  scene->mMeshes = new aiMesh* [2] {
  };
  for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
    auto* mesh = new aiMesh;
    scene->mMeshes[i] = mesh;
    mesh->mMaterialIndex = static_cast<unsigned int>(-1);
    mesh->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
    mesh->mNumVertices = 3;
    const auto z = static_cast<ai_real>(i);
    mesh->mVertices = new aiVector3D[3]{{0, 0, z}, {2, 0, z}, {0, 3, z}};
    mesh->mNormals = new aiVector3D[3]{{0, 0, 1}, {0, 0, 1}, {0, 0, 1}};
    mesh->mNumFaces = 1;
    mesh->mFaces = new aiFace[1];
    mesh->mFaces[0].mNumIndices = 3;
    mesh->mFaces[0].mIndices = new unsigned int[3]{0, 1, 2};
  }
  return scene;
}

// Mirror gz-physics' CustomMeshShape, including its protected mesh assignment.
class CustomMeshShape final : public dynamics::MeshShape
{
public:
  explicit CustomMeshShape(const Eigen::Vector3d& scale)
    : MeshShape(scale, nullptr)
  {
    mMesh = createCustomMeshScene().release();
    mIsBoundingBoxDirty = true;
    mIsVolumeDirty = true;
  }

  void reassignMesh()
  {
    mMesh = mMesh.get();
  }

  MeshOwnership getMeshOwnership() const
  {
    return mMesh.getOwnership();
  }
};

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

  aiSetImportPropertyInteger(
      propertyStore, AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, 1);

  if (ignoreUnitSize) {
    aiSetImportPropertyInteger(
        propertyStore, AI_CONFIG_IMPORT_COLLADA_IGNORE_UNIT_SIZE, 1);
  }

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

TEST(MeshShapeTest, CustomMeshShapeOwnsManuallyAllocatedScene)
{
  std::shared_ptr<dynamics::MeshShape> clone;
  {
    CustomMeshShape shape(Eigen::Vector3d(2, 3, 4));
    EXPECT_EQ(
        shape.getMeshOwnership(), dynamics::MeshShape::MeshOwnership::Imported);
    shape.reassignMesh();
    EXPECT_EQ(
        shape.getMeshOwnership(), dynamics::MeshShape::MeshOwnership::Imported);
    const auto mesh = shape.getTriMesh();
    ASSERT_NE(mesh, nullptr);
    ASSERT_EQ(mesh->getVertices().size(), 6u);
    ASSERT_EQ(mesh->getTriangles().size(), 2u);
    EXPECT_EQ(
        mesh->getTriangles()[1], math::TriMesh<double>::Triangle(3, 4, 5));
    EXPECT_TRUE(shape.getBoundingBox().computeFullExtents().isApprox(
        Eigen::Vector3d(4, 9, 4), 1e-12));
    EXPECT_EQ(shape.getMesh()->mNumMaterials, 0u);
    clone = std::dynamic_pointer_cast<dynamics::MeshShape>(shape.clone());
    ASSERT_NE(clone, nullptr);
    EXPECT_NE(clone->getMesh(), shape.getMesh());
  }
  EXPECT_TRUE(clone->getBoundingBox().computeFullExtents().isApprox(
      Eigen::Vector3d(4, 9, 4), 1e-12));
  EXPECT_EQ(clone->getTriMesh()->getTriangles().size(), 2u);
}

TEST(MeshShapeTest, ExplicitNoneOwnershipPreservesBorrowedScene)
{
  auto scene = createCustomMeshScene();
  {
    dynamics::MeshShape shape(
        Eigen::Vector3d::Ones(),
        scene.get(),
        "",
        nullptr,
        dynamics::MeshShape::MeshOwnership::None);
    EXPECT_EQ(shape.getMesh(), scene.get());
    ASSERT_NE(shape.getTriMesh(), nullptr);
    EXPECT_EQ(shape.getTriMesh()->getTriangles().size(), 2u);
  }
  EXPECT_EQ(scene->mNumMeshes, 2u);
  EXPECT_EQ(scene->mMeshes[0]->mFaces[0].mIndices[2], 2u);
}

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
  const std::string filePath = DART_DATA_LOCAL_PATH "test/tetra_cm.dae";
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
  const std::string otherUri = common::Uri::createFromPath(
                                   DART_DATA_LOCAL_PATH "test/tetra_cm_alt.dae")
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
  ASSERT_NE(arrow.getMesh(), nullptr);
  ASSERT_EQ(arrow.getMesh()->mNumMaterials, 1u);
  ASSERT_GT(arrow.getMesh()->mNumMeshes, 0u);
  EXPECT_EQ(arrow.getMesh()->mMeshes[0]->mMaterialIndex, 0u);

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
  const std::string filePath = DART_DATA_LOCAL_PATH "test/tetra_cm.dae";
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

  // The fixture spans 10 x 20 x 30 cm; the file declares 0.01 m per unit.
  EXPECT_NEAR(unitScale, 0.01, 1e-12);
  EXPECT_TRUE(
      extentsIgnoringUnits.isApprox(Eigen::Vector3d(10.0, 20.0, 30.0), 1e-6));
  EXPECT_TRUE(extentsWithUnits.isApprox(Eigen::Vector3d(0.1, 0.2, 0.3), 1e-6));
  EXPECT_TRUE(extentsWithUnits.isApprox(extentsIgnoringUnits * unitScale, 1e-6))
      << "extentsWithUnits=" << extentsWithUnits.transpose()
      << ", extentsIgnoringUnits=" << extentsIgnoringUnits.transpose()
      << ", unitScale=" << unitScale;
}

TEST(MeshShapeTest, ColladaUriWithoutExtensionStillLoads)
{
  const std::string filePath = DART_DATA_LOCAL_PATH "test/tetra_cm.dae";
  const std::string aliasUri = "collada-nodot://meshshape/tetra";

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

  EXPECT_TRUE(aliasExtents.isApprox(Eigen::Vector3d(0.1, 0.2, 0.3), 1e-6));
  EXPECT_TRUE(aliasExtents.isApprox(canonicalExtents, 1e-6))
      << "aliasExtents=" << aliasExtents.transpose()
      << ", canonicalExtents=" << canonicalExtents.transpose();
}

TEST(MeshShapeTest, PolygonMeshPreservesQuadFaces)
{
  const std::string filePath = DART_DATA_LOCAL_PATH "obj/Quad.obj";
  const std::string fileUri = common::Uri::createFromPath(filePath).toString();
  ASSERT_FALSE(fileUri.empty());

  auto retriever = std::make_shared<common::LocalResourceRetriever>();
  const aiScene* scene = dynamics::MeshShape::loadMesh(fileUri, retriever);
  ASSERT_NE(scene, nullptr);

  auto shape = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), scene, fileUri, retriever);

  const auto triMesh = shape->getTriMesh();
  ASSERT_NE(triMesh, nullptr);
  EXPECT_EQ(triMesh->getTriangles().size(), 2u);

  const auto polygonMesh = shape->getPolygonMesh();
  ASSERT_NE(polygonMesh, nullptr);
  ASSERT_TRUE(polygonMesh->hasFaces());
  ASSERT_EQ(polygonMesh->getFaces().size(), 1u);
  EXPECT_EQ(polygonMesh->getFaces()[0].size(), 4u);
}

DART_SUPPRESS_DEPRECATED_END
