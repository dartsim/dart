#include "dart/collision/CollisionOption.hpp"
#include "dart/collision/CollisionResult.hpp"
#include "dart/collision/fcl/FCLCollisionDetector.hpp"
#include "dart/common/Deprecated.hpp"
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/config.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/dynamics/ArrowShape.hpp"
#include "dart/dynamics/AssimpInputResourceAdaptor.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/simulation/World.hpp"

#include <assimp/cexport.h>
#include <assimp/cimport.h>
#include <assimp/config.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cmath>

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
  explicit CustomMeshShape(
      const Eigen::Vector3d& scale,
      std::unique_ptr<aiScene> scene = createCustomMeshScene())
    : MeshShape(scale, nullptr)
  {
    mMesh = scene.release();
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

// A unit cube as gz::common::MeshManager::CreateBox emits it: four vertices
// per face, each carrying the face normal, and two outward triangles per face.
struct BoxMeshData
{
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3d> normals;
  std::vector<std::array<unsigned int, 3>> triangles;
};

BoxMeshData createBoxMeshData()
{
  constexpr double corners[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
  BoxMeshData data;
  for (int axis = 0; axis < 3; ++axis) {
    for (const double sign : {-1.0, 1.0}) {
      const Eigen::Vector3d normal = sign * Eigen::Vector3d::Unit(axis);
      // (u, v, normal) is right-handed, so the corners wind counterclockwise
      // seen from outside.
      Eigen::Vector3d u = Eigen::Vector3d::Unit((axis + 1) % 3);
      Eigen::Vector3d v = Eigen::Vector3d::Unit((axis + 2) % 3);
      if (sign < 0)
        u.swap(v);

      const auto first = static_cast<unsigned int>(data.vertices.size());
      for (const auto& corner : corners) {
        data.vertices.push_back(0.5 * (normal + corner[0] * u + corner[1] * v));
        data.normals.push_back(normal);
      }
      data.triangles.push_back({first, first + 1, first + 2});
      data.triangles.push_back({first, first + 2, first + 3});
    }
  }
  return data;
}

// Fills the scene as gz-physics' CustomMeshShape does: one node listing the
// submesh, no materials, raw vertex and normal arrays, and one three-index face
// per triangle.
std::unique_ptr<aiScene> createCustomMeshScene(const BoxMeshData& data)
{
  auto scene = std::make_unique<aiScene>();
  scene->mRootNode = new aiNode;
  scene->mRootNode->mNumMeshes = 1;
  scene->mRootNode->mMeshes = new unsigned int[1]{0};
  scene->mNumMeshes = 1;
  scene->mMeshes = new aiMesh*[1];
  aiMesh* mesh = new aiMesh;
  scene->mMeshes[0] = mesh;
  mesh->mMaterialIndex = static_cast<unsigned int>(-1);
  mesh->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
  mesh->mNumVertices = static_cast<unsigned int>(data.vertices.size());
  mesh->mVertices = new aiVector3D[mesh->mNumVertices];
  mesh->mNormals = new aiVector3D[mesh->mNumVertices];
  for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
    for (int k = 0; k < 3; ++k) {
      mesh->mVertices[i][k] = static_cast<ai_real>(data.vertices[i][k]);
      mesh->mNormals[i][k] = static_cast<ai_real>(data.normals[i][k]);
    }
  }
  mesh->mNumFaces = static_cast<unsigned int>(data.triangles.size());
  mesh->mFaces = new aiFace[mesh->mNumFaces];
  for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
    mesh->mFaces[i].mNumIndices = 3;
    mesh->mFaces[i].mIndices = new unsigned int[3]{
        data.triangles[i][0], data.triangles[i][1], data.triangles[i][2]};
  }
  return scene;
}

std::shared_ptr<math::TriMesh<double>> createBoxTriMesh(const BoxMeshData& data)
{
  auto mesh = std::make_shared<math::TriMesh<double>>();
  mesh->reserveVertices(data.vertices.size());
  mesh->reserveVertexNormals(data.normals.size());
  mesh->reserveTriangles(data.triangles.size());
  for (const auto& vertex : data.vertices)
    mesh->addVertex(vertex);
  for (const auto& normal : data.normals)
    mesh->addVertexNormal(normal);
  for (const auto& triangle : data.triangles)
    mesh->addTriangle(triangle[0], triangle[1], triangle[2]);
  return mesh;
}

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

enum class MeshConstruction
{
  // gz-physics: MeshShape(scale, nullptr), then a raw aiScene assigned to the
  // protected mesh handle.
  GzAiScene,
  // The Assimp-free TriMesh constructor.
  TriMesh
};

std::shared_ptr<dynamics::MeshShape> createBoxMeshShape(
    MeshConstruction construction, const Eigen::Vector3d& scale)
{
  const BoxMeshData data = createBoxMeshData();
  if (construction == MeshConstruction::GzAiScene) {
    return std::make_shared<CustomMeshShape>(
        scale, createCustomMeshScene(data));
  }
  return std::make_shared<dynamics::MeshShape>(scale, createBoxTriMesh(data));
}

// A fixed skeleton holding the mesh and a free skeleton holding a box, in a
// world with the default collision detector and no gravity.
struct MeshBoxScene
{
  simulation::WorldPtr world;
  dynamics::SkeletonPtr meshSkeleton;
  dynamics::SkeletonPtr boxSkeleton;
  dynamics::ShapeNode* meshNode{nullptr};
  dynamics::ShapeNode* boxNode{nullptr};
  dynamics::FreeJoint* boxJoint{nullptr};

  void setBoxX(double x) const
  {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation().x() = x;
    boxJoint->setPositions(dynamics::FreeJoint::convertToPositions(transform));
  }
};

MeshBoxScene createMeshBoxScene(
    const dynamics::ShapePtr& meshShape, double boxX)
{
  MeshBoxScene scene;
  scene.world = simulation::World::create();
  scene.world->setGravity(Eigen::Vector3d::Zero());

  scene.meshSkeleton = dynamics::Skeleton::create("mesh");
  auto* meshBody
      = scene.meshSkeleton->createJointAndBodyNodePair<dynamics::WeldJoint>()
            .second;
  scene.meshNode = meshBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(meshShape, "mesh");
  scene.world->addSkeleton(scene.meshSkeleton);

  scene.boxSkeleton = dynamics::Skeleton::create("box");
  auto pair
      = scene.boxSkeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  scene.boxNode = pair.second->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(1.0, 0.5, 0.5)),
      "box");
  scene.boxJoint = pair.first;
  scene.setBoxX(boxX);
  scene.world->addSkeleton(scene.boxSkeleton);
  return scene;
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

  auto original = std::shared_ptr<dynamics::MeshShape>(new dynamics::MeshShape(
      Eigen::Vector3d::Ones(), scene, fileUri, retriever));
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

  auto mesh = std::shared_ptr<dynamics::MeshShape>(new dynamics::MeshShape(
      Eigen::Vector3d::Ones(), scene, fileUri, retriever));
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
  const auto shapeWithUnits = std::shared_ptr<dynamics::MeshShape>(
      new dynamics::MeshShape(Eigen::Vector3d::Ones(), sceneWithUnits));
  const Eigen::Vector3d extentsWithUnits
      = shapeWithUnits->getBoundingBox().computeFullExtents();

  const aiScene* sceneIgnoringUnits
      = loadMeshWithOverrides(fileUri, retriever, true);
  ASSERT_NE(sceneIgnoringUnits, nullptr);
  const auto shapeIgnoringUnits = std::shared_ptr<dynamics::MeshShape>(
      new dynamics::MeshShape(Eigen::Vector3d::Ones(), sceneIgnoringUnits));
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
  const auto aliasShape = std::shared_ptr<dynamics::MeshShape>(
      new dynamics::MeshShape(Eigen::Vector3d::Ones(), aliasScene));
  const Eigen::Vector3d aliasExtents
      = aliasShape->getBoundingBox().computeFullExtents();

  auto canonicalRetriever = std::make_shared<common::LocalResourceRetriever>();
  const aiScene* canonicalScene = dynamics::MeshShape::loadMesh(
      common::Uri::createFromPath(filePath).toString(), canonicalRetriever);
  ASSERT_NE(canonicalScene, nullptr);
  const auto canonicalShape = std::shared_ptr<dynamics::MeshShape>(
      new dynamics::MeshShape(Eigen::Vector3d::Ones(), canonicalScene));
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

  auto shape = std::shared_ptr<dynamics::MeshShape>(new dynamics::MeshShape(
      Eigen::Vector3d::Ones(), scene, fileUri, retriever));

  const auto triMesh = shape->getTriMesh();
  ASSERT_NE(triMesh, nullptr);
  EXPECT_EQ(triMesh->getTriangles().size(), 2u);

  const auto polygonMesh = shape->getPolygonMesh();
  ASSERT_NE(polygonMesh, nullptr);
  ASSERT_TRUE(polygonMesh->hasFaces());
  ASSERT_EQ(polygonMesh->getFaces().size(), 1u);
  EXPECT_EQ(polygonMesh->getFaces()[0].size(), 4u);
}

class MeshBoxCollisionTest : public ::testing::TestWithParam<MeshConstruction>
{
protected:
  // The scaled mesh spans [-1, 1] x [-0.5, 0.5] x [-0.5, 0.5].
  const Eigen::Vector3d scale{2.0, 1.0, 1.0};
};

TEST_P(MeshBoxCollisionTest, BoundingBoxFollowsScale)
{
  const auto shape = createBoxMeshShape(GetParam(), scale);
  const auto scene = createMeshBoxScene(shape, 5.0);
  EXPECT_EQ(scene.meshNode->getShape(), shape);

  // The queries gz-physics makes on an attached mesh: its local bounding box,
  // the size from that box, and the scale.
  const auto& box = scene.meshNode->getShape()->getBoundingBox();
  EXPECT_TRUE(box.getMin().isApprox(Eigen::Vector3d(-1.0, -0.5, -0.5), 1e-12));
  EXPECT_TRUE(box.getMax().isApprox(Eigen::Vector3d(1.0, 0.5, 0.5), 1e-12));
  EXPECT_TRUE(box.computeFullExtents().isApprox(scale, 1e-12));
  EXPECT_TRUE(shape->getScale().isApprox(scale, 1e-12));
  EXPECT_NEAR(shape->getVolume(), 2.0, 1e-12);

  // Either construction also offers the cube as an aiScene.
  const aiScene* legacy = shape->getMesh();
  ASSERT_NE(legacy, nullptr);
  unsigned int faces = 0;
  for (unsigned int i = 0; i < legacy->mNumMeshes; ++i)
    faces += legacy->mMeshes[i]->mNumFaces;
  EXPECT_EQ(faces, 12u);
  EXPECT_EQ(shape->getTriMesh()->getTriangles().size(), 12u);

  shape->setScale(Eigen::Vector3d(1.0, 2.0, 4.0));
  EXPECT_TRUE(shape->getBoundingBox().getMin().isApprox(
      Eigen::Vector3d(-0.5, -1.0, -2.0), 1e-12));
  EXPECT_TRUE(shape->getBoundingBox().getMax().isApprox(
      Eigen::Vector3d(0.5, 1.0, 2.0), 1e-12));
  EXPECT_NEAR(shape->getVolume(), 8.0, 1e-12);
}

TEST_P(MeshBoxCollisionTest, CollidesWithBoxUnderDefaultDetector)
{
  const auto shape = createBoxMeshShape(GetParam(), scale);
  const auto scene = createMeshBoxScene(shape, 1.4);
  EXPECT_NE(
      std::dynamic_pointer_cast<collision::FCLCollisionDetector>(
          scene.world->getConstraintSolver()->getCollisionDetector()),
      nullptr);

  // The box spans x in [0.9, 1.9] and pokes 0.1 into the mesh's +x face.
  collision::CollisionResult result;
  const collision::CollisionOption option(true, 100u);
  EXPECT_TRUE(scene.world->checkCollision(option, &result));
  ASSERT_GT(result.getNumContacts(), 0u);
  for (const auto& contact : result.getContacts()) {
    const bool boxFirst = contact.getShapeNode1() == scene.boxNode;
    EXPECT_EQ(
        contact.getShapeNode1(), boxFirst ? scene.boxNode : scene.meshNode);
    EXPECT_EQ(
        contact.getShapeNode2(), boxFirst ? scene.meshNode : scene.boxNode);
    // The normal points from the second object to the first.
    EXPECT_NEAR(contact.normal.x(), boxFirst ? 1.0 : -1.0, 1e-3);
    EXPECT_NEAR(contact.normal.y(), 0.0, 1e-3);
    EXPECT_NEAR(contact.normal.z(), 0.0, 1e-3);
    EXPECT_NEAR(contact.penetrationDepth, 0.1, 1e-3);
    EXPECT_GE(contact.point.x(), 0.9 - 1e-3);
    EXPECT_LE(contact.point.x(), 1.0 + 1e-3);
    EXPECT_LE(std::abs(contact.point.y()), 0.25 + 1e-3);
    EXPECT_LE(std::abs(contact.point.z()), 0.25 + 1e-3);
  }

  // A gap of 0.1 between the box and the +x face.
  scene.setBoxX(1.6);
  result.clear();
  EXPECT_FALSE(scene.world->checkCollision(option, &result));
  EXPECT_EQ(result.getNumContacts(), 0u);

  // Scaling the mesh to x in [-0.5, 0.5] opens a gap at the first pose.
  scene.setBoxX(1.4);
  EXPECT_TRUE(scene.world->checkCollision());
  shape->setScale(Eigen::Vector3d::Ones());
  EXPECT_FALSE(scene.world->checkCollision());
}

TEST_P(MeshBoxCollisionTest, SteppingPushesTheBoxOutOfTheMesh)
{
  const auto scene
      = createMeshBoxScene(createBoxMeshShape(GetParam(), scale), 1.4);
  scene.world->step();
  EXPECT_GT(scene.world->getLastCollisionResult().getNumContacts(), 0u);
  EXPECT_GT(scene.boxNode->getBodyNodePtr()->getLinearVelocity().x(), 0.0);
}

TEST_P(MeshBoxCollisionTest, ClonedSkeletonsKeepGeometryAndContacts)
{
  const auto shape = createBoxMeshShape(GetParam(), scale);
  // A TriMesh shape clones its cached aiScene view as well.
  ASSERT_NE(shape->getMesh(), nullptr);
  const auto scene = createMeshBoxScene(shape, 1.4);
  const auto meshClone = scene.meshSkeleton->cloneSkeleton("mesh_clone");
  const auto boxClone = scene.boxSkeleton->cloneSkeleton("box_clone");

  const auto* cloneNode = meshClone->getBodyNode(0)->getShapeNode(0);
  ASSERT_NE(cloneNode, nullptr);
  EXPECT_NE(cloneNode->getShape(), scene.meshNode->getShape());
  const auto& original = scene.meshNode->getShape()->getBoundingBox();
  EXPECT_TRUE(cloneNode->getShape()->getBoundingBox().getMin().isApprox(
      original.getMin(), 1e-12));
  EXPECT_TRUE(cloneNode->getShape()->getBoundingBox().getMax().isApprox(
      original.getMax(), 1e-12));

  const auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->addSkeleton(meshClone);
  world->addSkeleton(boxClone);
  collision::CollisionResult result;
  EXPECT_TRUE(
      world->checkCollision(collision::CollisionOption(true, 100u), &result));
  EXPECT_GT(result.getNumContacts(), 0u);
}

INSTANTIATE_TEST_SUITE_P(
    GzAiSceneAndTriMesh,
    MeshBoxCollisionTest,
    ::testing::Values(MeshConstruction::GzAiScene, MeshConstruction::TriMesh),
    [](const ::testing::TestParamInfo<MeshConstruction>& info) {
      return info.param == MeshConstruction::GzAiScene ? "GzAiScene"
                                                       : "TriMesh";
    });

using MeshOwnership = dynamics::MeshShape::MeshOwnership;

// A TriMesh shape builds an aiScene view on first use and owns it; replacing
// the mesh must release that view once, not again when the shape dies.
TEST(MeshShapeTest, ReplacingTheCachedAiSceneReleasesItOnce)
{
  const BoxMeshData data = createBoxMeshData();
  for (const bool shared : {false, true}) {
    dynamics::MeshShape shape(Eigen::Vector3d::Ones(), createBoxTriMesh(data));
    ASSERT_NE(shape.getMesh(), nullptr);

    if (shared) {
      shape.setMesh(
          std::shared_ptr<const aiScene>(createCustomMeshScene(data)));
    } else {
      shape.setMesh(
          createCustomMeshScene(data).release(),
          MeshOwnership::Manual,
          common::Uri());
    }
    ASSERT_NE(shape.getMesh(), nullptr);
    EXPECT_EQ(shape.getTriMesh()->getTriangles().size(), 12u);
  }
}

class MeshOwnershipTest : public ::testing::TestWithParam<MeshOwnership>
{
};

// Run under AddressSanitizer with detect_leaks=1: a scene the shape should
// release but does not shows up as a leak, one it releases twice or releases
// although the caller keeps it as a use-after-free or double free.
TEST_P(MeshOwnershipTest, ShapeReleasesTheSceneExactlyWhenItOwnsIt)
{
  const MeshOwnership ownership = GetParam();
  const BoxMeshData data = createBoxMeshData();

  int customReleases = 0;
  std::shared_ptr<const aiScene> customScene;
  const aiScene* scene = nullptr;
  switch (ownership) {
    case MeshOwnership::Imported: {
      const std::string uri = common::Uri::createFromPath(DART_DATA_LOCAL_PATH
                                                          "test/tetra_cm.dae")
                                  .toString();
      scene = dynamics::MeshShape::loadMesh(
          uri, std::make_shared<common::LocalResourceRetriever>());
      break;
    }
    case MeshOwnership::Copied: {
      aiScene* copy = nullptr;
      aiCopyScene(createCustomMeshScene(data).get(), &copy);
      scene = copy;
      break;
    }
    case MeshOwnership::Custom:
      customScene = std::shared_ptr<const aiScene>(
          createCustomMeshScene(data).release(),
          [&customReleases](const aiScene* released) {
            ++customReleases;
            delete released;
          });
      scene = customScene.get();
      break;
    case MeshOwnership::Manual:
    case MeshOwnership::None:
      scene = createCustomMeshScene(data).release();
      break;
  }
  ASSERT_NE(scene, nullptr);

  std::shared_ptr<dynamics::MeshShape> clone;
  {
    const auto shape
        = ownership == MeshOwnership::Custom
              ? std::make_shared<dynamics::MeshShape>(
                  Eigen::Vector3d::Ones(), std::move(customScene))
              : std::make_shared<dynamics::MeshShape>(
                  Eigen::Vector3d::Ones(), scene, "", nullptr, ownership);
    EXPECT_EQ(shape->getMesh(), scene);
    ASSERT_NE(shape->getTriMesh(), nullptr);
    EXPECT_GT(shape->getTriMesh()->getTriangles().size(), 0u);

    // The clone owns a copy of the scene and outlives the original.
    clone = std::dynamic_pointer_cast<dynamics::MeshShape>(shape->clone());
    ASSERT_NE(clone, nullptr);
    EXPECT_NE(clone->getMesh(), scene);
  }
  EXPECT_GT(clone->getTriMesh()->getTriangles().size(), 0u);
  EXPECT_NE(clone->getMesh(), nullptr);

  if (ownership == MeshOwnership::Custom) {
    EXPECT_EQ(customReleases, 1);
  }

  // The shape is gone; a scene it never owned must still be intact.
  if (ownership == MeshOwnership::None) {
    ASSERT_GT(scene->mNumMeshes, 0u);
    EXPECT_EQ(scene->mMeshes[0]->mNumFaces, 12u);
    delete scene;
  }
}

INSTANTIATE_TEST_SUITE_P(
    AllOwnershipModes,
    MeshOwnershipTest,
    ::testing::Values(
        MeshOwnership::None,
        MeshOwnership::Imported,
        MeshOwnership::Copied,
        MeshOwnership::Manual,
        MeshOwnership::Custom),
    [](const ::testing::TestParamInfo<MeshOwnership>& info) {
      switch (info.param) {
        case MeshOwnership::None:
          return "None";
        case MeshOwnership::Imported:
          return "Imported";
        case MeshOwnership::Copied:
          return "Copied";
        case MeshOwnership::Manual:
          return "Manual";
        case MeshOwnership::Custom:
          return "Custom";
      }
      return "Unknown";
    });

DART_SUPPRESS_DEPRECATED_END
