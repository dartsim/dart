#include "dart/common/diagnostics.hpp"
#include "dart/common/filesystem.hpp"
#include "dart/common/local_resource_retriever.hpp"
#include "dart/common/uri.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/arrow_shape.hpp"
#include "dart/dynamics/assimp_input_resource_adaptor.hpp"
#include "dart/dynamics/mesh_shape.hpp"
#include "dart/math/tri_mesh.hpp"
#include "dart/utils/mesh_loader.hpp"

#include <Eigen/Core>
#include <assimp/cimport.h>
#include <assimp/config.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>
#include <set>
#include <sstream>
#include <string>

#include <cmath>

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
    if (isAlias(uri)) {
      return mDelegate->exists(mTargetUri);
    }
    return mDelegate->exists(uri);
  }

  common::ResourcePtr retrieve(const common::Uri& uri) override
  {
    if (isAlias(uri)) {
      return mDelegate->retrieve(mTargetUri);
    }
    return mDelegate->retrieve(uri);
  }

  std::string getFilePath(const common::Uri& uri) override
  {
    DART_SUPPRESS_DEPRECATED_BEGIN
    if (isAlias(uri)) {
      return mDelegate->getFilePath(mTargetUri);
    }
    return mDelegate->getFilePath(uri);
    DART_SUPPRESS_DEPRECATED_END
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

DART_SUPPRESS_DEPRECATED_BEGIN
class DirectAssignMeshShape final : public dynamics::MeshShape
{
public:
  DirectAssignMeshShape()
    : dynamics::MeshShape(Eigen::Vector3d::Ones(), nullptr)
  {
    auto* scene = new aiScene();
    scene->mNumMeshes = 2;
    scene->mMeshes = new aiMesh*[scene->mNumMeshes];

    scene->mRootNode = new aiNode();
    scene->mRootNode->mNumMeshes = scene->mNumMeshes;
    scene->mRootNode->mMeshes = new unsigned int[scene->mNumMeshes];
    scene->mRootNode->mMeshes[0] = 0u;
    scene->mRootNode->mMeshes[1] = 1u;

    for (unsigned int meshIndex = 0; meshIndex < scene->mNumMeshes;
         ++meshIndex) {
      auto* mesh = new aiMesh();

      mesh->mNumVertices = 3;
      mesh->mVertices = new aiVector3D[mesh->mNumVertices];
      mesh->mNormals = new aiVector3D[mesh->mNumVertices];

      const ai_real z = (meshIndex == 0 ? 0.0f : 1.0f);

      mesh->mVertices[0] = aiVector3D(0.0f, 0.0f, z);
      mesh->mVertices[1] = aiVector3D(1.0f, 0.0f, z);
      mesh->mVertices[2] = aiVector3D(0.0f, 1.0f, z);

      mesh->mNormals[0] = aiVector3D(0.0f, 0.0f, 1.0f);
      mesh->mNormals[1] = aiVector3D(0.0f, 0.0f, 1.0f);
      mesh->mNormals[2] = aiVector3D(0.0f, 0.0f, 1.0f);

      mesh->mNumFaces = 1;
      mesh->mFaces = new aiFace[mesh->mNumFaces];
      mesh->mFaces[0].mNumIndices = 3;
      mesh->mFaces[0].mIndices = new unsigned int[3];
      mesh->mFaces[0].mIndices[0] = 0u;
      mesh->mFaces[0].mIndices[1] = 1u;
      mesh->mFaces[0].mIndices[2] = 2u;

      mesh->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;

      scene->mMeshes[meshIndex] = mesh;
    }

    mRawScene = scene;
    this->mMesh = scene;
    this->mIsBoundingBoxDirty = true;
    this->mIsVolumeDirty = true;
  }

  const aiScene* rawScene() const
  {
    return mRawScene;
  }

  MeshOwnership meshOwnership() const
  {
    return this->mMesh.getOwnership();
  }

private:
  const aiScene* mRawScene{nullptr};
};
DART_SUPPRESS_DEPRECATED_END

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
  if (!file.is_open()) {
    return 1.0;
  }

  const std::string token = "meter=\"";
  std::string buffer(
      (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  const std::size_t pos = buffer.find(token);
  if (pos == std::string::npos) {
    return 1.0;
  }

  const std::size_t start = pos + token.size();
  const std::size_t end = buffer.find('"', start);
  if (end == std::string::npos) {
    return 1.0;
  }

  try {
    return std::stod(buffer.substr(start, end - start));
  } catch (...) {
    return 1.0;
  }
}

std::shared_ptr<math::TriMesh<double>> createTetraMesh()
{
  auto mesh = std::make_shared<math::TriMesh<double>>();
  mesh->addVertex(0.0, 0.0, 0.0);
  mesh->addVertex(1.0, 0.0, 0.0);
  mesh->addVertex(0.0, 1.0, 0.0);
  mesh->addVertex(0.0, 0.0, 1.0);
  mesh->addTriangle(0, 1, 2);
  mesh->addTriangle(0, 1, 3);
  mesh->addTriangle(0, 2, 3);
  mesh->addTriangle(1, 2, 3);
  return mesh;
}

} // namespace

TEST(MeshShapeTest, CloneCreatesIndependentScene)
{
  const std::string filePath = dart::config::dataPath("skel/kima/l-foot.dae");
  const common::Uri fileUri = common::Uri::createFromPath(filePath);
  const std::string fileUriString = fileUri.toString();
  ASSERT_FALSE(fileUriString.empty());

  auto retriever = std::make_shared<common::LocalResourceRetriever>();
  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene
      = dynamics::MeshShape::loadMesh(fileUriString, retriever);
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(scene, nullptr);

  DART_SUPPRESS_DEPRECATED_BEGIN
  auto original = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), scene, fileUri, retriever);
  DART_SUPPRESS_DEPRECATED_END
  const Eigen::Vector3d originalExtents
      = original->getBoundingBox().computeFullExtents();

  auto cloned
      = std::dynamic_pointer_cast<dynamics::MeshShape>(original->clone());
  ASSERT_NE(cloned, nullptr);

  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_NE(original->getMesh(), cloned->getMesh());
  DART_SUPPRESS_DEPRECATED_END

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

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* sceneWithUnits
      = dynamics::MeshShape::loadMesh(fileUri, retriever);
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(sceneWithUnits, nullptr);
  DART_SUPPRESS_DEPRECATED_BEGIN
  const auto shapeWithUnits = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), sceneWithUnits);
  DART_SUPPRESS_DEPRECATED_END
  const Eigen::Vector3d extentsWithUnits
      = shapeWithUnits->getBoundingBox().computeFullExtents();

  const aiScene* sceneIgnoringUnits
      = loadMeshWithOverrides(fileUri, retriever, true);
  ASSERT_NE(sceneIgnoringUnits, nullptr);
  DART_SUPPRESS_DEPRECATED_BEGIN
  const auto shapeIgnoringUnits = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), sceneIgnoringUnits);
  DART_SUPPRESS_DEPRECATED_END
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
  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* aliasScene
      = dynamics::MeshShape::loadMesh(aliasUri, aliasRetriever);
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(aliasScene, nullptr);
  DART_SUPPRESS_DEPRECATED_BEGIN
  const auto aliasShape = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), aliasScene);
  DART_SUPPRESS_DEPRECATED_END
  const Eigen::Vector3d aliasExtents
      = aliasShape->getBoundingBox().computeFullExtents();

  auto canonicalRetriever = std::make_shared<common::LocalResourceRetriever>();
  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* canonicalScene = dynamics::MeshShape::loadMesh(
      common::Uri::createFromPath(filePath).toString(), canonicalRetriever);
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(canonicalScene, nullptr);
  DART_SUPPRESS_DEPRECATED_BEGIN
  const auto canonicalShape = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), canonicalScene);
  DART_SUPPRESS_DEPRECATED_END
  const Eigen::Vector3d canonicalExtents
      = canonicalShape->getBoundingBox().computeFullExtents();

  EXPECT_TRUE(aliasExtents.isApprox(canonicalExtents, 1e-6))
      << "aliasExtents=" << aliasExtents.transpose()
      << ", canonicalExtents=" << canonicalExtents.transpose();
}

TEST(MeshShapeTest, PolygonMeshPreservesQuadFaces)
{
  const std::string filePath = dart::config::dataPath("obj/Quad.obj");
  const common::Uri fileUri = common::Uri::createFromPath(filePath);
  const std::string fileUriString = fileUri.toString();
  ASSERT_FALSE(fileUriString.empty());

  auto retriever = std::make_shared<common::LocalResourceRetriever>();
  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene
      = dynamics::MeshShape::loadMesh(fileUriString, retriever);
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(scene, nullptr);

  DART_SUPPRESS_DEPRECATED_BEGIN
  auto shape = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), scene, fileUri, retriever);
  DART_SUPPRESS_DEPRECATED_END

  const auto polygonMesh = shape->getPolygonMesh();
  ASSERT_NE(polygonMesh, nullptr);
  ASSERT_TRUE(polygonMesh->hasFaces());
  ASSERT_EQ(polygonMesh->getFaces().size(), 1u);
  EXPECT_EQ(polygonMesh->getFaces()[0].size(), 4u);
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

    DART_SUPPRESS_DEPRECATED_BEGIN
    dynamics::MeshShape shape(Eigen::Vector3d::Ones(), scene);
    DART_SUPPRESS_DEPRECATED_END

    DART_SUPPRESS_DEPRECATED_BEGIN
    EXPECT_EQ(shape.getMesh(), scene.get());
    DART_SUPPRESS_DEPRECATED_END
  }

  EXPECT_EQ(deleted.load(), 1);
}

TEST(MeshShapeTest, DirectAssignSceneBuildsTriMeshAndUsesDeleteSemantics)
{
  DirectAssignMeshShape shape;
  EXPECT_EQ(shape.meshOwnership(), dynamics::MeshShape::MeshOwnership::Manual);

  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_EQ(shape.getMesh(), shape.rawScene());
  DART_SUPPRESS_DEPRECATED_END

  const auto triMesh = shape.getTriMesh();
  ASSERT_NE(triMesh, nullptr);
  EXPECT_EQ(triMesh->getVertices().size(), 6u);
  EXPECT_EQ(triMesh->getTriangles().size(), 2u);
  EXPECT_TRUE(triMesh->hasVertexNormals());

  const Eigen::Vector3d extents = shape.getBoundingBox().computeFullExtents();
  EXPECT_NEAR(extents.x(), 1.0, 1e-12);
  EXPECT_NEAR(extents.y(), 1.0, 1e-12);
  EXPECT_NEAR(extents.z(), 1.0, 1e-12);
}

TEST(MeshShapeTest, TracksOwnershipAndUriMetadata)
{
  auto retriever = std::make_shared<RecordingRetriever>();
  const common::Uri fileUri
      = common::Uri::createFromStringOrPath("/tmp/manual-mesh.dae");

  auto* manualScene = new aiScene;

  DART_SUPPRESS_DEPRECATED_BEGIN
  dynamics::MeshShape shape(
      Eigen::Vector3d::Ones(),
      manualScene,
      fileUri,
      retriever,
      dynamics::MeshShape::MeshOwnership::Manual);
  DART_SUPPRESS_DEPRECATED_END

  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_EQ(shape.getMesh(), manualScene);
  DART_SUPPRESS_DEPRECATED_END
  EXPECT_EQ(shape.getMeshPath(), fileUri.getFilesystemPath());
  EXPECT_EQ(shape.getMeshUri(), fileUri.toString());

  const common::Uri retrieverUri("package://example/mesh.dae");
  auto* retrieverScene = new aiScene;
  DART_SUPPRESS_DEPRECATED_BEGIN
  shape.setMesh(
      retrieverScene,
      dynamics::MeshShape::MeshOwnership::Manual,
      retrieverUri,
      retriever);
  DART_SUPPRESS_DEPRECATED_END
  EXPECT_EQ(retriever->lastUri, retrieverUri.toString());
  EXPECT_EQ(shape.getMeshPath(), "/virtual/path/from/retriever");

  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_EQ(shape.getMesh(), retrieverScene);
  DART_SUPPRESS_DEPRECATED_END

  // No-op when the mesh pointer and ownership are unchanged.

  DART_SUPPRESS_DEPRECATED_BEGIN
  shape.setMesh(
      shape.getMesh(),
      dynamics::MeshShape::MeshOwnership::Manual,
      retrieverUri,
      retriever);
  DART_SUPPRESS_DEPRECATED_END

  // Clearing the mesh resets related metadata.
  DART_SUPPRESS_DEPRECATED_BEGIN
  shape.setMesh(
      nullptr,
      dynamics::MeshShape::MeshOwnership::Manual,
      common::Uri(),
      nullptr);
  DART_SUPPRESS_DEPRECATED_END

  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_EQ(shape.getMesh(), nullptr);
  DART_SUPPRESS_DEPRECATED_END
  EXPECT_TRUE(shape.getMeshPath().empty());
  EXPECT_TRUE(shape.getMeshUri().empty());
}

TEST(MeshShapeTest, TriMeshConstructorTracksUriMetadata)
{
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);

  const std::string filePath = dart::config::dataPath("skel/kima/l-foot.dae");
  const common::Uri fileUri = common::Uri::createFromPath(filePath);

  dynamics::MeshShape shape(Eigen::Vector3d::Ones(), triMesh, fileUri);

  EXPECT_EQ(shape.getMeshUri(), fileUri.toString());
  EXPECT_EQ(shape.getMeshPath(), fileUri.getFilesystemPath());
}

TEST(MeshShapeTest, TriMeshConstructorPreservesMaterialsFromUri)
{
  const std::string filePath = dart::config::dataPath("skel/kima/l-foot.dae");
  const std::string aliasUri = "collada-nodot://meshshape/lfoot";

  auto aliasRetriever
      = std::make_shared<AliasUriResourceRetriever>(aliasUri, filePath);
  auto loader = std::make_unique<utils::MeshLoaderd>();
  auto triMesh = loader->load(aliasUri, aliasRetriever);
  ASSERT_NE(triMesh, nullptr);

  dynamics::MeshShape shape(
      Eigen::Vector3d::Ones(),
      std::move(triMesh),
      common::Uri(aliasUri),
      aliasRetriever);

  EXPECT_EQ(shape.getMeshUri(), aliasUri);
  EXPECT_EQ(shape.getMeshPath(), filePath);
  EXPECT_EQ(shape.getResourceRetriever(), aliasRetriever);
  EXPECT_FALSE(shape.getMaterials().empty());
}

TEST(MeshShapeTest, TriMeshGetMeshCachesImportedScene)
{
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addVertex(1.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);
  triMesh->addTriangle(1, 3, 2);

  dynamics::MeshShape shape(Eigen::Vector3d::Ones(), triMesh, common::Uri());

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* sceneFirst = shape.getMesh();
  ASSERT_NE(sceneFirst, nullptr);
  const aiScene* sceneSecond = shape.getMesh();
  DART_SUPPRESS_DEPRECATED_END

  EXPECT_EQ(sceneFirst, sceneSecond);
  ASSERT_GE(sceneFirst->mNumMeshes, 1u);
  ASSERT_NE(sceneFirst->mMeshes, nullptr);
  ASSERT_NE(sceneFirst->mMeshes[0], nullptr);
  EXPECT_EQ(sceneFirst->mMeshes[0]->mNumVertices, 4u);
  EXPECT_EQ(sceneFirst->mMeshes[0]->mNumFaces, 2u);
}

TEST(MeshShapeTest, TriMeshGetMeshPreservesTextureCoords)
{
  const auto tempDir = common::filesystem::temp_directory_path();
  const auto timestamp
      = std::chrono::high_resolution_clock::now().time_since_epoch().count();
  const auto baseName
      = std::string("dart_meshshape_texcoords_") + std::to_string(timestamp);
  const auto objPath = tempDir / (baseName + ".obj");

  {
    std::ofstream objFile(objPath.string());
    ASSERT_TRUE(objFile);
    objFile << "v 0 0 0\n"
            << "v 1 0 0\n"
            << "v 0 1 0\n"
            << "vt 0 0\n"
            << "vt 1 0\n"
            << "vt 0 1\n"
            << "f 1/1 2/2 3/3\n";
  }

  auto retriever = std::make_shared<common::LocalResourceRetriever>();
  auto loader = std::make_unique<utils::MeshLoaderd>();
  auto triMesh = loader->load(objPath.string(), retriever);
  ASSERT_NE(triMesh, nullptr);

  dynamics::MeshShape shape(
      Eigen::Vector3d::Ones(),
      std::move(triMesh),
      common::Uri::createFromPath(objPath.string()),
      retriever);

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene = shape.getMesh();
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(scene, nullptr);
  ASSERT_GE(scene->mNumMeshes, 1u);
  const aiMesh* mesh = scene->mMeshes[0];
  ASSERT_NE(mesh, nullptr);
  ASSERT_TRUE(mesh->HasTextureCoords(0));
  ASSERT_NE(mesh->mTextureCoords[0], nullptr);
  EXPECT_EQ(mesh->mNumVertices, 3u);

  auto hasTexCoord = [mesh](float u, float v) {
    for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
      const auto& texCoord = mesh->mTextureCoords[0][i];
      if (std::fabs(texCoord.x - u) < 1e-6f
          && std::fabs(texCoord.y - v) < 1e-6f) {
        return true;
      }
    }
    return false;
  };

  EXPECT_TRUE(hasTexCoord(0.0f, 0.0f));
  EXPECT_TRUE(hasTexCoord(1.0f, 0.0f));
  EXPECT_TRUE(hasTexCoord(0.0f, 1.0f));

  common::error_code ec;
  common::filesystem::remove(objPath, ec);
}

TEST(MeshShapeTest, TriMeshGetMeshPreservesMaterialIndices)
{
  const auto tempDir = common::filesystem::temp_directory_path();
  const auto timestamp
      = std::chrono::high_resolution_clock::now().time_since_epoch().count();
  const auto baseName
      = std::string("dart_meshshape_materials_") + std::to_string(timestamp);
  const auto objPath = tempDir / (baseName + ".obj");
  const auto mtlPath = tempDir / (baseName + ".mtl");

  {
    std::ofstream mtlFile(mtlPath.string());
    ASSERT_TRUE(mtlFile);
    mtlFile << "newmtl material_0\n"
            << "Ka 0 0 0\n"
            << "Kd 1 0 0\n"
            << "Ks 0 0 0\n"
            << "Ns 0\n"
            << "newmtl material_1\n"
            << "Ka 0 0 0\n"
            << "Kd 0 1 0\n"
            << "Ks 0 0 0\n"
            << "Ns 0\n";
  }

  {
    std::ofstream objFile(objPath.string());
    ASSERT_TRUE(objFile);
    objFile << "mtllib " << mtlPath.filename().string() << "\n"
            << "v 0 0 0\n"
            << "v 1 0 0\n"
            << "v 0 1 0\n"
            << "v 1 1 0\n"
            << "usemtl material_0\n"
            << "f 1 2 3\n"
            << "usemtl material_1\n"
            << "f 2 4 3\n";
  }

  auto retriever = std::make_shared<common::LocalResourceRetriever>();
  auto loader = std::make_unique<utils::MeshLoaderd>();
  auto triMesh = loader->load(objPath.string(), retriever);
  ASSERT_NE(triMesh, nullptr);

  dynamics::MeshShape shape(
      Eigen::Vector3d::Ones(),
      std::move(triMesh),
      common::Uri::createFromPath(objPath.string()),
      retriever);

  EXPECT_GE(shape.getMaterials().size(), 2u);

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene = shape.getMesh();
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(scene, nullptr);

  ASSERT_GE(scene->mNumMaterials, 2u);

  std::set<unsigned int> usedMaterialIndices;
  std::set<std::string> usedMaterialNames;
  for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
    const aiMesh* mesh = scene->mMeshes[i];
    ASSERT_NE(mesh, nullptr);
    ASSERT_LT(mesh->mMaterialIndex, scene->mNumMaterials);
    usedMaterialIndices.insert(mesh->mMaterialIndex);

    aiString materialName;
    if (aiGetMaterialString(
            scene->mMaterials[mesh->mMaterialIndex],
            AI_MATKEY_NAME,
            &materialName)
        == aiReturn_SUCCESS) {
      usedMaterialNames.insert(materialName.C_Str());
    }
  }

  std::ostringstream materialSummary;
  materialSummary << "indices=";
  bool first = true;
  for (const auto& index : usedMaterialIndices) {
    if (!first) {
      materialSummary << ",";
    }
    materialSummary << index;
    first = false;
  }
  materialSummary << " names=";
  first = true;
  for (const auto& name : usedMaterialNames) {
    if (!first) {
      materialSummary << ",";
    }
    materialSummary << name;
    first = false;
  }
  EXPECT_GE(usedMaterialIndices.size(), 2u) << materialSummary.str();
  EXPECT_GE(usedMaterialNames.size(), 2u) << materialSummary.str();

  std::set<std::string> expectedMaterialNames;
  for (std::size_t index = 0; index < shape.getMaterials().size(); ++index) {
    expectedMaterialNames.insert("material_" + std::to_string(index));
  }
  for (const auto& name : usedMaterialNames) {
    EXPECT_TRUE(expectedMaterialNames.count(name)) << materialSummary.str();
  }

  common::error_code ec;
  common::filesystem::remove(objPath, ec);
  common::filesystem::remove(mtlPath, ec);
}

TEST(MeshShapeTest, SetMeshAdoptsCachedSceneWithoutUseAfterFree)
{
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);

  dynamics::MeshShape shape(Eigen::Vector3d::Ones(), triMesh, common::Uri());

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* cachedScene = shape.getMesh();
  ASSERT_NE(cachedScene, nullptr);

  // Calling setMesh() with the cached aiScene pointer should not release the
  // scene before adopting it (regression test for use-after-free).
  shape.setMesh(
      cachedScene, common::Uri("package://example/mesh.obj"), nullptr);

  const aiScene* adoptedScene = shape.getMesh();
  DART_SUPPRESS_DEPRECATED_END

  EXPECT_EQ(adoptedScene, cachedScene);

  const auto updatedTriMesh = shape.getTriMesh();
  ASSERT_NE(updatedTriMesh, nullptr);
  EXPECT_TRUE(updatedTriMesh->hasTriangles());
  EXPECT_EQ(updatedTriMesh->getVertices().size(), 3u);
  EXPECT_EQ(updatedTriMesh->getTriangles().size(), 1u);
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

  DART_SUPPRESS_DEPRECATED_BEGIN
  ASSERT_NE(cloned->getMesh(), nullptr);
  EXPECT_NE(cloned->getMesh(), arrow.getMesh());
  DART_SUPPRESS_DEPRECATED_END
}

TEST(ArrowShapeTest, MaterialCountIsInitialized)
{
  dynamics::ArrowShape arrow(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitX(),
      dynamics::ArrowShape::Properties(),
      Eigen::Vector4d::Ones(),
      4);

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene = arrow.getMesh();
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(scene, nullptr);
  EXPECT_EQ(scene->mNumMaterials, 1u);
  ASSERT_NE(scene->mMaterials, nullptr);
  EXPECT_NE(scene->mMaterials[0], nullptr);
}

TEST(MeshShapeTest, ScaleColorAndInertia)
{
  auto mesh = createTetraMesh();
  dynamics::MeshShape shape(Eigen::Vector3d(1.0, 2.0, 3.0), mesh);

  EXPECT_TRUE(shape.getScale().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

  shape.setScale(2.0);
  EXPECT_TRUE(shape.getScale().isApprox(Eigen::Vector3d::Constant(2.0)));

  shape.setScale(Eigen::Vector3d(1.0, 0.5, 2.0));
  EXPECT_TRUE(shape.getScale().isApprox(Eigen::Vector3d(1.0, 0.5, 2.0)));

  shape.setColorMode(dynamics::MeshShape::COLOR_INDEX);
  shape.setColorIndex(1);
  shape.setAlphaMode(dynamics::MeshShape::SHAPE_ALPHA);
  EXPECT_EQ(shape.getColorMode(), dynamics::MeshShape::COLOR_INDEX);
  EXPECT_EQ(shape.getColorIndex(), 1);
  EXPECT_EQ(shape.getAlphaMode(), dynamics::MeshShape::SHAPE_ALPHA);

  shape.setScale(Eigen::Vector3d::Ones());
  Eigen::Matrix3d inertia = shape.computeInertia(5.0);
  EXPECT_TRUE(inertia.array().isFinite().all());
  EXPECT_NEAR((inertia - inertia.transpose()).norm(), 0.0, 1e-12);

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* sceneA = shape.getMesh();
  const aiScene* sceneB = shape.getMesh();
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(sceneA, nullptr);
  EXPECT_EQ(sceneA, sceneB);
}

TEST(MeshShapeTest, CollectSubMeshRangesRespectsExpectedCounts)
{
  DirectAssignMeshShape shape;

  class MeshShapeHarness final : public dynamics::MeshShape
  {
  public:
    using dynamics::MeshShape::collectSubMeshRanges;
    using dynamics::MeshShape::SubMeshRange;
  };

  std::vector<MeshShapeHarness::SubMeshRange> ranges;
  const aiScene* scene = shape.rawScene();
  ASSERT_TRUE(MeshShapeHarness::collectSubMeshRanges(scene, ranges, 6u, 2u));
  ASSERT_EQ(ranges.size(), 2u);
  EXPECT_EQ(ranges[0].vertexOffset, 0u);
  EXPECT_EQ(ranges[0].vertexCount, 3u);
  EXPECT_EQ(ranges[0].triangleCount, 1u);
  EXPECT_EQ(ranges[1].vertexOffset, 3u);
  EXPECT_EQ(ranges[1].triangleOffset, 1u);

  std::vector<MeshShapeHarness::SubMeshRange> mismatch;
  EXPECT_FALSE(MeshShapeHarness::collectSubMeshRanges(scene, mismatch, 5u, 2u));
  EXPECT_TRUE(mismatch.empty());
}

TEST(MeshShapeTest, ConvertAssimpMeshPopulatesSubMeshes)
{
  DirectAssignMeshShape shape;

  class MeshShapeHarness final : public dynamics::MeshShape
  {
  public:
    using dynamics::MeshShape::convertAssimpMesh;
    using dynamics::MeshShape::SubMeshRange;
  };

  std::vector<MeshShapeHarness::SubMeshRange> ranges;
  auto triMesh = MeshShapeHarness::convertAssimpMesh(shape.rawScene(), &ranges);
  ASSERT_NE(triMesh, nullptr);
  EXPECT_EQ(triMesh->getVertices().size(), 6u);
  EXPECT_EQ(triMesh->getTriangles().size(), 2u);

  ASSERT_EQ(ranges.size(), 2u);
  EXPECT_EQ(ranges[0].triangleCount, 1u);
  EXPECT_EQ(ranges[1].triangleOffset, 1u);
}

TEST(MeshShapeTest, ConvertToAssimpMeshUsesEmbeddedMaterials)
{
  const auto tempDir = common::filesystem::temp_directory_path();
  const auto timestamp
      = std::chrono::high_resolution_clock::now().time_since_epoch().count();
  const auto baseName
      = std::string("dart_meshshape_embedded_") + std::to_string(timestamp);
  const auto objPath = tempDir / (baseName + ".obj");
  const auto mtlPath = tempDir / (baseName + ".mtl");

  {
    std::ofstream mtlFile(mtlPath.string());
    ASSERT_TRUE(mtlFile);
    mtlFile << "newmtl material_0\n"
            << "Kd 1 0 0\n"
            << "newmtl material_1\n"
            << "Kd 0 1 0\n";
  }

  {
    std::ofstream objFile(objPath.string());
    ASSERT_TRUE(objFile);
    objFile << "mtllib " << mtlPath.filename().string() << "\n"
            << "v 0 0 0\n"
            << "v 1 0 0\n"
            << "v 0 1 0\n"
            << "v 1 1 0\n"
            << "vt 0 0\n"
            << "vt 1 0\n"
            << "vt 0 1\n"
            << "vt 1 1\n"
            << "usemtl material_0\n"
            << "f 1/1 2/2 3/3\n"
            << "usemtl material_1\n"
            << "f 2/2 4/4 3/3\n";
  }

  auto retriever = std::make_shared<common::LocalResourceRetriever>();
  auto loader = std::make_unique<utils::MeshLoaderd>();
  auto triMesh = loader->load(objPath.string(), retriever);
  ASSERT_NE(triMesh, nullptr);

  class MeshShapeHarness final : public dynamics::MeshShape
  {
  public:
    using dynamics::MeshShape::MeshShape;
    const aiScene* callConvertToAssimpMesh() const
    {
      return convertToAssimpMesh();
    }
  };

  MeshShapeHarness shape(
      Eigen::Vector3d::Ones(),
      std::move(triMesh),
      common::Uri::createFromPath(objPath.string()),
      retriever);

  const aiScene* scene = shape.callConvertToAssimpMesh();
  ASSERT_NE(scene, nullptr);
  EXPECT_GE(scene->mNumMaterials, 2u);
  EXPECT_GE(scene->mNumMeshes, 1u);

  common::error_code ec;
  common::filesystem::remove(objPath, ec);
  common::filesystem::remove(mtlPath, ec);
}

TEST(MeshShapeTest, ConvertToAssimpMeshPreservesNormals)
{
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);
  triMesh->addVertexNormal(0.0, 0.0, 1.0);
  triMesh->addVertexNormal(0.0, 0.0, 1.0);
  triMesh->addVertexNormal(0.0, 0.0, 1.0);

  class MeshShapeHarness final : public dynamics::MeshShape
  {
  public:
    using dynamics::MeshShape::MeshShape;
    const aiScene* callConvertToAssimpMesh() const
    {
      return convertToAssimpMesh();
    }
  };

  MeshShapeHarness shape(Eigen::Vector3d::Ones(), triMesh, common::Uri());
  const aiScene* scene = shape.callConvertToAssimpMesh();
  ASSERT_NE(scene, nullptr);
  ASSERT_GE(scene->mNumMeshes, 1u);
  ASSERT_NE(scene->mMeshes[0], nullptr);
  EXPECT_TRUE(scene->mMeshes[0]->HasNormals());
}

TEST(MeshShapeTest, TriMeshPolygonMeshAndBoundingVolume)
{
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addVertex(0.0, 0.0, 1.0);
  triMesh->addTriangle(0, 1, 2);
  triMesh->addTriangle(0, 1, 3);

  const Eigen::Vector3d scale(2.0, 3.0, 4.0);
  dynamics::MeshShape shape(scale, triMesh, common::Uri());

  const auto polygonMesh = shape.getPolygonMesh();
  ASSERT_NE(polygonMesh, nullptr);
  EXPECT_TRUE(polygonMesh->hasFaces());
  EXPECT_EQ(polygonMesh->getFaces().size(), triMesh->getTriangles().size());

  const auto bbox = shape.getBoundingBox().computeFullExtents();
  EXPECT_TRUE(bbox.isApprox(scale, 1e-12));
  EXPECT_NEAR(shape.getVolume(), scale.x() * scale.y() * scale.z(), 1e-12);

  EXPECT_EQ(shape.getNumMaterials(), 0u);
  EXPECT_EQ(shape.getMaterial(0), nullptr);
}

TEST(MeshShapeTest, LoadMeshFromFilePathAndUriAccessors)
{
  const std::string filePath = dart::config::dataPath("obj/BoxSmall.obj");

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene = dynamics::MeshShape::loadMesh(filePath);
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(scene, nullptr);

  const common::Uri fileUri = common::Uri::createFromPath(filePath);
  auto retriever = std::make_shared<common::LocalResourceRetriever>();
  DART_SUPPRESS_DEPRECATED_BEGIN
  dynamics::MeshShape shape(Eigen::Vector3d::Ones(), scene, fileUri, retriever);
  DART_SUPPRESS_DEPRECATED_END

  EXPECT_EQ(shape.getMeshUri(), fileUri.toString());
  EXPECT_EQ(shape.getMeshUri2().toString(), fileUri.toString());
  EXPECT_EQ(shape.getMeshPath(), filePath);
  EXPECT_EQ(shape.getResourceRetriever(), retriever);
}

TEST(MeshShapeTest, UriConstructionSetMeshAndRenderSettings)
{
  const std::string filePath = dart::config::dataPath("obj/BoxSmall.obj");
  const common::Uri fileUri = common::Uri::createFromPath(filePath);
  auto retriever = std::make_shared<common::LocalResourceRetriever>();

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene
      = dynamics::MeshShape::loadMesh(fileUri.toString(), retriever);
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(scene, nullptr);

  DART_SUPPRESS_DEPRECATED_BEGIN
  dynamics::MeshShape shape(Eigen::Vector3d::Ones(), scene, fileUri, retriever);
  DART_SUPPRESS_DEPRECATED_END

  EXPECT_EQ(shape.getMeshUri(), fileUri.toString());
  EXPECT_EQ(shape.getMeshPath(), filePath);

  shape.setColorMode(dynamics::MeshShape::SHAPE_COLOR);
  shape.setAlphaMode(dynamics::MeshShape::AUTO);
  shape.setColorIndex(2);
  EXPECT_EQ(shape.getColorMode(), dynamics::MeshShape::SHAPE_COLOR);
  EXPECT_EQ(shape.getAlphaMode(), dynamics::MeshShape::AUTO);
  EXPECT_EQ(shape.getColorIndex(), 2);

  shape.setDisplayList(7);
  EXPECT_EQ(shape.getDisplayList(), 7);

  const auto extents = shape.getBoundingBox().computeFullExtents();
  EXPECT_TRUE(extents.allFinite());
  EXPECT_GT(shape.getVolume(), 0.0);

  const std::string quadPath = dart::config::dataPath("obj/Quad.obj");
  const common::Uri quadUri = common::Uri::createFromPath(quadPath);

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* quadScene
      = dynamics::MeshShape::loadMesh(quadUri.toString(), retriever);
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(quadScene, nullptr);

  DART_SUPPRESS_DEPRECATED_BEGIN
  shape.setMesh(
      quadScene,
      dynamics::MeshShape::MeshOwnership::Imported,
      quadUri,
      retriever);
  DART_SUPPRESS_DEPRECATED_END

  EXPECT_EQ(shape.getMeshUri(), quadUri.toString());
  EXPECT_EQ(shape.getMeshPath(), quadPath);
}
