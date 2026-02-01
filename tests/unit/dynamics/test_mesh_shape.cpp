#include "dart/common/diagnostics.hpp"

#include <dart/all.hpp>

#include <Eigen/Core>
#include <assimp/cexport.h>
#include <assimp/cimport.h>
#include <assimp/config.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <iterator>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>

#include <cmath>
#include <cstring>

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

class StringResource final : public common::Resource
{
public:
  explicit StringResource(std::string data)
    : mData(std::move(data)), mPosition(0)
  {
  }

  std::size_t getSize() override
  {
    return mData.size();
  }

  std::size_t tell() override
  {
    return mPosition;
  }

  bool seek(ptrdiff_t offset, SeekType origin) override
  {
    ptrdiff_t base = 0;
    switch (origin) {
      case SEEKTYPE_CUR:
        base = static_cast<ptrdiff_t>(mPosition);
        break;
      case SEEKTYPE_END:
        base = std::ssize(mData);
        break;
      case SEEKTYPE_SET:
        base = 0;
        break;
      default:
        return false;
    }

    const ptrdiff_t next = base + offset;
    if (next < 0) {
      return false;
    }
    if (static_cast<std::size_t>(next) > mData.size()) {
      return false;
    }

    mPosition = static_cast<std::size_t>(next);
    return true;
  }

  std::size_t read(void* buffer, std::size_t size, std::size_t count) override
  {
    if (!buffer || size == 0 || count == 0) {
      return 0;
    }

    const std::size_t remaining = mData.size() - mPosition;
    const std::size_t availableCount = remaining / size;
    const std::size_t toReadCount = std::min(count, availableCount);
    const std::size_t toReadBytes = toReadCount * size;

    if (toReadBytes > 0) {
      std::memcpy(buffer, mData.data() + mPosition, toReadBytes);
      mPosition += toReadBytes;
    }

    return toReadCount;
  }

private:
  std::string mData;
  std::size_t mPosition;
};

class StringResourceRetriever final : public common::ResourceRetriever
{
public:
  explicit StringResourceRetriever(
      std::unordered_map<std::string, std::string> data)
    : mData(std::move(data))
  {
  }

  bool exists(const common::Uri& uri) override
  {
    return findData(uri) != nullptr;
  }

  common::ResourcePtr retrieve(const common::Uri& uri) override
  {
    const std::string* data = findData(uri);
    if (!data) {
      return nullptr;
    }
    return std::make_shared<StringResource>(*data);
  }

  std::string getFilePath(const common::Uri& /*uri*/) override
  {
    return std::string();
  }

private:
  const std::string* findData(const common::Uri& uri) const
  {
    const auto byFull = mData.find(uri.toString());
    if (byFull != mData.end()) {
      return &byFull->second;
    }

    const auto findByPath = [&](const std::string& path) -> const std::string* {
      const auto byPath = mData.find(path);
      if (byPath != mData.end()) {
        return &byPath->second;
      }
      if (!path.empty() && path.front() == '/') {
        const auto byTrim = mData.find(path.substr(1));
        if (byTrim != mData.end()) {
          return &byTrim->second;
        }
      }
      const auto slash = path.find_last_of("/\\");
      if (slash != std::string::npos) {
        const auto byName = mData.find(path.substr(slash + 1));
        if (byName != mData.end()) {
          return &byName->second;
        }
      }
      return nullptr;
    };

    if (uri.mPath) {
      if (const std::string* data = findByPath(uri.mPath.get())) {
        return data;
      }
    }

    if (uri.mAuthority) {
      if (const std::string* data = findByPath(uri.mAuthority.get())) {
        return data;
      }
    }

    return nullptr;
  }

  std::unordered_map<std::string, std::string> mData;
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
    const std::string& filePath, bool ignoreUnitSize)
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

  const aiScene* scene = aiImportFileExWithProperties(
      filePath.c_str(),
      aiProcess_GenNormals | aiProcess_Triangulate
          | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType
          | aiProcess_OptimizeMeshes,
      nullptr,
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

std::shared_ptr<math::TriMesh<double>> loadTriMeshFromUri(
    const std::string& uri, const common::ResourceRetrieverPtr& retriever)
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene = dynamics::MeshShape::loadMesh(uri, retriever);
  DART_SUPPRESS_DEPRECATED_END
  if (!scene) {
    return nullptr;
  }

  class MeshShapeHarness final : public dynamics::MeshShape
  {
  public:
    using dynamics::MeshShape::convertAssimpMesh;
  };

  auto triMesh = MeshShapeHarness::convertAssimpMesh(scene);
  aiReleaseImport(const_cast<aiScene*>(scene));
  return triMesh;
}

class SubmeshMismatchMeshShape final : public dynamics::MeshShape
{
public:
  using dynamics::MeshShape::MeshShape;
  using dynamics::MeshShape::SubMeshRange;

  void setSubMeshRanges(std::vector<SubMeshRange> ranges)
  {
    mSubMeshRanges = std::move(ranges);
  }

  void setMaterials(std::vector<dynamics::MeshMaterial> materials)
  {
    mMaterials = std::move(materials);
  }

  const aiScene* callConvertToAssimpMesh() const
  {
    return convertToAssimpMesh();
  }
};

class SubmeshMaterialMeshShape final : public dynamics::MeshShape
{
public:
  using dynamics::MeshShape::MeshShape;
  using dynamics::MeshShape::SubMeshRange;

  void setSubMeshRanges(std::vector<SubMeshRange> ranges)
  {
    mSubMeshRanges = std::move(ranges);
  }

  void setMaterials(std::vector<dynamics::MeshMaterial> materials)
  {
    mMaterials = std::move(materials);
  }

  void setTextureCoords(std::vector<Eigen::Vector3d> coords, int components)
  {
    mTextureCoords = std::move(coords);
    mTextureCoordComponents = components;
  }

  const aiScene* callConvertToAssimpMesh() const
  {
    return convertToAssimpMesh();
  }
};

class MeshHandleHarness final : public dynamics::MeshShape
{
public:
  using dynamics::MeshShape::MeshShape;

  MeshHandle& meshHandle()
  {
    return mMesh;
  }

  const MeshHandle& meshHandle() const
  {
    return mMesh;
  }

  void resetMeshHandle()
  {
    mMesh.reset();
  }

  void setCachedScene(const aiScene* scene)
  {
    mCachedAiScene = scene;
  }

  void clearTriMesh()
  {
    mTriMesh.reset();
  }
};

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

  const aiScene* sceneIgnoringUnits = loadMeshWithOverrides(filePath, true);
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
  auto triMesh = loadTriMeshFromUri(aliasUri, aliasRetriever);
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
  auto triMesh = loadTriMeshFromUri(
      common::Uri::createFromPath(objPath.string()).toString(), retriever);
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
  auto triMesh = loadTriMeshFromUri(
      common::Uri::createFromPath(objPath.string()).toString(), retriever);
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
  auto triMesh = loadTriMeshFromUri(
      common::Uri::createFromPath(objPath.string()).toString(), retriever);
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

TEST(MeshShapeTest, ConvertToAssimpMeshUsesMemoryResourceForSubMeshes)
{
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addVertex(1.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);
  triMesh->addTriangle(1, 3, 2);
  triMesh->addVertexNormal(0.0, 0.0, 1.0);
  triMesh->addVertexNormal(0.0, 0.0, 1.0);
  triMesh->addVertexNormal(0.0, 0.0, 1.0);
  triMesh->addVertexNormal(0.0, 0.0, 1.0);

  SubmeshMaterialMeshShape shape(
      Eigen::Vector3d::Ones(), triMesh, common::Uri());

  std::vector<dynamics::MeshMaterial> materials(2);
  materials[0].diffuse = Eigen::Vector4f(0.2f, 0.3f, 0.4f, 1.0f);
  materials[1].diffuse = Eigen::Vector4f(0.5f, 0.6f, 0.7f, 1.0f);
  shape.setMaterials(std::move(materials));

  std::vector<SubmeshMaterialMeshShape::SubMeshRange> ranges(2);
  ranges[0].vertexOffset = 0u;
  ranges[0].vertexCount = 2u;
  ranges[0].triangleOffset = 0u;
  ranges[0].triangleCount = 1u;
  ranges[0].materialIndex = 0u;
  ranges[1].vertexOffset = 2u;
  ranges[1].vertexCount = 2u;
  ranges[1].triangleOffset = 1u;
  ranges[1].triangleCount = 1u;
  ranges[1].materialIndex = 1u;
  shape.setSubMeshRanges(std::move(ranges));

  const aiScene* scene = shape.callConvertToAssimpMesh();
  ASSERT_NE(scene, nullptr);
  EXPECT_GE(scene->mNumMeshes, 1u);
  EXPECT_GE(scene->mNumMaterials, 2u);
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

TEST(MeshShapeTest, EmptyMeshBoundingBoxAndVolume)
{
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  dynamics::MeshShape shape(Eigen::Vector3d(2.0, 3.0, 4.0), triMesh);

  const auto extents = shape.getBoundingBox().computeFullExtents();
  EXPECT_TRUE(extents.isApprox(Eigen::Vector3d::Zero(), 1e-12));
  EXPECT_DOUBLE_EQ(shape.getVolume(), 0.0);
}

TEST(MeshShapeTest, PolygonMeshPreservesVertexNormals)
{
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);
  triMesh->addVertexNormal(0.0, 0.0, 1.0);
  triMesh->addVertexNormal(0.0, 0.0, 1.0);
  triMesh->addVertexNormal(0.0, 0.0, 1.0);

  dynamics::MeshShape shape(Eigen::Vector3d::Ones(), triMesh, common::Uri());
  const auto polygonMesh = shape.getPolygonMesh();
  ASSERT_NE(polygonMesh, nullptr);
  ASSERT_TRUE(polygonMesh->hasVertexNormals());
  EXPECT_EQ(polygonMesh->getVertexNormals().size(), 3u);
}

TEST(MeshShapeTest, ConvertToAssimpMeshSkipsInvalidSubmeshRanges)
{
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addVertex(1.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);
  triMesh->addTriangle(1, 3, 2);

  SubmeshMismatchMeshShape shape(
      Eigen::Vector3d::Ones(), triMesh, common::Uri());

  std::vector<dynamics::MeshMaterial> materials(2);
  shape.setMaterials(materials);

  std::vector<SubmeshMismatchMeshShape::SubMeshRange> ranges(1);
  ranges[0].vertexOffset = 0;
  ranges[0].vertexCount = triMesh->getVertices().size() + 1;
  ranges[0].triangleOffset = 0;
  ranges[0].triangleCount = triMesh->getTriangles().size();
  ranges[0].materialIndex = 0;
  shape.setSubMeshRanges(std::move(ranges));

  const aiScene* scene = shape.callConvertToAssimpMesh();
  ASSERT_NE(scene, nullptr);
  EXPECT_GE(scene->mNumMeshes, 1u);
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

TEST(MeshShapeTest, SetMeshWithUriStringAndCloneMesh)
{
  const std::string meshPath
      = dart::config::dataPath("urdf/KR5/meshes/base_link.STL");
  const common::Uri meshUri = common::Uri::createFromPath(meshPath);

  DART_SUPPRESS_DEPRECATED_BEGIN
  class MeshShapeCloneHarness final : public dynamics::MeshShape
  {
  public:
    using dynamics::MeshShape::cloneMesh;
    using dynamics::MeshShape::MeshShape;
  };
  DART_SUPPRESS_DEPRECATED_END

  DART_SUPPRESS_DEPRECATED_BEGIN
  auto retriever = std::make_shared<common::LocalResourceRetriever>();
  const aiScene* scene
      = dynamics::MeshShape::loadMesh(meshUri.toString(), retriever);
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(scene, nullptr);

  DART_SUPPRESS_DEPRECATED_BEGIN
  MeshShapeCloneHarness shape(
      Eigen::Vector3d::Ones(), scene, meshUri, retriever);
  shape.setMesh(scene, meshUri.toString(), retriever);
  DART_SUPPRESS_DEPRECATED_END

  EXPECT_EQ(shape.getMeshUri(), meshUri.toString());
  EXPECT_FALSE(shape.getMeshPath().empty());

  aiScene* clonedScene = shape.cloneMesh();
  ASSERT_NE(clonedScene, nullptr);
  EXPECT_NE(clonedScene, scene);
  EXPECT_EQ(clonedScene->mNumMeshes, scene->mNumMeshes);
  aiReleaseImport(clonedScene);
}

TEST(MeshShapeTest, BoundingBoxVolumeAndColorModes)
{
  auto mesh = createTetraMesh();
  dynamics::MeshShape shape(Eigen::Vector3d(1.0, 2.0, 0.5), mesh);

  const auto extents = shape.getBoundingBox().computeFullExtents();
  EXPECT_TRUE(extents.isApprox(Eigen::Vector3d(1.0, 2.0, 0.5), 1e-12));
  EXPECT_NEAR(shape.getVolume(), 1.0 * 2.0 * 0.5, 1e-12);

  shape.setScale(2.0);
  const auto scaledExtents = shape.getBoundingBox().computeFullExtents();
  EXPECT_TRUE(scaledExtents.isApprox(Eigen::Vector3d::Constant(2.0), 1e-12));
  EXPECT_NEAR(shape.getVolume(), 8.0, 1e-12);

  shape.setColorMode(dynamics::MeshShape::SHAPE_COLOR);
  EXPECT_EQ(shape.getColorMode(), dynamics::MeshShape::SHAPE_COLOR);
  shape.setColorMode(dynamics::MeshShape::MATERIAL_COLOR);
  EXPECT_EQ(shape.getColorMode(), dynamics::MeshShape::MATERIAL_COLOR);
  shape.setColorMode(dynamics::MeshShape::COLOR_INDEX);
  shape.setColorIndex(3);
  EXPECT_EQ(shape.getColorIndex(), 3);
}

TEST(MeshShapeTest, LoadMeshFromTempObjAndUriMetadata)
{
  const auto tempDir = common::filesystem::temp_directory_path();
  const auto timestamp
      = std::chrono::high_resolution_clock::now().time_since_epoch().count();
  const auto baseName
      = std::string("dart_meshshape_load_") + std::to_string(timestamp);
  const auto objPath = tempDir / (baseName + ".obj");

  {
    std::ofstream objFile(objPath.string());
    ASSERT_TRUE(objFile);
    objFile << "v 0 0 0\n"
            << "v 1 0 0\n"
            << "v 0 1 0\n"
            << "f 1 2 3\n";
  }

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene = dynamics::MeshShape::loadMesh(objPath.string());
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(scene, nullptr);

  const common::Uri fileUri = common::Uri::createFromPath(objPath.string());
  auto retriever = std::make_shared<common::LocalResourceRetriever>();
  DART_SUPPRESS_DEPRECATED_BEGIN
  dynamics::MeshShape shape(Eigen::Vector3d::Ones(), scene, fileUri, retriever);
  DART_SUPPRESS_DEPRECATED_END

  EXPECT_EQ(shape.getMeshUri(), fileUri.toString());
  EXPECT_EQ(shape.getMeshPath(), objPath.string());
  EXPECT_GE(shape.getVolume(), 0.0);

  const common::Uri nonFileUri("package://example/mesh.obj");
  auto uriMesh = createTetraMesh();
  dynamics::MeshShape uriShape(Eigen::Vector3d::Ones(), uriMesh, nonFileUri);
  EXPECT_EQ(uriShape.getMeshUri(), nonFileUri.toString());
  EXPECT_TRUE(uriShape.getMeshPath().empty());

  common::error_code ec;
  common::filesystem::remove(objPath, ec);
}

TEST(MeshShapeTest, ExtractMaterialsResolvesTexturePaths)
{
  const auto tempDir = common::filesystem::temp_directory_path();
  const auto timestamp
      = std::chrono::high_resolution_clock::now().time_since_epoch().count();
  const auto baseName = std::string("dart_meshshape_materials_tex_")
                        + std::to_string(timestamp);
  const auto objPath = tempDir / (baseName + ".obj");
  const auto mtlPath = tempDir / (baseName + ".mtl");
  const auto texPath = tempDir / (baseName + ".png");

  {
    std::ofstream texFile(texPath.string());
    ASSERT_TRUE(texFile);
    texFile << " ";
  }

  {
    std::ofstream mtlFile(mtlPath.string());
    ASSERT_TRUE(mtlFile);
    mtlFile << "newmtl material_0\n"
            << "Ka 0 0 0\n"
            << "Kd 0.8 0.7 0.6\n"
            << "Ks 0 0 0\n"
            << "Ns 0\n"
            << "map_Kd " << texPath.filename().string() << "\n";
  }

  {
    std::ofstream objFile(objPath.string());
    ASSERT_TRUE(objFile);
    objFile << "mtllib " << mtlPath.filename().string() << "\n"
            << "v 0 0 0\n"
            << "v 1 0 0\n"
            << "v 0 1 0\n"
            << "usemtl material_0\n"
            << "f 1 2 3\n";
  }

  auto retriever = std::make_shared<common::LocalResourceRetriever>();
  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene
      = dynamics::MeshShape::loadMesh(objPath.string(), retriever);
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(scene, nullptr);

  DART_SUPPRESS_DEPRECATED_BEGIN
  dynamics::MeshShape shape(
      Eigen::Vector3d::Ones(),
      scene,
      common::Uri::createFromPath(objPath.string()),
      retriever);
  DART_SUPPRESS_DEPRECATED_END

  common::error_code ec;
  const auto expectedPath = common::filesystem::canonical(texPath, ec).string();
  ASSERT_FALSE(ec);

  bool foundTexture = false;
  for (const auto& material : shape.getMaterials()) {
    if (material.textureImagePaths.empty()) {
      continue;
    }
    foundTexture = true;
    EXPECT_EQ(material.textureImagePaths.front(), expectedPath);
  }
  EXPECT_TRUE(foundTexture);

  common::filesystem::remove(objPath, ec);
  common::filesystem::remove(mtlPath, ec);
  common::filesystem::remove(texPath, ec);
}

TEST(MeshShapeTest, ConvertToAssimpMeshWithSubMeshesAndTexCoords)
{
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addVertex(1.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);
  triMesh->addTriangle(1, 3, 2);
  for (int i = 0; i < 4; ++i) {
    triMesh->addVertexNormal(0.0, 0.0, 1.0);
  }

  SubmeshMaterialMeshShape shape(
      Eigen::Vector3d::Ones(), triMesh, common::Uri());

  std::vector<dynamics::MeshMaterial> materials(2);
  materials[0].diffuse = Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f);
  materials[1].diffuse = Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f);
  shape.setMaterials(std::move(materials));

  std::vector<SubmeshMaterialMeshShape::SubMeshRange> ranges(2);
  ranges[0].vertexOffset = 0u;
  ranges[0].vertexCount = 2u;
  ranges[0].triangleOffset = 0u;
  ranges[0].triangleCount = 1u;
  ranges[0].materialIndex = 0u;
  ranges[1].vertexOffset = 2u;
  ranges[1].vertexCount = 2u;
  ranges[1].triangleOffset = 1u;
  ranges[1].triangleCount = 1u;
  ranges[1].materialIndex = 1u;
  shape.setSubMeshRanges(std::move(ranges));

  std::vector<Eigen::Vector3d> texCoords;
  texCoords.emplace_back(0.0, 0.0, 0.0);
  texCoords.emplace_back(1.0, 0.0, 0.0);
  texCoords.emplace_back(0.0, 1.0, 0.0);
  texCoords.emplace_back(1.0, 1.0, 0.0);
  shape.setTextureCoords(std::move(texCoords), 2);

  const aiScene* scene = shape.callConvertToAssimpMesh();
  ASSERT_NE(scene, nullptr);
  EXPECT_GE(scene->mNumMaterials, 2u);
  EXPECT_GE(scene->mNumMeshes, 1u);
  ASSERT_NE(scene->mMeshes, nullptr);
  EXPECT_TRUE(scene->mMeshes[0]->HasTextureCoords(0));
}

TEST(MeshShapeTest, MeshHandleAccessorsAndOwnership)
{
  const std::string objData
      = "v 0 0 0\n"
        "v 1 0 0\n"
        "v 0 1 0\n"
        "f 1 2 3\n";
  const unsigned int flags = aiProcess_Triangulate
                             | aiProcess_JoinIdenticalVertices
                             | aiProcess_SortByPType;
  const aiScene* imported = aiImportFileFromMemory(
      objData.data(), static_cast<unsigned int>(objData.size()), flags, "obj");
  ASSERT_NE(imported, nullptr);

  aiScene* copied = nullptr;
  aiCopyScene(imported, &copied);
  ASSERT_NE(copied, nullptr);

  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);

  MeshHandleHarness shape(Eigen::Vector3d::Ones(), triMesh, common::Uri());
  shape.meshHandle().set(
      imported, dynamics::MeshShape::MeshOwnership::Imported);
  EXPECT_TRUE(shape.meshHandle());
  EXPECT_EQ(shape.meshHandle().get(), imported);
  EXPECT_EQ(shape.meshHandle()->mNumMeshes, imported->mNumMeshes);
  EXPECT_EQ(
      shape.meshHandle().getOwnership(),
      dynamics::MeshShape::MeshOwnership::Imported);
  EXPECT_EQ(shape.meshHandle().getShared().get(), imported);

  shape.meshHandle().reset();
  EXPECT_FALSE(shape.meshHandle());
  EXPECT_EQ(
      shape.meshHandle().getOwnership(),
      dynamics::MeshShape::MeshOwnership::None);
  EXPECT_EQ(shape.meshHandle().getShared().get(), nullptr);

  shape.meshHandle().set(copied, dynamics::MeshShape::MeshOwnership::Copied);
  EXPECT_TRUE(shape.meshHandle());
  EXPECT_EQ(
      shape.meshHandle().getOwnership(),
      dynamics::MeshShape::MeshOwnership::Copied);
  shape.meshHandle().reset();

  auto* manualScene = new aiScene();
  shape.meshHandle().set(
      manualScene, dynamics::MeshShape::MeshOwnership::Manual);
  EXPECT_EQ(
      shape.meshHandle().getOwnership(),
      dynamics::MeshShape::MeshOwnership::Manual);
  shape.meshHandle().reset();

  auto customScene = std::shared_ptr<const aiScene>(
      new aiScene(),
      [](const aiScene* scene) { delete const_cast<aiScene*>(scene); });
  shape.meshHandle().set(customScene);
  EXPECT_EQ(
      shape.meshHandle().getOwnership(),
      dynamics::MeshShape::MeshOwnership::Custom);
  EXPECT_EQ(shape.meshHandle().getShared().get(), customScene.get());
  shape.meshHandle().reset();
}

TEST(MeshShapeTest, GetTriMeshUsesCachedScene)
{
  const std::string objData
      = "v 0 0 0\n"
        "v 1 0 0\n"
        "v 0 1 0\n"
        "f 1 2 3\n";
  const unsigned int flags = aiProcess_Triangulate
                             | aiProcess_JoinIdenticalVertices
                             | aiProcess_SortByPType;
  const aiScene* imported = aiImportFileFromMemory(
      objData.data(), static_cast<unsigned int>(objData.size()), flags, "obj");
  ASSERT_NE(imported, nullptr);

  auto triMesh = std::make_shared<math::TriMesh<double>>();
  MeshHandleHarness shape(Eigen::Vector3d::Ones(), triMesh, common::Uri());
  shape.clearTriMesh();
  shape.resetMeshHandle();
  shape.setCachedScene(imported);

  const auto cachedTriMesh = shape.getTriMesh();
  ASSERT_NE(cachedTriMesh, nullptr);
  EXPECT_TRUE(cachedTriMesh->hasTriangles());
  EXPECT_EQ(cachedTriMesh->getVertices().size(), 3u);
}

TEST(MeshShapeTest, ConvertToAssimpMeshExportsMtlAndTexCoordZ)
{
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addVertex(1.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);
  triMesh->addTriangle(1, 3, 2);
  for (int i = 0; i < 4; ++i) {
    triMesh->addVertexNormal(0.0, 0.0, 1.0);
  }

  SubmeshMaterialMeshShape shape(
      Eigen::Vector3d::Ones(), triMesh, common::Uri());

  std::vector<dynamics::MeshMaterial> materials(2);
  materials[0].ambient = Eigen::Vector4f(0.1f, 0.2f, 0.3f, 1.0f);
  materials[0].diffuse = Eigen::Vector4f(0.4f, 0.5f, 0.6f, 1.0f);
  materials[0].specular = Eigen::Vector4f(0.2f, 0.3f, 0.4f, 1.0f);
  materials[0].emissive = Eigen::Vector4f(0.0f, 0.1f, 0.2f, 1.0f);
  materials[0].shininess = 42.0f;
  materials[0].textureImagePaths = {"albedo.png"};
  materials[1].diffuse = Eigen::Vector4f(0.2f, 0.8f, 0.1f, 1.0f);
  shape.setMaterials(std::move(materials));

  std::vector<SubmeshMaterialMeshShape::SubMeshRange> ranges(2);
  ranges[0].vertexOffset = 0u;
  ranges[0].vertexCount = 2u;
  ranges[0].triangleOffset = 0u;
  ranges[0].triangleCount = 1u;
  ranges[0].materialIndex = 0u;
  ranges[1].vertexOffset = 2u;
  ranges[1].vertexCount = 2u;
  ranges[1].triangleOffset = 1u;
  ranges[1].triangleCount = 1u;
  ranges[1].materialIndex = 1u;
  shape.setSubMeshRanges(std::move(ranges));

  std::vector<Eigen::Vector3d> texCoords;
  texCoords.emplace_back(0.0, 0.0, 0.25);
  texCoords.emplace_back(1.0, 0.0, 0.5);
  texCoords.emplace_back(0.0, 1.0, 0.75);
  texCoords.emplace_back(1.0, 1.0, 1.0);
  shape.setTextureCoords(std::move(texCoords), 3);

  const aiScene* scene = shape.callConvertToAssimpMesh();
  ASSERT_NE(scene, nullptr);
  ASSERT_GE(scene->mNumMeshes, 1u);
  ASSERT_GE(scene->mNumMaterials, 2u);

  const aiMesh* mesh = scene->mMeshes[0];
  ASSERT_NE(mesh, nullptr);
  ASSERT_TRUE(mesh->HasTextureCoords(0));
  bool hasZ = false;
  for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
    if (mesh->mTextureCoords[0][i].z > 0.1f) {
      hasZ = true;
      break;
    }
  }
  EXPECT_TRUE(hasZ);

  const aiMaterial* mat0 = scene->mMaterials[0];
  EXPECT_NE(mat0, nullptr);
}

TEST(MeshShapeTest, ConvertToAssimpMeshSubmeshMismatchUsesSingleMesh)
{
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addVertex(1.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);
  triMesh->addTriangle(1, 3, 2);

  SubmeshMismatchMeshShape shape(
      Eigen::Vector3d::Ones(), triMesh, common::Uri());

  std::vector<dynamics::MeshMaterial> materials(2);
  shape.setMaterials(std::move(materials));

  std::vector<SubmeshMismatchMeshShape::SubMeshRange> ranges(1);
  ranges[0].vertexOffset = 0u;
  ranges[0].vertexCount = triMesh->getVertices().size();
  ranges[0].triangleOffset = 0u;
  ranges[0].triangleCount = triMesh->getTriangles().size() - 1u;
  ranges[0].materialIndex = 0u;
  shape.setSubMeshRanges(std::move(ranges));

  const aiScene* scene = shape.callConvertToAssimpMesh();
  ASSERT_NE(scene, nullptr);
  EXPECT_EQ(scene->mNumMaterials, 1u);
  EXPECT_GE(scene->mNumMeshes, 1u);
}

TEST(MeshShapeTest, ConvertToAssimpMeshSubmeshFaceFormats)
{
  auto runCase = [](bool withTexCoords,
                    bool withNormals,
                    bool addZeroRange,
                    unsigned int materialIndex) -> const aiScene* {
    auto triMesh = std::make_shared<math::TriMesh<double>>();
    triMesh->addVertex(0.0, 0.0, 0.0);
    triMesh->addVertex(1.0, 0.0, 0.0);
    triMesh->addVertex(0.0, 1.0, 0.0);
    triMesh->addTriangle(0, 1, 2);
    if (withNormals) {
      triMesh->addVertexNormal(0.0, 0.0, 1.0);
      triMesh->addVertexNormal(0.0, 0.0, 1.0);
      triMesh->addVertexNormal(0.0, 0.0, 1.0);
    }

    SubmeshMaterialMeshShape shape(
        Eigen::Vector3d::Ones(), triMesh, common::Uri());

    std::vector<dynamics::MeshMaterial> materials(2);
    shape.setMaterials(std::move(materials));

    std::vector<SubmeshMaterialMeshShape::SubMeshRange> ranges;
    if (addZeroRange) {
      SubmeshMaterialMeshShape::SubMeshRange emptyRange;
      emptyRange.vertexOffset = 0u;
      emptyRange.vertexCount = 0u;
      emptyRange.triangleOffset = 0u;
      emptyRange.triangleCount = 0u;
      emptyRange.materialIndex = 0u;
      ranges.push_back(emptyRange);
    }
    SubmeshMaterialMeshShape::SubMeshRange range;
    range.vertexOffset = 0u;
    range.vertexCount = 3u;
    range.triangleOffset = 0u;
    range.triangleCount = 1u;
    range.materialIndex = materialIndex;
    ranges.push_back(range);
    shape.setSubMeshRanges(std::move(ranges));

    if (withTexCoords) {
      std::vector<Eigen::Vector3d> texCoords;
      texCoords.emplace_back(0.0, 0.0, 0.0);
      texCoords.emplace_back(1.0, 0.0, 0.0);
      texCoords.emplace_back(0.0, 1.0, 0.0);
      shape.setTextureCoords(std::move(texCoords), 2);
    }

    const aiScene* scene = shape.callConvertToAssimpMesh();
    if (!scene) {
      ADD_FAILURE() << "convertToAssimpMesh returned null scene";
      return nullptr;
    }
    if (scene->mNumMeshes < 1u) {
      ADD_FAILURE() << "convertToAssimpMesh produced no meshes";
    }
    return scene;
  };

  const aiScene* texScene = runCase(true, false, true, 5u);
  ASSERT_NE(texScene, nullptr);
  EXPECT_TRUE(texScene->mMeshes[0]->HasTextureCoords(0));

  const aiScene* normalScene = runCase(false, true, false, 0u);
  ASSERT_NE(normalScene, nullptr);
  EXPECT_FALSE(normalScene->mMeshes[0]->HasTextureCoords(0));

  const aiScene* plainScene = runCase(false, false, false, 0u);
  ASSERT_NE(plainScene, nullptr);
  EXPECT_FALSE(plainScene->mMeshes[0]->HasTextureCoords(0));
}

TEST(MeshShapeTest, ConvertToAssimpMeshReturnsNullWithoutTriangles)
{
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);

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
  EXPECT_EQ(scene, nullptr);
}

TEST(MeshShapeTest, ConvertToAssimpMeshHandlesOutOfRangeMaterialIndex)
{
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);

  SubmeshMaterialMeshShape shape(
      Eigen::Vector3d::Ones(), triMesh, common::Uri());

  std::vector<dynamics::MeshMaterial> materials(2);
  materials[0].diffuse = Eigen::Vector4f(0.4f, 0.4f, 0.4f, 1.0f);
  materials[1].diffuse = Eigen::Vector4f(0.1f, 0.7f, 0.2f, 1.0f);
  shape.setMaterials(std::move(materials));

  std::vector<SubmeshMaterialMeshShape::SubMeshRange> ranges(1);
  ranges[0].vertexOffset = 0u;
  ranges[0].vertexCount = 3u;
  ranges[0].triangleOffset = 0u;
  ranges[0].triangleCount = 1u;
  ranges[0].materialIndex = 5u;
  shape.setSubMeshRanges(std::move(ranges));

  const aiScene* scene = shape.callConvertToAssimpMesh();
  ASSERT_NE(scene, nullptr);
  EXPECT_GE(scene->mNumMaterials, 1u);
  EXPECT_GE(scene->mNumMeshes, 1u);
}

TEST(MeshShapeTest, LoadMeshFromMemoryResourceWithPathResolution)
{
  const std::string objData
      = "mtllib mesh.mtl\n"
        "v 0 0 0\n"
        "v 1 0 0\n"
        "v 0 1 0\n"
        "usemtl material_0\n"
        "f 1 2 3\n";
  const std::string mtlData
      = "newmtl material_0\n"
        "Ka 0.1 0.2 0.3\n"
        "Kd 0.4 0.5 0.6\n"
        "Ks 0.7 0.8 0.9\n"
        "Ns 12\n";

  std::unordered_map<std::string, std::string> data;
  data.emplace("mesh.obj", objData);
  data.emplace("mesh.mtl", mtlData);

  auto retriever = std::make_shared<StringResourceRetriever>(std::move(data));
  const std::string uriString = "package://example/assets/mesh.obj";

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene = dynamics::MeshShape::loadMesh(uriString, retriever);
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(scene, nullptr);

  const common::Uri meshUri = common::Uri::createFromStringOrPath(uriString);
  DART_SUPPRESS_DEPRECATED_BEGIN
  dynamics::MeshShape shape(Eigen::Vector3d::Ones(), scene, meshUri, retriever);
  DART_SUPPRESS_DEPRECATED_END

  ASSERT_FALSE(shape.getMaterials().empty());
  const auto* material = shape.getMaterial(0u);
  ASSERT_NE(material, nullptr);
}

TEST(MeshShapeTest, LoadMeshFromCustomUriWithMaterial)
{
  const std::string objData
      = "mtllib mesh.mtl\n"
        "v 0 0 0\n"
        "v 1 0 0\n"
        "v 0 1 0\n"
        "usemtl material_0\n"
        "f 1 2 3\n";
  const std::string mtlData
      = "newmtl material_0\n"
        "Ka 0.1 0.2 0.3\n"
        "Kd 0.4 0.5 0.6\n"
        "Ks 0.7 0.8 0.9\n"
        "Ns 12\n";

  std::unordered_map<std::string, std::string> data;
  data.emplace("memory:///mesh.obj", objData);
  data.emplace("memory:///mesh.mtl", mtlData);
  data.emplace("mesh.mtl", mtlData);

  auto retriever = std::make_shared<StringResourceRetriever>(std::move(data));

  const std::string uriString = "memory:///mesh.obj";
  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene = dynamics::MeshShape::loadMesh(uriString, retriever);
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(scene, nullptr);

  const common::Uri meshUri = common::Uri::createFromStringOrPath(uriString);
  DART_SUPPRESS_DEPRECATED_BEGIN
  dynamics::MeshShape shape(Eigen::Vector3d::Ones(), scene, meshUri, retriever);
  DART_SUPPRESS_DEPRECATED_END

  ASSERT_FALSE(shape.getMaterials().empty());
  const auto* material = shape.getMaterial(0u);
  ASSERT_NE(material, nullptr);
}

TEST(MeshShapeTest, TriMeshConstructorPreservesTexturePaths)
{
  const auto tempDir = common::filesystem::temp_directory_path();
  const auto timestamp
      = std::chrono::high_resolution_clock::now().time_since_epoch().count();
  const auto baseName
      = std::string("dart_meshshape_textures_") + std::to_string(timestamp);
  const auto objPath = tempDir / (baseName + ".obj");

  {
    std::ofstream objFile(objPath.string());
    ASSERT_TRUE(objFile);
    objFile << "# placeholder file for mesh uri\n";
  }

  auto* scene = new aiScene();
  scene->mNumMaterials = 1;
  scene->mMaterials = new aiMaterial*[scene->mNumMaterials];
  scene->mMaterials[0] = new aiMaterial();
  aiString texturePath("textures/albedo.png");
  scene->mMaterials[0]->AddProperty(&texturePath, AI_MATKEY_TEXTURE_DIFFUSE(0));

  scene->mNumMeshes = 1;
  scene->mMeshes = new aiMesh*[scene->mNumMeshes];
  auto* mesh = new aiMesh();
  mesh->mNumVertices = 3;
  mesh->mVertices = new aiVector3D[mesh->mNumVertices];
  mesh->mVertices[0] = aiVector3D(0.0f, 0.0f, 0.0f);
  mesh->mVertices[1] = aiVector3D(1.0f, 0.0f, 0.0f);
  mesh->mVertices[2] = aiVector3D(0.0f, 1.0f, 0.0f);
  mesh->mNumFaces = 1;
  mesh->mFaces = new aiFace[mesh->mNumFaces];
  mesh->mFaces[0].mNumIndices = 3;
  mesh->mFaces[0].mIndices = new unsigned int[3];
  mesh->mFaces[0].mIndices[0] = 0u;
  mesh->mFaces[0].mIndices[1] = 1u;
  mesh->mFaces[0].mIndices[2] = 2u;
  mesh->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
  mesh->mMaterialIndex = 0u;
  scene->mMeshes[0] = mesh;

  scene->mRootNode = new aiNode();
  scene->mRootNode->mNumMeshes = 1;
  scene->mRootNode->mMeshes = new unsigned int[1];
  scene->mRootNode->mMeshes[0] = 0u;

  const common::Uri meshUri = common::Uri::createFromPath(objPath.string());

  DART_SUPPRESS_DEPRECATED_BEGIN
  dynamics::MeshShape shape(
      Eigen::Vector3d::Ones(),
      scene,
      meshUri,
      nullptr,
      dynamics::MeshShape::MeshOwnership::Manual);
  DART_SUPPRESS_DEPRECATED_END

  ASSERT_FALSE(shape.getMaterials().empty());
  const auto* material = shape.getMaterial(0u);
  ASSERT_NE(material, nullptr);
  ASSERT_FALSE(material->textureImagePaths.empty());

  bool foundTexture = false;
  for (const auto& path : material->textureImagePaths) {
    if (path.find("textures/albedo.png") != std::string::npos) {
      foundTexture = true;
      break;
    }
  }
  EXPECT_TRUE(foundTexture);

  common::error_code ec;
  common::filesystem::remove(objPath, ec);
}
