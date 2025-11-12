#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/AssimpInputResourceAdaptor.hpp"
#include "dart/dynamics/MeshShape.hpp"

#include <assimp/cimport.h>
#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <gtest/gtest.h>

#include <fstream>
#include <memory>
#include <string>

using namespace dart;

namespace {

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

TEST(MeshShapeTest, ColladaUnitMetadataApplied)
{
#ifndef AI_CONFIG_IMPORT_COLLADA_IGNORE_UNIT_SIZE
  GTEST_SKIP() << "Assimp build does not expose unit-size control property.";
#endif

  const std::string filePath
      = std::string(DART_DATA_PATH) + "skel/kima/l-foot.dae";
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
