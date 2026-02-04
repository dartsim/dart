/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file provides unit tests for the new MeshLoader abstraction
 */

#include <dart/config.hpp>

#include <dart/utils/assimp_mesh_loader.hpp>
#include <dart/utils/dart_resource_retriever.hpp>
#include <dart/utils/mesh_loader.hpp>

#include <dart/common/local_resource_retriever.hpp>

#include <assimp/cimport.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <gtest/gtest.h>

using namespace dart;

//==============================================================================
TEST(MeshLoader, AssimpMeshLoaderBasic)
{
  // Create an AssimpMeshLoader
  auto loader = std::make_unique<utils::AssimpMeshLoaderd>();
  ASSERT_NE(loader, nullptr);
}

//==============================================================================
TEST(MeshLoader, LoadNonExistentFile)
{
  auto loader = std::make_unique<utils::AssimpMeshLoaderd>();
  auto retriever = std::make_shared<common::LocalResourceRetriever>();

  // Try to load a non-existent file
  auto mesh = loader->load("non_existent_file.dae", retriever);
  EXPECT_EQ(mesh, nullptr);
}

//==============================================================================
// Test actual file loading with a real mesh file from the repository
TEST(MeshLoader, LoadActualMesh)
{
  auto loader = std::make_unique<utils::AssimpMeshLoaderd>();
  auto retriever = utils::DartResourceRetriever::create();

  // Use DART resource URI for the sample mesh file
  // This uses the "dart://sample/obj/BoxSmall.obj" pattern that DART tests use
  std::string meshUri = "dart://sample/obj/BoxSmall.obj";
  auto mesh = loader->load(meshUri, retriever);

  ASSERT_NE(mesh, nullptr) << "Failed to load mesh from " << meshUri;

  // Verify the mesh has data
  EXPECT_GT(mesh->getVertices().size(), 0);
  EXPECT_GT(mesh->getTriangles().size(), 0);
  EXPECT_TRUE(mesh->hasVertices());
  EXPECT_TRUE(mesh->hasTriangles());

  // BoxSmall.obj is a simple box, should have reasonable number of vertices
  EXPECT_GE(mesh->getVertices().size(), 8); // At least 8 vertices for a box
  EXPECT_GE(
      mesh->getTriangles().size(),
      12); // At least 12 triangles for a box (6 faces * 2)

  // Verify normals are computed or loaded
  if (mesh->hasVertexNormals()) {
    EXPECT_EQ(mesh->getVertexNormals().size(), mesh->getVertices().size());
  }
}

//==============================================================================
TEST(MeshLoader, LoadPolygonMeshPreservesQuad)
{
  const std::string meshPath = dart::config::dataPath("obj/Quad.obj");
  auto loader = std::make_unique<utils::AssimpMeshLoaderd>();

  auto polygonMesh = loader->loadPolygonMesh(meshPath);
  ASSERT_NE(polygonMesh, nullptr);
  ASSERT_TRUE(polygonMesh->hasFaces());
  ASSERT_EQ(polygonMesh->getFaces().size(), 1u);
  EXPECT_EQ(polygonMesh->getFaces()[0].size(), 4u);

  auto triMesh = loader->load(meshPath);
  ASSERT_NE(triMesh, nullptr);
  EXPECT_EQ(triMesh->getTriangles().size(), 2u);
}

//==============================================================================
TEST(MeshLoader, MergesMultipleMeshes)
{
  const std::string meshPath = dart::config::dataPath("skel/kima/thorax.dae");
  ASSERT_FALSE(meshPath.empty());

  auto loader = std::make_unique<utils::AssimpMeshLoaderd>();
  auto mesh = loader->load(meshPath);

  ASSERT_NE(mesh, nullptr) << "Failed to load mesh from " << meshPath;

  const unsigned int flags = aiProcess_GenNormals | aiProcess_Triangulate
                             | aiProcess_JoinIdenticalVertices
                             | aiProcess_SortByPType | aiProcess_OptimizeMeshes;

  const aiScene* scene = aiImportFile(meshPath.c_str(), flags);
  ASSERT_NE(scene, nullptr) << "Assimp error: " << aiGetErrorString();

  scene = aiApplyPostProcessing(scene, aiProcess_PreTransformVertices);
  ASSERT_NE(scene, nullptr);

  ASSERT_GT(scene->mNumMeshes, 1u)
      << "Expected an asset with multiple meshes to validate merge behavior.";

  std::size_t expectedVertices = 0;
  std::size_t expectedTriangles = 0;
  for (std::size_t meshIndex = 0; meshIndex < scene->mNumMeshes; ++meshIndex) {
    const aiMesh* assimpMesh = scene->mMeshes[meshIndex];
    ASSERT_NE(assimpMesh, nullptr);

    expectedVertices += assimpMesh->mNumVertices;
    for (unsigned int faceIndex = 0; faceIndex < assimpMesh->mNumFaces;
         ++faceIndex) {
      const aiFace& face = assimpMesh->mFaces[faceIndex];
      if (face.mNumIndices >= 3u) {
        expectedTriangles += face.mNumIndices - 2u;
      }
    }
  }

  EXPECT_EQ(mesh->getVertices().size(), expectedVertices);
  EXPECT_EQ(mesh->getTriangles().size(), expectedTriangles);
  EXPECT_TRUE(mesh->hasVertexNormals());
  EXPECT_EQ(mesh->getVertexNormals().size(), mesh->getVertices().size());

  aiReleaseImport(scene);
}
