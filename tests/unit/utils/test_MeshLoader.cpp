/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * This file provides unit tests for the new MeshLoader abstraction
 */

#include <dart/utils/AssimpMeshLoader.hpp>
#include <dart/utils/MeshLoader.hpp>
#include <dart/utils/DartResourceRetriever.hpp>
#include <dart/common/LocalResourceRetriever.hpp>
#include <dart/math/TriMesh.hpp>

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
TEST(MeshLoader, TriMeshAPICompleteness)
{
  // Verify that TriMesh has all the methods needed by the loader
  auto mesh = std::make_shared<math::TriMesh<double>>();

  // Test reserve methods
  mesh->reserveVertices(3);
  mesh->reserveTriangles(1);
  mesh->reserveVertexNormals(3);

  // Test add methods
  mesh->addVertex(0.0, 0.0, 0.0);
  mesh->addVertex(1.0, 0.0, 0.0);
  mesh->addVertex(0.0, 1.0, 0.0);

  mesh->addTriangle(0, 1, 2);

  mesh->addVertexNormal(0.0, 0.0, 1.0);
  mesh->addVertexNormal(0.0, 0.0, 1.0);
  mesh->addVertexNormal(0.0, 0.0, 1.0);

  // Verify the mesh
  EXPECT_EQ(mesh->getVertices().size(), 3);
  EXPECT_EQ(mesh->getTriangles().size(), 1);
  EXPECT_EQ(mesh->getVertexNormals().size(), 3);

  // Test vector overloads
  Eigen::Vector3d vertex(0.5, 0.5, 0.0);
  mesh->addVertex(vertex);
  EXPECT_EQ(mesh->getVertices().size(), 4);

  Eigen::Matrix<std::size_t, 3, 1> triangle;
  triangle << 0, 1, 3;
  mesh->addTriangle(triangle);
  EXPECT_EQ(mesh->getTriangles().size(), 2);

  Eigen::Vector3d normal(0.0, 1.0, 0.0);
  mesh->addVertexNormal(normal);
  EXPECT_EQ(mesh->getVertexNormals().size(), 4);
}

//==============================================================================
TEST(MeshLoader, TriMeshComputeNormals)
{
  auto mesh = std::make_shared<math::TriMesh<double>>();

  // Create a simple triangle
  mesh->addVertex(0.0, 0.0, 0.0);
  mesh->addVertex(1.0, 0.0, 0.0);
  mesh->addVertex(0.0, 1.0, 0.0);
  mesh->addTriangle(0, 1, 2);

  // Compute normals
  mesh->computeVertexNormals();

  // Verify normals were computed
  EXPECT_TRUE(mesh->hasVertexNormals());
  EXPECT_EQ(mesh->getVertexNormals().size(), 3);

  // Normals should point in +Z direction (approximately)
  for (const auto& normal : mesh->getVertexNormals()) {
    EXPECT_NEAR(normal.z(), 1.0, 1e-10);
    EXPECT_NEAR(normal.x(), 0.0, 1e-10);
    EXPECT_NEAR(normal.y(), 0.0, 1e-10);
  }
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
  EXPECT_GE(mesh->getTriangles().size(), 12); // At least 12 triangles for a box (6 faces * 2)

  // Verify normals are computed or loaded
  if (mesh->hasVertexNormals()) {
    EXPECT_EQ(mesh->getVertexNormals().size(), mesh->getVertices().size());
  }
}
