/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * Integration tests for MeshShape with TriMesh support
 */

#include <dart/dynamics/MeshShape.hpp>
#include <dart/math/TriMesh.hpp>
#include <dart/common/Uri.hpp>
#include <dart/common/Deprecated.hpp>

#include <gtest/gtest.h>

using namespace dart;

//==============================================================================
TEST(MeshShapeIntegration, TriMeshConstructor)
{
  // Create a simple TriMesh
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);

  // Create MeshShape with TriMesh
  Eigen::Vector3d scale(1.0, 1.0, 1.0);
  auto meshShape = std::make_shared<dynamics::MeshShape>(
      scale,
      triMesh,
      common::Uri("test_mesh.dae"));

  ASSERT_NE(meshShape, nullptr);
  EXPECT_EQ(meshShape->getScale(), scale);
  EXPECT_EQ(meshShape->getMeshUri(), "test_mesh.dae");
}

//==============================================================================
TEST(MeshShapeIntegration, GetTriMesh)
{
  // Create a TriMesh with known data
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addVertex(1.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);
  triMesh->addTriangle(1, 3, 2);

  // Create MeshShape
  Eigen::Vector3d scale(2.0, 2.0, 2.0);
  auto meshShape = std::make_shared<dynamics::MeshShape>(
      scale,
      triMesh,
      common::Uri());

  // Get TriMesh back
  auto retrievedMesh = meshShape->getTriMesh();
  ASSERT_NE(retrievedMesh, nullptr);
  EXPECT_EQ(retrievedMesh->getVertices().size(), 4);
  EXPECT_EQ(retrievedMesh->getTriangles().size(), 2);

  // Verify it's the same mesh (shared_ptr should point to same object)
  EXPECT_EQ(retrievedMesh, triMesh);
}

//==============================================================================
TEST(MeshShapeIntegration, ConvertAssimpMesh)
{
  // This tests the convertAssimpMesh static method
  // Since we can't easily create an aiScene here without assimp,
  // we test with nullptr to verify error handling

  auto triMesh = dynamics::MeshShape::convertAssimpMesh(nullptr);
  EXPECT_EQ(triMesh, nullptr);
}

//==============================================================================
TEST(MeshShapeIntegration, BackwardCompatibilityOldConstructor)
{
  // Test that old constructor still works (even though deprecated)
  // This verifies backward compatibility

  // We can't easily test with real aiScene without loading a file,
  // but we can test that the API still compiles and accepts nullptr
  const aiScene* scene = nullptr;
  Eigen::Vector3d scale(1.0, 1.0, 1.0);

  // This should compile (though it will have deprecation warnings)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  auto meshShape = std::make_shared<dynamics::MeshShape>(
      scale,
      scene,
      common::Uri("test.dae"),
      nullptr);
  #pragma GCC diagnostic pop

  ASSERT_NE(meshShape, nullptr);

  // When constructed with nullptr aiScene, getTriMesh should return nullptr
  auto triMesh = meshShape->getTriMesh();
  EXPECT_EQ(triMesh, nullptr);
}

//==============================================================================
TEST(MeshShapeIntegration, ScalePreservation)
{
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);

  // Test with non-uniform scale
  Eigen::Vector3d scale(2.0, 3.0, 4.0);
  auto meshShape = std::make_shared<dynamics::MeshShape>(
      scale,
      triMesh,
      common::Uri());

  EXPECT_EQ(meshShape->getScale(), scale);

  // Test setScale
  Eigen::Vector3d newScale(5.0, 6.0, 7.0);
  meshShape->setScale(newScale);
  EXPECT_EQ(meshShape->getScale(), newScale);

  // Test uniform scale
  meshShape->setScale(10.0);
  EXPECT_EQ(meshShape->getScale(), Eigen::Vector3d(10.0, 10.0, 10.0));
}

//==============================================================================
TEST(MeshShapeIntegration, TypeAndCopy)
{
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);

  Eigen::Vector3d scale(1.0, 1.0, 1.0);
  auto meshShape = std::make_shared<dynamics::MeshShape>(
      scale,
      triMesh,
      common::Uri("test.dae"));

  // Verify type
  EXPECT_EQ(meshShape->getType(), "MeshShape");
  EXPECT_EQ(meshShape->getType(), dynamics::MeshShape::getStaticType());

  // Test cloning (this may need adjustment based on actual implementation)
  // Clone creates a deep copy with old API, so getMesh() should work
  auto cloned = meshShape->clone();
  ASSERT_NE(cloned, nullptr);
  EXPECT_EQ(cloned->getType(), "MeshShape");
}

//==============================================================================
TEST(MeshShapeIntegration, ConstructorInitializesShapeFlags)
{
  // Bug fix: constructors must call Shape(Shape::MESH) to initialize flags.
  // Without this, getBoundingBox()/getVolume() could use uninitialized values.
  auto triMesh = std::make_shared<math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);

  auto meshShape = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0), triMesh, common::Uri("test.dae"));

  // Verify shape flags are properly initialized
  EXPECT_EQ(meshShape->getBoundingBox().getMin(), Eigen::Vector3d::Zero());
  EXPECT_EQ(meshShape->getBoundingBox().getMax(), Eigen::Vector3d(1.0, 1.0, 0.0));
  EXPECT_GT(meshShape->getVolume(), 0.0);
}

//==============================================================================
TEST(MeshShapeIntegration, SetMeshClearsCacheAndRefreshesMaterials)
{
  // Bug fix: setMesh() must clear cached aiScene to prevent stale data.
  auto triMesh1 = std::make_shared<math::TriMesh<double>>();
  triMesh1->addVertex(0.0, 0.0, 0.0);
  triMesh1->addVertex(1.0, 0.0, 0.0);
  triMesh1->addVertex(0.0, 1.0, 0.0);
  triMesh1->addTriangle(0, 1, 2);

  auto meshShape = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0), triMesh1, common::Uri("first.dae"));

  // Populate cache using deprecated API
  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene1 = meshShape->getMesh();
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(scene1, nullptr);
  EXPECT_EQ(scene1->mMeshes[0]->mNumVertices, 3u);

  // Create second mesh with 4 vertices
  auto triMesh2 = std::make_shared<math::TriMesh<double>>();
  triMesh2->addVertex(0.0, 0.0, 0.0);
  triMesh2->addVertex(2.0, 0.0, 0.0);
  triMesh2->addVertex(0.0, 2.0, 0.0);
  triMesh2->addVertex(2.0, 2.0, 0.0);
  triMesh2->addTriangle(0, 1, 2);
  triMesh2->addTriangle(1, 3, 2);

  auto meshShape2 = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0), triMesh2, common::Uri("second.dae"));

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene2 = meshShape2->getMesh();
  meshShape->setMesh(scene2, common::Uri("second.dae"), nullptr);
  const aiScene* updatedScene = meshShape->getMesh();
  DART_SUPPRESS_DEPRECATED_END

  // Verify cache was cleared and new mesh is returned
  EXPECT_EQ(meshShape->getTriMesh()->getVertices().size(), 4);
  EXPECT_EQ(updatedScene->mMeshes[0]->mNumVertices, 4u);
}
