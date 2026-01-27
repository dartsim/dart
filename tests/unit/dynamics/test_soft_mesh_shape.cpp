// Copyright (c) 2011-2025, The DART development contributors

#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/soft_body_node.hpp>
#include <dart/dynamics/soft_mesh_shape.hpp>

#include <gtest/gtest.h>

#include <cmath>

using namespace dart;
using namespace dart::dynamics;

namespace {

SoftBodyNode* createSoftBodyBox(
    SkeletonPtr& skeleton,
    const Eigen::Vector3d& size = Eigen::Vector3d::Constant(0.1),
    const Eigen::Vector3i& frags = Eigen::Vector3i(3, 3, 3),
    double totalMass = 1.0)
{
  skeleton = Skeleton::create("soft_skel");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  SoftBodyNode* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      size,
      Eigen::Isometry3d::Identity(),
      frags,
      totalMass,
      10.0,
      10.0,
      0.01);

  return softBody;
}

} // namespace

TEST(SoftMeshShapeTest, Constructor)
{
  SkeletonPtr skeleton;
  SoftBodyNode* softBody = createSoftBodyBox(skeleton);
  ASSERT_NE(softBody, nullptr);

  auto softMeshShape = std::make_shared<SoftMeshShape>(softBody);
  ASSERT_NE(softMeshShape, nullptr);
  EXPECT_EQ(softMeshShape->getType(), "SoftMeshShape");
}

TEST(SoftMeshShapeTest, GetSoftBodyNode)
{
  SkeletonPtr skeleton;
  SoftBodyNode* softBody = createSoftBodyBox(skeleton);
  ASSERT_NE(softBody, nullptr);

  auto softMeshShape = std::make_shared<SoftMeshShape>(softBody);

  const SoftBodyNode* retrievedNode = softMeshShape->getSoftBodyNode();
  EXPECT_EQ(retrievedNode, softBody);
}

TEST(SoftMeshShapeTest, GetType)
{
  SkeletonPtr skeleton;
  SoftBodyNode* softBody = createSoftBodyBox(skeleton);
  ASSERT_NE(softBody, nullptr);

  auto softMeshShape = std::make_shared<SoftMeshShape>(softBody);

  EXPECT_EQ(softMeshShape->getType(), "SoftMeshShape");
  EXPECT_EQ(softMeshShape->getType(), SoftMeshShape::getStaticType());
}

TEST(SoftMeshShapeTest, GetTriMesh)
{
  SkeletonPtr skeleton;
  SoftBodyNode* softBody = createSoftBodyBox(skeleton);
  ASSERT_NE(softBody, nullptr);

  auto softMeshShape = std::make_shared<SoftMeshShape>(softBody);

  auto triMesh = softMeshShape->getTriMesh();
  ASSERT_NE(triMesh, nullptr);

  const auto& vertices = triMesh->getVertices();
  EXPECT_EQ(vertices.size(), softBody->getNumPointMasses());

  const auto& triangles = triMesh->getTriangles();
  EXPECT_EQ(triangles.size(), softBody->getNumFaces());

  for (const auto& vertex : vertices) {
    EXPECT_TRUE(vertex.array().isFinite().all());
  }
}

TEST(SoftMeshShapeTest, Update)
{
  SkeletonPtr skeleton;
  SoftBodyNode* softBody = createSoftBodyBox(skeleton);
  ASSERT_NE(softBody, nullptr);

  auto softMeshShape = std::make_shared<SoftMeshShape>(softBody);

  auto triMesh = softMeshShape->getTriMesh();
  ASSERT_NE(triMesh, nullptr);
  ASSERT_GT(triMesh->getVertices().size(), 0u);

  PointMass* pointMass = softBody->getPointMass(0);
  ASSERT_NE(pointMass, nullptr);

  Eigen::Vector3d restingPos = pointMass->getRestingPosition();
  Eigen::Vector3d displacement(1.0, 2.0, 3.0);
  pointMass->setPositions(displacement);

  softMeshShape->update();

  const auto& updatedVertices = triMesh->getVertices();
  Eigen::Vector3d expectedPos = restingPos + displacement;
  EXPECT_TRUE(updatedVertices[0].isApprox(expectedPos, 1e-10));
}

TEST(SoftMeshShapeTest, ComputeInertia)
{
  SkeletonPtr skeleton;
  SoftBodyNode* softBody = createSoftBodyBox(skeleton);
  ASSERT_NE(softBody, nullptr);

  auto softMeshShape = std::make_shared<SoftMeshShape>(softBody);

  Eigen::Matrix3d inertia = softMeshShape->computeInertia(1.0);
  EXPECT_TRUE(inertia.isApprox(Eigen::Matrix3d::Zero()));

  Eigen::Matrix3d inertia2 = softMeshShape->computeInertia(5.0);
  EXPECT_TRUE(inertia2.isApprox(Eigen::Matrix3d::Zero()));
}

TEST(SoftMeshShapeTest, Clone)
{
  SkeletonPtr skeleton;
  SoftBodyNode* softBody = createSoftBodyBox(skeleton);
  ASSERT_NE(softBody, nullptr);

  auto softMeshShape = std::make_shared<SoftMeshShape>(softBody);

  ShapePtr cloned = softMeshShape->clone();
  EXPECT_EQ(cloned, nullptr);
}

TEST(SoftMeshShapeTest, BoundingBox)
{
  Eigen::Vector3d size(1.0, 2.0, 3.0);
  SkeletonPtr skeleton;
  SoftBodyNode* softBody = createSoftBodyBox(skeleton, size);
  ASSERT_NE(softBody, nullptr);

  auto softMeshShape = std::make_shared<SoftMeshShape>(softBody);

  const auto& bbox = softMeshShape->getBoundingBox();

  EXPECT_TRUE(bbox.getMin().isApprox(Eigen::Vector3d::Zero(), 1e-10));
  EXPECT_TRUE(bbox.getMax().isApprox(Eigen::Vector3d::Zero(), 1e-10));
}

TEST(SoftMeshShapeTest, TriMeshVerticesMatchPointMasses)
{
  SkeletonPtr skeleton;
  SoftBodyNode* softBody = createSoftBodyBox(skeleton);
  ASSERT_NE(softBody, nullptr);

  auto softMeshShape = std::make_shared<SoftMeshShape>(softBody);
  auto triMesh = softMeshShape->getTriMesh();
  ASSERT_NE(triMesh, nullptr);

  const auto& vertices = triMesh->getVertices();

  for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i) {
    PointMass* pm = softBody->getPointMass(i);
    Eigen::Vector3d restingPos = pm->getRestingPosition();
    EXPECT_TRUE(vertices[i].isApprox(restingPos, 1e-10))
        << "Vertex " << i << " mismatch: expected " << restingPos.transpose()
        << ", got " << vertices[i].transpose();
  }
}

TEST(SoftMeshShapeTest, TriMeshTrianglesMatchFaces)
{
  SkeletonPtr skeleton;
  SoftBodyNode* softBody = createSoftBodyBox(skeleton);
  ASSERT_NE(softBody, nullptr);

  auto softMeshShape = std::make_shared<SoftMeshShape>(softBody);
  auto triMesh = softMeshShape->getTriMesh();
  ASSERT_NE(triMesh, nullptr);

  const auto& triangles = triMesh->getTriangles();

  for (std::size_t i = 0; i < softBody->getNumFaces(); ++i) {
    Eigen::Vector3i face = softBody->getFace(i);
    const auto& triangle = triangles[i];

    EXPECT_EQ(triangle[0], static_cast<std::size_t>(face[0]));
    EXPECT_EQ(triangle[1], static_cast<std::size_t>(face[1]));
    EXPECT_EQ(triangle[2], static_cast<std::size_t>(face[2]));
  }
}

TEST(SoftMeshShapeTest, DynamicVariance)
{
  SkeletonPtr skeleton;
  SoftBodyNode* softBody = createSoftBodyBox(skeleton);
  ASSERT_NE(softBody, nullptr);

  auto softMeshShape = std::make_shared<SoftMeshShape>(softBody);

  EXPECT_TRUE(
      (softMeshShape->checkDataVariance(Shape::DYNAMIC_VERTICES) != 0u));
}

TEST(SoftMeshShapeTest, GetAssimpMesh)
{
  SkeletonPtr skeleton;
  SoftBodyNode* softBody = createSoftBodyBox(skeleton);
  ASSERT_NE(softBody, nullptr);

  auto softMeshShape = std::make_shared<SoftMeshShape>(softBody);

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiMesh* assimpMesh = softMeshShape->getAssimpMesh();
  DART_SUPPRESS_DEPRECATED_END

  ASSERT_NE(assimpMesh, nullptr);

  EXPECT_EQ(assimpMesh->mNumVertices, softBody->getNumPointMasses());
  EXPECT_EQ(assimpMesh->mNumFaces, softBody->getNumFaces());

  ASSERT_NE(assimpMesh->mVertices, nullptr);
  ASSERT_NE(assimpMesh->mNormals, nullptr);

  ASSERT_NE(assimpMesh->mFaces, nullptr);
  for (unsigned int i = 0; i < assimpMesh->mNumFaces; ++i) {
    EXPECT_EQ(assimpMesh->mFaces[i].mNumIndices, 3u);
    ASSERT_NE(assimpMesh->mFaces[i].mIndices, nullptr);
  }
}

TEST(SoftMeshShapeTest, UpdateSyncsAssimpMesh)
{
  SkeletonPtr skeleton;
  SoftBodyNode* softBody = createSoftBodyBox(skeleton);
  ASSERT_NE(softBody, nullptr);

  auto softMeshShape = std::make_shared<SoftMeshShape>(softBody);

  PointMass* pointMass = softBody->getPointMass(0);
  ASSERT_NE(pointMass, nullptr);

  Eigen::Vector3d restingPos = pointMass->getRestingPosition();
  Eigen::Vector3d displacement(10.0, 20.0, 30.0);
  pointMass->setPositions(displacement);

  softMeshShape->update();

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiMesh* assimpMesh = softMeshShape->getAssimpMesh();
  DART_SUPPRESS_DEPRECATED_END

  ASSERT_NE(assimpMesh, nullptr);
  ASSERT_NE(assimpMesh->mVertices, nullptr);

  Eigen::Vector3d expectedPos = restingPos + displacement;
  EXPECT_NEAR(assimpMesh->mVertices[0].x, expectedPos[0], 1e-5);
  EXPECT_NEAR(assimpMesh->mVertices[0].y, expectedPos[1], 1e-5);
  EXPECT_NEAR(assimpMesh->mVertices[0].z, expectedPos[2], 1e-5);
}

TEST(SoftMeshShapeTest, MultiplePointMassUpdates)
{
  SkeletonPtr skeleton;
  SoftBodyNode* softBody = createSoftBodyBox(skeleton);
  ASSERT_NE(softBody, nullptr);

  auto softMeshShape = std::make_shared<SoftMeshShape>(softBody);
  auto triMesh = softMeshShape->getTriMesh();
  ASSERT_NE(triMesh, nullptr);

  std::vector<Eigen::Vector3d> expectedPositions;
  for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i) {
    PointMass* pm = softBody->getPointMass(i);
    Eigen::Vector3d restingPos = pm->getRestingPosition();
    Eigen::Vector3d displacement(
        static_cast<double>(i),
        static_cast<double>(i * 2),
        static_cast<double>(i * 3));
    pm->setPositions(displacement);
    expectedPositions.push_back(restingPos + displacement);
  }

  softMeshShape->update();

  const auto& vertices = triMesh->getVertices();
  for (std::size_t i = 0; i < expectedPositions.size(); ++i) {
    EXPECT_TRUE(vertices[i].isApprox(expectedPositions[i], 1e-10))
        << "Vertex " << i << " not updated correctly";
  }
}
