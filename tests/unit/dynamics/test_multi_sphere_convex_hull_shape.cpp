// Copyright (c) 2011-2025, The DART development contributors

#include <dart/dynamics/multi_sphere_convex_hull_shape.hpp>

#include <gtest/gtest.h>

using namespace dart::dynamics;

//==============================================================================
TEST(MultiSphereConvexHullShapeTest, DefaultConstructor)
{
  MultiSphereConvexHullShape::Spheres emptySpheres;
  auto shape = std::make_shared<MultiSphereConvexHullShape>(emptySpheres);

  EXPECT_EQ(shape->getNumSpheres(), 0u);
  EXPECT_TRUE(shape->getSpheres().empty());
  EXPECT_EQ(shape->getType(), MultiSphereConvexHullShape::getStaticType());
  EXPECT_EQ(shape->getType(), "MultiSphereConvexHullShape");
}

//==============================================================================
TEST(MultiSphereConvexHullShapeTest, ConstructorWithSpheres)
{
  MultiSphereConvexHullShape::Spheres spheres;
  spheres.emplace_back(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));
  spheres.emplace_back(0.5, Eigen::Vector3d(2.0, 0.0, 0.0));
  spheres.emplace_back(0.75, Eigen::Vector3d(1.0, 1.0, 0.0));

  auto shape = std::make_shared<MultiSphereConvexHullShape>(spheres);

  EXPECT_EQ(shape->getNumSpheres(), 3u);
  EXPECT_EQ(shape->getSpheres().size(), 3u);

  const auto& storedSpheres = shape->getSpheres();
  EXPECT_DOUBLE_EQ(storedSpheres[0].first, 1.0);
  EXPECT_TRUE(storedSpheres[0].second.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_DOUBLE_EQ(storedSpheres[1].first, 0.5);
  EXPECT_TRUE(storedSpheres[1].second.isApprox(Eigen::Vector3d(2.0, 0.0, 0.0)));
  EXPECT_DOUBLE_EQ(storedSpheres[2].first, 0.75);
  EXPECT_TRUE(storedSpheres[2].second.isApprox(Eigen::Vector3d(1.0, 1.0, 0.0)));
}

//==============================================================================
TEST(MultiSphereConvexHullShapeTest, AddSphere)
{
  MultiSphereConvexHullShape::Spheres emptySpheres;
  auto shape = std::make_shared<MultiSphereConvexHullShape>(emptySpheres);

  EXPECT_EQ(shape->getNumSpheres(), 0u);

  shape->addSphere(std::make_pair(1.0, Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_EQ(shape->getNumSpheres(), 1u);

  shape->addSphere(0.5, Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_EQ(shape->getNumSpheres(), 2u);

  const auto& storedSpheres = shape->getSpheres();
  EXPECT_DOUBLE_EQ(storedSpheres[0].first, 1.0);
  EXPECT_TRUE(storedSpheres[0].second.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_DOUBLE_EQ(storedSpheres[1].first, 0.5);
  EXPECT_TRUE(storedSpheres[1].second.isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
}

//==============================================================================
TEST(MultiSphereConvexHullShapeTest, AddSpheres)
{
  MultiSphereConvexHullShape::Spheres initialSpheres;
  initialSpheres.emplace_back(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));

  auto shape = std::make_shared<MultiSphereConvexHullShape>(initialSpheres);
  EXPECT_EQ(shape->getNumSpheres(), 1u);

  MultiSphereConvexHullShape::Spheres moreSpheres;
  moreSpheres.emplace_back(0.5, Eigen::Vector3d(2.0, 0.0, 0.0));
  moreSpheres.emplace_back(0.75, Eigen::Vector3d(0.0, 2.0, 0.0));
  moreSpheres.emplace_back(0.25, Eigen::Vector3d(0.0, 0.0, 2.0));

  shape->addSpheres(moreSpheres);
  EXPECT_EQ(shape->getNumSpheres(), 4u);

  const auto& storedSpheres = shape->getSpheres();
  EXPECT_DOUBLE_EQ(storedSpheres[0].first, 1.0);
  EXPECT_DOUBLE_EQ(storedSpheres[1].first, 0.5);
  EXPECT_DOUBLE_EQ(storedSpheres[2].first, 0.75);
  EXPECT_DOUBLE_EQ(storedSpheres[3].first, 0.25);
}

//==============================================================================
TEST(MultiSphereConvexHullShapeTest, RemoveAllSpheres)
{
  MultiSphereConvexHullShape::Spheres spheres;
  spheres.emplace_back(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));
  spheres.emplace_back(0.5, Eigen::Vector3d(2.0, 0.0, 0.0));
  spheres.emplace_back(0.75, Eigen::Vector3d(1.0, 1.0, 0.0));

  auto shape = std::make_shared<MultiSphereConvexHullShape>(spheres);
  EXPECT_EQ(shape->getNumSpheres(), 3u);

  shape->removeAllSpheres();

  EXPECT_EQ(shape->getNumSpheres(), 0u);
  EXPECT_TRUE(shape->getSpheres().empty());
}

//==============================================================================
TEST(MultiSphereConvexHullShapeTest, BoundingBox)
{
  // Sphere 1: center (0,0,0), radius 1.0 -> bounds (-1,-1,-1) to (1,1,1)
  // Sphere 2: center (3,0,0), radius 0.5 -> bounds (2.5,-0.5,-0.5) to
  // (3.5,0.5,0.5) Sphere 3: center (0,2,0), radius 0.5 -> bounds
  // (-0.5,1.5,-0.5) to (0.5,2.5,0.5) Combined bounding box: min=(-1,-1,-1),
  // max=(3.5,2.5,1)
  MultiSphereConvexHullShape::Spheres spheres;
  spheres.emplace_back(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));
  spheres.emplace_back(0.5, Eigen::Vector3d(3.0, 0.0, 0.0));
  spheres.emplace_back(0.5, Eigen::Vector3d(0.0, 2.0, 0.0));

  auto shape = std::make_shared<MultiSphereConvexHullShape>(spheres);

  const auto& bbox = shape->getBoundingBox();
  EXPECT_TRUE(bbox.getMin().isApprox(Eigen::Vector3d(-1.0, -1.0, -1.0)));
  EXPECT_TRUE(bbox.getMax().isApprox(Eigen::Vector3d(3.5, 2.5, 1.0)));
}

//==============================================================================
TEST(MultiSphereConvexHullShapeTest, BoundingBoxUpdatesOnAddSphere)
{
  MultiSphereConvexHullShape::Spheres spheres;
  spheres.emplace_back(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));

  auto shape = std::make_shared<MultiSphereConvexHullShape>(spheres);

  const auto& bbox1 = shape->getBoundingBox();
  EXPECT_TRUE(bbox1.getMin().isApprox(Eigen::Vector3d(-1.0, -1.0, -1.0)));
  EXPECT_TRUE(bbox1.getMax().isApprox(Eigen::Vector3d(1.0, 1.0, 1.0)));

  shape->addSphere(0.5, Eigen::Vector3d(5.0, 0.0, 0.0));

  const auto& bbox2 = shape->getBoundingBox();
  EXPECT_TRUE(bbox2.getMin().isApprox(Eigen::Vector3d(-1.0, -1.0, -1.0)));
  EXPECT_TRUE(bbox2.getMax().isApprox(Eigen::Vector3d(5.5, 1.0, 1.0)));
}

//==============================================================================
TEST(MultiSphereConvexHullShapeTest, ComputeVolume)
{
  MultiSphereConvexHullShape::Spheres spheres;
  spheres.emplace_back(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));

  auto shape = std::make_shared<MultiSphereConvexHullShape>(spheres);

  shape->getBoundingBox();

  double volume = shape->getVolume();
  EXPECT_GT(volume, 0.0);
  EXPECT_DOUBLE_EQ(volume, 8.0);
}

//==============================================================================
TEST(MultiSphereConvexHullShapeTest, ComputeVolumeMultipleSpheres)
{
  MultiSphereConvexHullShape::Spheres spheres;
  spheres.emplace_back(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));
  spheres.emplace_back(0.5, Eigen::Vector3d(3.0, 0.0, 0.0));

  auto shape = std::make_shared<MultiSphereConvexHullShape>(spheres);

  shape->getBoundingBox();

  double volume = shape->getVolume();
  EXPECT_GT(volume, 0.0);
  EXPECT_DOUBLE_EQ(volume, 18.0);
}

//==============================================================================
TEST(MultiSphereConvexHullShapeTest, ComputeInertia)
{
  MultiSphereConvexHullShape::Spheres spheres;
  spheres.emplace_back(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));

  auto shape = std::make_shared<MultiSphereConvexHullShape>(spheres);

  double mass = 5.0;
  Eigen::Matrix3d inertia = shape->computeInertia(mass);

  EXPECT_EQ(inertia.rows(), 3);
  EXPECT_EQ(inertia.cols(), 3);

  EXPECT_GT(inertia(0, 0), 0.0);
  EXPECT_GT(inertia(1, 1), 0.0);
  EXPECT_GT(inertia(2, 2), 0.0);

  EXPECT_DOUBLE_EQ(inertia(0, 1), 0.0);
  EXPECT_DOUBLE_EQ(inertia(0, 2), 0.0);
  EXPECT_DOUBLE_EQ(inertia(1, 0), 0.0);
  EXPECT_DOUBLE_EQ(inertia(1, 2), 0.0);
  EXPECT_DOUBLE_EQ(inertia(2, 0), 0.0);
  EXPECT_DOUBLE_EQ(inertia(2, 1), 0.0);

  // Box inertia formula: I_xx = m/12 * (y^2 + z^2)
  // For 2x2x2 cube: I_xx = 5/12 * (4 + 4) = 10/3
  double expectedDiagonal = mass / 12.0 * (4.0 + 4.0);
  EXPECT_DOUBLE_EQ(inertia(0, 0), expectedDiagonal);
  EXPECT_DOUBLE_EQ(inertia(1, 1), expectedDiagonal);
  EXPECT_DOUBLE_EQ(inertia(2, 2), expectedDiagonal);
}

//==============================================================================
TEST(MultiSphereConvexHullShapeTest, ComputeInertiaAsymmetric)
{
  // Bounding box: min=(-1,-1,-1), max=(4.5,1,1) -> size 5.5x2x2
  MultiSphereConvexHullShape::Spheres spheres;
  spheres.emplace_back(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));
  spheres.emplace_back(0.5, Eigen::Vector3d(4.0, 0.0, 0.0));

  auto shape = std::make_shared<MultiSphereConvexHullShape>(spheres);

  double mass = 10.0;
  Eigen::Matrix3d inertia = shape->computeInertia(mass);

  double sizeX = 5.5;
  double sizeY = 2.0;
  double sizeZ = 2.0;

  double expectedIxx = mass / 12.0 * (sizeY * sizeY + sizeZ * sizeZ);
  double expectedIyy = mass / 12.0 * (sizeX * sizeX + sizeZ * sizeZ);
  double expectedIzz = mass / 12.0 * (sizeX * sizeX + sizeY * sizeY);

  EXPECT_DOUBLE_EQ(inertia(0, 0), expectedIxx);
  EXPECT_DOUBLE_EQ(inertia(1, 1), expectedIyy);
  EXPECT_DOUBLE_EQ(inertia(2, 2), expectedIzz);

  EXPECT_GT(inertia(1, 1), inertia(0, 0));
  EXPECT_GT(inertia(2, 2), inertia(0, 0));
}

//==============================================================================
TEST(MultiSphereConvexHullShapeTest, Clone)
{
  MultiSphereConvexHullShape::Spheres spheres;
  spheres.emplace_back(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));
  spheres.emplace_back(0.5, Eigen::Vector3d(2.0, 0.0, 0.0));
  spheres.emplace_back(0.75, Eigen::Vector3d(1.0, 1.0, 0.0));

  auto shape = std::make_shared<MultiSphereConvexHullShape>(spheres);

  auto cloned = shape->clone();
  auto clonedShape
      = std::dynamic_pointer_cast<MultiSphereConvexHullShape>(cloned);

  ASSERT_NE(clonedShape, nullptr);
  EXPECT_NE(clonedShape.get(), shape.get());
  EXPECT_EQ(clonedShape->getNumSpheres(), shape->getNumSpheres());

  const auto& originalSpheres = shape->getSpheres();
  const auto& clonedSpheres = clonedShape->getSpheres();

  for (std::size_t i = 0; i < originalSpheres.size(); ++i) {
    EXPECT_DOUBLE_EQ(clonedSpheres[i].first, originalSpheres[i].first);
    EXPECT_TRUE(clonedSpheres[i].second.isApprox(originalSpheres[i].second));
  }
}

//==============================================================================
TEST(MultiSphereConvexHullShapeTest, CloneIsIndependent)
{
  MultiSphereConvexHullShape::Spheres spheres;
  spheres.emplace_back(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));

  auto shape = std::make_shared<MultiSphereConvexHullShape>(spheres);

  auto cloned = shape->clone();
  auto clonedShape
      = std::dynamic_pointer_cast<MultiSphereConvexHullShape>(cloned);

  ASSERT_NE(clonedShape, nullptr);

  shape->addSphere(0.5, Eigen::Vector3d(5.0, 0.0, 0.0));

  EXPECT_EQ(shape->getNumSpheres(), 2u);
  EXPECT_EQ(clonedShape->getNumSpheres(), 1u);
}

//==============================================================================
TEST(MultiSphereConvexHullShapeTest, SingleSphere)
{
  // Sphere at (1,2,3) with radius 2 -> bounds (-1,0,1) to (3,4,5) -> volume 64
  MultiSphereConvexHullShape::Spheres spheres;
  spheres.emplace_back(2.0, Eigen::Vector3d(1.0, 2.0, 3.0));

  auto shape = std::make_shared<MultiSphereConvexHullShape>(spheres);

  EXPECT_EQ(shape->getNumSpheres(), 1u);

  const auto& bbox = shape->getBoundingBox();
  EXPECT_TRUE(bbox.getMin().isApprox(Eigen::Vector3d(-1.0, 0.0, 1.0)));
  EXPECT_TRUE(bbox.getMax().isApprox(Eigen::Vector3d(3.0, 4.0, 5.0)));

  EXPECT_DOUBLE_EQ(shape->getVolume(), 64.0);
}
