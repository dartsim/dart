// Copyright (c) 2011, The DART development contributors

#include <dart/dynamics/box_shape.hpp>

#include <gtest/gtest.h>

#include <limits>

#include <cmath>

using namespace dart::dynamics;

//==============================================================================
TEST(BoxShapeTest, Constructor)
{
  Eigen::Vector3d size(1.0, 2.0, 3.0);
  auto box = std::make_shared<BoxShape>(size);

  EXPECT_TRUE(box->getSize().isApprox(size));
  EXPECT_EQ(box->getType(), BoxShape::getStaticType());
  EXPECT_EQ(box->getType(), "BoxShape");
}

//==============================================================================
TEST(BoxShapeTest, SetAndGetSize)
{
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));

  Eigen::Vector3d newSize(2.0, 3.0, 4.0);
  box->setSize(newSize);

  EXPECT_TRUE(box->getSize().isApprox(newSize));
}

//==============================================================================
TEST(BoxShapeTest, ComputeVolume)
{
  Eigen::Vector3d size(2.0, 3.0, 4.0);
  double expectedVolume = 2.0 * 3.0 * 4.0;

  EXPECT_DOUBLE_EQ(BoxShape::computeVolume(size), expectedVolume);

  auto box = std::make_shared<BoxShape>(size);
  EXPECT_DOUBLE_EQ(box->getVolume(), expectedVolume);
}

//==============================================================================
TEST(BoxShapeTest, ComputeInertia)
{
  Eigen::Vector3d size(2.0, 3.0, 4.0);
  double mass = 5.0;

  Eigen::Matrix3d expectedInertia = Eigen::Matrix3d::Identity();
  expectedInertia(0, 0) = mass / 12.0 * (9.0 + 16.0); // y^2 + z^2
  expectedInertia(1, 1) = mass / 12.0 * (4.0 + 16.0); // x^2 + z^2
  expectedInertia(2, 2) = mass / 12.0 * (4.0 + 9.0);  // x^2 + y^2

  Eigen::Matrix3d inertia = BoxShape::computeInertia(size, mass);
  EXPECT_TRUE(inertia.isApprox(expectedInertia));

  auto box = std::make_shared<BoxShape>(size);
  Eigen::Matrix3d instanceInertia = box->computeInertia(mass);
  EXPECT_TRUE(instanceInertia.isApprox(expectedInertia));
}

//==============================================================================
TEST(BoxShapeTest, BoundingBox)
{
  Eigen::Vector3d size(2.0, 4.0, 6.0);
  auto box = std::make_shared<BoxShape>(size);

  const auto& bbox = box->getBoundingBox();
  EXPECT_TRUE(bbox.getMin().isApprox(Eigen::Vector3d(-1.0, -2.0, -3.0)));
  EXPECT_TRUE(bbox.getMax().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

//==============================================================================
TEST(BoxShapeTest, BoundingBoxUpdatesOnSizeChange)
{
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));

  box->setSize(Eigen::Vector3d(4.0, 6.0, 8.0));

  const auto& bbox = box->getBoundingBox();
  EXPECT_TRUE(bbox.getMin().isApprox(Eigen::Vector3d(-2.0, -3.0, -4.0)));
  EXPECT_TRUE(bbox.getMax().isApprox(Eigen::Vector3d(2.0, 3.0, 4.0)));
}

//==============================================================================
TEST(BoxShapeTest, Clone)
{
  Eigen::Vector3d size(1.5, 2.5, 3.5);
  auto box = std::make_shared<BoxShape>(size);

  auto cloned = box->clone();
  auto clonedBox = std::dynamic_pointer_cast<BoxShape>(cloned);

  ASSERT_NE(clonedBox, nullptr);
  EXPECT_TRUE(clonedBox->getSize().isApprox(size));
  EXPECT_NE(clonedBox.get(), box.get());
}

//==============================================================================
TEST(BoxShapeTest, UnitCube)
{
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());

  EXPECT_DOUBLE_EQ(box->getVolume(), 1.0);

  const auto& bbox = box->getBoundingBox();
  EXPECT_TRUE(bbox.getMin().isApprox(Eigen::Vector3d(-0.5, -0.5, -0.5)));
  EXPECT_TRUE(bbox.getMax().isApprox(Eigen::Vector3d(0.5, 0.5, 0.5)));
}

//==============================================================================
TEST(BoxShapeTest, VolumeUpdatesOnSizeChange)
{
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  EXPECT_DOUBLE_EQ(box->getVolume(), 1.0);

  box->setSize(Eigen::Vector3d(2.0, 3.0, 4.0));
  EXPECT_DOUBLE_EQ(box->getVolume(), 24.0);
}

//==============================================================================
TEST(BoxShapeTest, InertiaUpdatesOnSizeChange)
{
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  const double mass = 6.0;

  box->setSize(Eigen::Vector3d(2.0, 2.0, 2.0));
  Eigen::Matrix3d inertia = box->computeInertia(mass);

  Eigen::Matrix3d expectedInertia = Eigen::Matrix3d::Identity();
  expectedInertia(0, 0) = mass / 12.0 * (4.0 + 4.0);
  expectedInertia(1, 1) = mass / 12.0 * (4.0 + 4.0);
  expectedInertia(2, 2) = mass / 12.0 * (4.0 + 4.0);

  EXPECT_TRUE(inertia.isApprox(expectedInertia));
}

//==============================================================================
TEST(BoxShapeTest, CloneIndependence)
{
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 2.0, 3.0));
  auto cloned = std::dynamic_pointer_cast<BoxShape>(box->clone());

  box->setSize(Eigen::Vector3d(5.0, 5.0, 5.0));

  EXPECT_TRUE(cloned->getSize().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(box->getSize().isApprox(Eigen::Vector3d(5.0, 5.0, 5.0)));
}

//==============================================================================
TEST(BoxShapeTest, SmallDimensions)
{
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1e-10, 1e-10, 1e-10));
  EXPECT_GT(box->getVolume(), 0.0);

  const auto& bbox = box->getBoundingBox();
  EXPECT_LT(bbox.getMin().x(), 0.0);
  EXPECT_GT(bbox.getMax().x(), 0.0);
}

//==============================================================================
TEST(BoxShapeTest, AsymmetricBox)
{
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 10.0, 100.0));

  EXPECT_DOUBLE_EQ(box->getVolume(), 1000.0);

  const auto& bbox = box->getBoundingBox();
  EXPECT_TRUE(bbox.getMin().isApprox(Eigen::Vector3d(-0.5, -5.0, -50.0)));
  EXPECT_TRUE(bbox.getMax().isApprox(Eigen::Vector3d(0.5, 5.0, 50.0)));
}
