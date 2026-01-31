// Copyright (c) 2011-2025, The DART development contributors

#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/point_mass.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/soft_body_node.hpp>

#include <gtest/gtest.h>

#include <cmath>

using namespace dart;
using namespace dart::dynamics;

namespace {

SoftBodyNode* createSoftBody(const SkeletonPtr& skeleton)
{
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  return pair.second;
}

} // namespace

TEST(SoftBodyNodeUniqueProperties, ConnectPointMassesAndFaces)
{
  SoftBodyNode::UniqueProperties properties(10.0, 10.0, 0.1);

  EXPECT_FALSE(properties.connectPointMasses(0, 1));

  properties.addPointMass(PointMass::Properties(Eigen::Vector3d::Zero(), 0.5));
  properties.addPointMass(PointMass::Properties(Eigen::Vector3d::UnitX(), 0.5));
  EXPECT_TRUE(properties.connectPointMasses(0, 1));
  EXPECT_FALSE(properties.connectPointMasses(0, 3));

  properties.addPointMass(PointMass::Properties(Eigen::Vector3d::UnitY(), 0.5));
  properties.addFace(Eigen::Vector3i(0, 1, 2));
  EXPECT_EQ(properties.mFaces.size(), 1u);
}

TEST(SoftBodyNodeHelper, MakeBoxProperties)
{
  const Eigen::Vector3d size(1.0, 2.0, 3.0);
  const auto transform = Eigen::Isometry3d::Identity();

  const auto props = SoftBodyNodeHelper::makeBoxProperties(
      size, transform, 4.0, 20.0, 30.0, 0.2);
  EXPECT_EQ(props.mPointProps.size(), 8u);
  EXPECT_EQ(props.mFaces.size(), 12u);

  const Eigen::Vector3i frags(2, 2, 2);
  const auto fragProps = SoftBodyNodeHelper::makeBoxProperties(
      size, transform, frags, 4.0, 20.0, 30.0, 0.2);
  EXPECT_EQ(fragProps.mPointProps.size(), 26u);
  EXPECT_FALSE(fragProps.mFaces.empty());
}

TEST(SoftBodyNodeHelper, MakeEllipsoidAndCylinderProperties)
{
  const auto single
      = SoftBodyNodeHelper::makeSinglePointMassProperties(1.0, 5.0, 6.0, 0.1);
  EXPECT_EQ(single.mPointProps.size(), 1u);
  EXPECT_TRUE(single.mFaces.empty());

  const auto sphere = SoftBodyNodeHelper::makeSphereProperties(
      0.5, 4, 3, 2.0, 10.0, 11.0, 0.2);
  EXPECT_GT(sphere.mPointProps.size(), 0u);
  EXPECT_GT(sphere.mFaces.size(), 0u);

  const auto ellipsoid = SoftBodyNodeHelper::makeEllipsoidProperties(
      Eigen::Vector3d(1.0, 2.0, 3.0), 4, 3, 2.0, 10.0, 11.0, 0.2);
  EXPECT_EQ(ellipsoid.mPointProps.size(), 10u);
  EXPECT_GT(ellipsoid.mFaces.size(), 0u);

  const auto cylinder = SoftBodyNodeHelper::makeCylinderProperties(
      1.0, 1.0, 4, 2, 2, 3.0, 10.0, 11.0, 0.2);
  EXPECT_EQ(cylinder.mPointProps.size(), 22u);
  EXPECT_GT(cylinder.mFaces.size(), 0u);
}

TEST(SoftBodyNodeHelper, SettersAndPointMassOperations)
{
  auto skeleton = Skeleton::create("soft-body-test");
  auto* softBody = createSoftBody(skeleton);

  SoftBodyNodeHelper::setSinglePointMass(softBody, 1.0, 5.0, 6.0, 0.1);
  EXPECT_EQ(softBody->getNumPointMasses(), 1u);
  auto* singlePm = softBody->getPointMass(0);
  ASSERT_NE(singlePm, nullptr);
  const auto bodyJac = singlePm->getBodyJacobian();
  EXPECT_EQ(bodyJac.rows(), 3);
  EXPECT_EQ(bodyJac.cols(), softBody->getNumDependentGenCoords() + 3);

  const auto worldJac = singlePm->getWorldJacobian();
  EXPECT_EQ(worldJac.rows(), 3);
  EXPECT_EQ(worldJac.cols(), softBody->getNumDependentGenCoords() + 3);

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      Eigen::Isometry3d::Identity(),
      2.0,
      5.0,
      6.0,
      0.1);
  EXPECT_EQ(softBody->getNumPointMasses(), 8u);

  SoftBodyNodeHelper::setEllipsoid(
      softBody, Eigen::Vector3d(1.0, 1.0, 1.0), 4, 3, 2.0, 5.0, 6.0, 0.1);
  EXPECT_EQ(softBody->getNumPointMasses(), 10u);

  SoftBodyNodeHelper::setCylinder(
      softBody, 0.5, 1.0, 4, 2, 2, 3.0, 5.0, 6.0, 0.1);
  EXPECT_EQ(softBody->getNumPointMasses(), 22u);

  softBody->setVertexSpringStiffness(12.0);
  softBody->setEdgeSpringStiffness(13.0);
  EXPECT_DOUBLE_EQ(softBody->getVertexSpringStiffness(), 12.0);
  EXPECT_DOUBLE_EQ(softBody->getEdgeSpringStiffness(), 13.0);

  softBody->setDampingCoefficient(-1.0);
  EXPECT_TRUE(std::isfinite(softBody->getDampingCoefficient()));

  auto opSkeleton = Skeleton::create("soft-body-ops");
  auto* opBody = createSoftBody(opSkeleton);
  SoftBodyNodeHelper::setSinglePointMass(opBody, 1.0, 5.0, 6.0, 0.1);
  EXPECT_EQ(opBody->getNumPointMasses(), 1u);

  PointMass::Properties pointProps(Eigen::Vector3d(0.1, 0.2, 0.3), 0.4);
  opBody->addPointMass(pointProps);
  opBody->addPointMass(pointProps);
  const auto prevFaces = opBody->getNumFaces();
  opBody->addFace(Eigen::Vector3i(0, 1, 2));
  EXPECT_EQ(opBody->getNumFaces(), prevFaces + 1u);

  auto* pm0 = opBody->getPointMass(0);
  auto* pm1 = opBody->getPointMass(1);
  ASSERT_NE(pm0, nullptr);
  ASSERT_NE(pm1, nullptr);

  pm0->setMass(0.25);
  EXPECT_DOUBLE_EQ(pm0->getMass(), 0.25);

  pm0->setPositions(Eigen::Vector3d(0.2, 0.3, 0.4));
  EXPECT_TRUE(pm0->getPositions().isApprox(Eigen::Vector3d(0.2, 0.3, 0.4)));

  pm0->setVelocities(Eigen::Vector3d(0.4, 0.5, 0.6));
  EXPECT_TRUE(pm0->getVelocities().isApprox(Eigen::Vector3d(0.4, 0.5, 0.6)));

  pm0->setAccelerations(Eigen::Vector3d(0.7, 0.8, 0.9));
  EXPECT_TRUE(pm0->getAccelerations().isApprox(Eigen::Vector3d(0.7, 0.8, 0.9)));

  pm0->setForces(Eigen::Vector3d(1.0, 2.0, 3.0));
  EXPECT_TRUE(pm0->getForces().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

  pm0->setColliding(true);
  EXPECT_TRUE(pm0->isColliding());

  opBody->connectPointMasses(0, 1);
  EXPECT_EQ(pm0->getNumConnectedPointMasses(), 1u);
  EXPECT_EQ(pm0->getConnectedPointMass(0), pm1);

  opBody->removeAllPointMasses();
  EXPECT_EQ(opBody->getNumPointMasses(), 0u);
}

TEST(SoftBodyNodeHelper, MakeBoxPropertiesWithLargeFrags)
{
  const Eigen::Vector3d size(1.0, 2.0, 3.0);
  const auto transform = Eigen::Isometry3d::Identity();

  const Eigen::Vector3i frags3(3, 3, 3);
  const auto props3 = SoftBodyNodeHelper::makeBoxProperties(
      size, transform, frags3, 4.0, 20.0, 30.0, 0.2);
  EXPECT_GE(props3.mPointProps.size(), 26u);
  EXPECT_GE(props3.mFaces.size(), 12u);

  const Eigen::Vector3i frags4(4, 4, 4);
  const auto props4 = SoftBodyNodeHelper::makeBoxProperties(
      size, transform, frags4, 4.0, 20.0, 30.0, 0.2);
  EXPECT_GT(props4.mPointProps.size(), props3.mPointProps.size());
  EXPECT_GT(props4.mFaces.size(), props3.mFaces.size());

  const Eigen::Vector3i fragsAsym(3, 4, 5);
  const auto propsAsym = SoftBodyNodeHelper::makeBoxProperties(
      size, transform, fragsAsym, 4.0, 20.0, 30.0, 0.2);
  EXPECT_GT(propsAsym.mPointProps.size(), 0u);
  EXPECT_GT(propsAsym.mFaces.size(), 0u);
}

TEST(SoftBodyNodeHelper, SetBoxWithFragsOnSoftBody)
{
  auto skeleton = Skeleton::create("frag-box");
  auto* softBody = createSoftBody(skeleton);

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      Eigen::Isometry3d::Identity(),
      Eigen::Vector3i(3, 3, 3),
      2.0,
      5.0,
      6.0,
      0.1);
  EXPECT_GT(softBody->getNumPointMasses(), 8u);
}

TEST(SoftBodyNodeHelper, MakeEllipsoidWithMoreSlices)
{
  const auto ellip = SoftBodyNodeHelper::makeEllipsoidProperties(
      Eigen::Vector3d(1.0, 1.5, 2.0), 8, 6, 3.0, 10.0, 11.0, 0.2);
  EXPECT_GT(ellip.mPointProps.size(), 10u);
  EXPECT_GT(ellip.mFaces.size(), 0u);
}

TEST(SoftBodyNodeHelper, MakeCylinderWithMoreRings)
{
  const auto cyl = SoftBodyNodeHelper::makeCylinderProperties(
      0.5, 2.0, 8, 4, 3, 3.0, 10.0, 11.0, 0.2);
  EXPECT_GT(cyl.mPointProps.size(), 22u);
  EXPECT_GT(cyl.mFaces.size(), 0u);
}

TEST(SoftBodyNodeHelper, ConstructorWithSpans)
{
  std::vector<PointMass::Properties> points;
  points.push_back(PointMass::Properties(Eigen::Vector3d::Zero(), 0.5));
  points.push_back(PointMass::Properties(Eigen::Vector3d::UnitX(), 0.5));
  points.push_back(PointMass::Properties(Eigen::Vector3d::UnitY(), 0.5));

  std::vector<Eigen::Vector3i> faces;
  faces.push_back(Eigen::Vector3i(0, 1, 2));

  SoftBodyNode::UniqueProperties props(10.0, 10.0, 0.1, points, faces);
  EXPECT_EQ(props.mPointProps.size(), 3u);
  EXPECT_EQ(props.mFaces.size(), 1u);
}
