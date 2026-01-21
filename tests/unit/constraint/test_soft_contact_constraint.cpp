// Copyright (c) 2011-2025, The DART development contributors

#include <dart/constraint/SoftContactConstraint.hpp>

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/Contact.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SoftBodyNode.hpp>
#include <dart/dynamics/SoftMeshShape.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <cmath>

using namespace dart;

namespace dart::test {
namespace {

class TestCollisionObject final : public collision::CollisionObject
{
public:
  TestCollisionObject(
      collision::CollisionDetector* collisionDetector,
      const dynamics::ShapeFrame* shapeFrame)
    : collision::CollisionObject(collisionDetector, shapeFrame)
  {
  }

private:
  void updateEngineData() override {}
};

class ExposedSoftContactConstraint final
  : public constraint::SoftContactConstraint
{
public:
  using SoftContactConstraint::applyImpulse;
  using SoftContactConstraint::applyUnitImpulse;
  using SoftContactConstraint::computeFrictionCoefficient;
  using SoftContactConstraint::computeRestitutionCoefficient;
  using SoftContactConstraint::excite;
  using SoftContactConstraint::getInformation;
  using SoftContactConstraint::getVelocityChange;
  using SoftContactConstraint::isActive;
  using SoftContactConstraint::SoftContactConstraint;
  using SoftContactConstraint::unexcite;
  using SoftContactConstraint::update;
};

struct ContactFixture
{
  std::shared_ptr<collision::CollisionDetector> detector;
  dynamics::SkeletonPtr softSkel;
  dynamics::SoftBodyNode* softBody{nullptr};
  dynamics::ShapeNode* softShapeNode{nullptr};
  dynamics::SkeletonPtr rigidSkel;
  dynamics::BodyNode* rigidBody{nullptr};
  dynamics::ShapeNode* rigidShapeNode{nullptr};
};

ContactFixture makeFixture(double friction, double restitution)
{
  ContactFixture fixture;
  fixture.detector = collision::DARTCollisionDetector::create();
  fixture.softSkel = dynamics::Skeleton::create("soft");
  auto softPair = fixture.softSkel->createJointAndBodyNodePair<
      dynamics::FreeJoint,
      dynamics::SoftBodyNode>();
  fixture.softBody = softPair.second;
  dynamics::SoftBodyNodeHelper::setBox(
      fixture.softBody,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      Eigen::Isometry3d::Identity(),
      1.0,
      10.0,
      10.0,
      0.1);

  fixture.softShapeNode = fixture.softBody->getShapeNode(0);
  auto* softDynamics = fixture.softShapeNode->getDynamicsAspect();
  softDynamics->setFrictionCoeff(friction);
  softDynamics->setRestitutionCoeff(restitution);

  fixture.rigidSkel = dynamics::Skeleton::create("rigid");
  auto rigidPair
      = fixture.rigidSkel->createJointAndBodyNodePair<dynamics::FreeJoint>();
  fixture.rigidBody = rigidPair.second;
  auto rigidShape
      = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.5, 0.5, 0.5));
  fixture.rigidShapeNode = fixture.rigidBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(rigidShape);
  auto* rigidDynamics = fixture.rigidShapeNode->getDynamicsAspect();
  rigidDynamics->setFrictionCoeff(friction);
  rigidDynamics->setRestitutionCoeff(restitution);

  return fixture;
}

void fillConstraintInfo(
    constraint::ConstraintInfo& info,
    std::vector<double>& x,
    std::vector<double>& lo,
    std::vector<double>& hi,
    std::vector<double>& b,
    std::vector<double>& w,
    std::vector<int>& findex,
    std::size_t dim,
    double invTimeStep)
{
  x.assign(dim, 0.0);
  lo.assign(dim, 0.0);
  hi.assign(dim, 0.0);
  b.assign(dim, 0.0);
  w.assign(dim, 0.0);
  findex.assign(dim, -1);

  info.x = x.data();
  info.lo = lo.data();
  info.hi = hi.data();
  info.b = b.data();
  info.w = w.data();
  info.findex = findex.data();
  info.invTimeStep = invTimeStep;
}

} // namespace

TEST(SoftContactConstraint, FrictionalContactInformation)
{
  constexpr double timeStep = 0.001;
  auto fixture = makeFixture(0.6, 0.5);

  TestCollisionObject softObj(fixture.detector.get(), fixture.softShapeNode);
  TestCollisionObject rigidObj(fixture.detector.get(), fixture.rigidShapeNode);

  collision::Contact contact;
  contact.collisionObject1 = &softObj;
  contact.collisionObject2 = &rigidObj;
  contact.point = Eigen::Vector3d::Zero();
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.penetrationDepth = 0.01;
  contact.triID1 = 0;
  contact.triID2 = 0;
  contact.userData = nullptr;

  ExposedSoftContactConstraint constraint(contact, timeStep);
  constraint.update();
  EXPECT_TRUE(constraint.isActive());

  const auto dim = constraint.getDimension();
  EXPECT_EQ(dim, 3u);

  constraint::ConstraintInfo info{};
  std::vector<double> x;
  std::vector<double> lo;
  std::vector<double> hi;
  std::vector<double> b;
  std::vector<double> w;
  std::vector<int> findex;
  fillConstraintInfo(info, x, lo, hi, b, w, findex, dim, 1.0 / timeStep);

  constraint.getInformation(&info);
  EXPECT_EQ(findex[0], -1);
  EXPECT_EQ(findex[1], 0);
  EXPECT_EQ(findex[2], 0);
  EXPECT_EQ(lo[0], 0.0);
  EXPECT_GT(hi[0], 0.0);

  constraint.excite();
  constraint.applyUnitImpulse(0);

  std::vector<double> vel(dim, 0.0);
  constraint.getVelocityChange(vel.data(), true);
  EXPECT_TRUE(std::isfinite(vel[0]));

  std::vector<double> lambda = {0.1, 0.05, -0.02};
  constraint.applyImpulse(lambda.data());
  constraint.unexcite();
}

TEST(SoftContactConstraint, FrictionlessContactInformation)
{
  constexpr double timeStep = 0.001;
  auto fixture = makeFixture(0.0, 0.0);

  TestCollisionObject softObj(fixture.detector.get(), fixture.softShapeNode);
  TestCollisionObject rigidObj(fixture.detector.get(), fixture.rigidShapeNode);

  collision::Contact contact;
  contact.collisionObject1 = &softObj;
  contact.collisionObject2 = &rigidObj;
  contact.point = Eigen::Vector3d::Zero();
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.penetrationDepth = 0.02;
  contact.triID1 = 0;
  contact.triID2 = 0;
  contact.userData = nullptr;

  ExposedSoftContactConstraint constraint(contact, timeStep);
  constraint.update();

  const auto dim = constraint.getDimension();
  EXPECT_EQ(dim, 1u);

  constraint::ConstraintInfo info{};
  std::vector<double> x;
  std::vector<double> lo;
  std::vector<double> hi;
  std::vector<double> b;
  std::vector<double> w;
  std::vector<int> findex;
  fillConstraintInfo(info, x, lo, hi, b, w, findex, dim, 1.0 / timeStep);

  constraint.getInformation(&info);
  EXPECT_EQ(findex[0], -1);
  EXPECT_EQ(lo[0], 0.0);
  EXPECT_GT(hi[0], 0.0);
}

TEST(SoftContactConstraint, CoefficientHelpers)
{
  auto fixture = makeFixture(0.4, 0.2);

  const auto prevAllowance
      = constraint::SoftContactConstraint::getErrorAllowance();
  const auto prevErp
      = constraint::SoftContactConstraint::getErrorReductionParameter();
  const auto prevErv
      = constraint::SoftContactConstraint::getMaxErrorReductionVelocity();
  const auto prevCfm
      = constraint::SoftContactConstraint::getConstraintForceMixing();

  EXPECT_TRUE(
      std::isfinite(
          ExposedSoftContactConstraint::computeFrictionCoefficient(
              fixture.rigidShapeNode)));
  EXPECT_TRUE(
      std::isfinite(
          ExposedSoftContactConstraint::computeRestitutionCoefficient(
              fixture.rigidShapeNode)));

  constraint::SoftContactConstraint::setErrorAllowance(-0.5);
  constraint::SoftContactConstraint::setErrorReductionParameter(1.2);
  constraint::SoftContactConstraint::setMaxErrorReductionVelocity(-1.0);
  constraint::SoftContactConstraint::setConstraintForceMixing(1e-12);

  EXPECT_TRUE(
      std::isfinite(constraint::SoftContactConstraint::getErrorAllowance()));
  EXPECT_TRUE(
      std::isfinite(
          constraint::SoftContactConstraint::getErrorReductionParameter()));
  EXPECT_TRUE(
      std::isfinite(
          constraint::SoftContactConstraint::getMaxErrorReductionVelocity()));
  EXPECT_TRUE(
      std::isfinite(
          constraint::SoftContactConstraint::getConstraintForceMixing()));

  constraint::SoftContactConstraint::setErrorAllowance(prevAllowance);
  constraint::SoftContactConstraint::setErrorReductionParameter(prevErp);
  constraint::SoftContactConstraint::setMaxErrorReductionVelocity(prevErv);
  constraint::SoftContactConstraint::setConstraintForceMixing(prevCfm);
}

} // namespace dart::test
