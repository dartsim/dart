// Copyright (c) 2011, The DART development contributors

#include "helpers/dynamics_helpers.hpp"

#include <dart/simulation/world.hpp>

#include <dart/constraint/constraint_solver.hpp>
#include <dart/constraint/contact_constraint.hpp>
#include <dart/constraint/contact_surface.hpp>
#include <dart/constraint/soft_contact_constraint.hpp>

#include <dart/collision/collision_object.hpp>
#include <dart/collision/contact.hpp>
#include <dart/collision/dart/dart_collision_detector.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/soft_body_node.hpp>
#include <dart/dynamics/soft_mesh_shape.hpp>

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
  using SoftContactConstraint::getRootSkeleton;
  using SoftContactConstraint::getVelocityChange;
  using SoftContactConstraint::isActive;
  using SoftContactConstraint::SoftContactConstraint;
  using SoftContactConstraint::unexcite;
  using SoftContactConstraint::uniteSkeletons;
  using SoftContactConstraint::update;
};

class ExposedContactConstraint final : public constraint::ContactConstraint
{
public:
  using ContactConstraint::applyImpulse;
  using ContactConstraint::applyUnitImpulse;
  using ContactConstraint::ContactConstraint;
  using ContactConstraint::excite;
  using ContactConstraint::getInformation;
  using ContactConstraint::getVelocityChange;
  using ContactConstraint::isActive;
  using ContactConstraint::unexcite;
  using ContactConstraint::update;
};

struct LcpBuffers
{
  std::vector<double> x;
  std::vector<double> lo;
  std::vector<double> hi;
  std::vector<double> b;
  std::vector<double> w;
  std::vector<int> findex;
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

void prepareConstraintInfo(
    constraint::ConstraintInfo& info,
    LcpBuffers& buffers,
    std::size_t dim,
    double invTimeStep,
    constraint::ConstraintPhase phase,
    bool useSplitImpulse)
{
  buffers.x.assign(dim, 0.0);
  buffers.lo.assign(dim, 0.0);
  buffers.hi.assign(dim, 0.0);
  buffers.b.assign(dim, 0.0);
  buffers.w.assign(dim, 0.0);
  buffers.findex.assign(dim, -1);

  info.x = buffers.x.data();
  info.lo = buffers.lo.data();
  info.hi = buffers.hi.data();
  info.b = buffers.b.data();
  info.w = buffers.w.data();
  info.findex = buffers.findex.data();
  info.invTimeStep = invTimeStep;
  info.phase = phase;
  info.useSplitImpulse = useSplitImpulse;
}

void setSurfaceProperties(
    const dynamics::SkeletonPtr& skeleton, double friction, double restitution)
{
  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    auto* bodyNode = skeleton->getBodyNode(i);
    for (std::size_t j = 0; j < bodyNode->getNumShapeNodes(); ++j) {
      auto* shapeNode = bodyNode->getShapeNode(j);
      auto* dynAspect = shapeNode->getDynamicsAspect();
      if (dynAspect) {
        dynAspect->setFrictionCoeff(friction);
        dynAspect->setRestitutionCoeff(restitution);
      }
    }
  }
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

TEST(SoftContactConstraint, ContactConstraintFrictionRestitutionPaths)
{
  constexpr double timeStep = 0.001;
  auto detector = collision::DARTCollisionDetector::create();
  ASSERT_NE(detector, nullptr);

  auto skelA = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.0, 0.0, 0.0));
  auto skelB = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.0, 0.19, 0.0));

  auto* shapeA = skelA->getBodyNode(0)->getShapeNode(0);
  auto* shapeB = skelB->getBodyNode(0)->getShapeNode(0);
  ASSERT_NE(shapeA, nullptr);
  ASSERT_NE(shapeB, nullptr);

  setSurfaceProperties(skelA, 0.8, 0.6);
  setSurfaceProperties(skelB, 0.4, 0.6);

  TestCollisionObject objectA(detector.get(), shapeA);
  TestCollisionObject objectB(detector.get(), shapeB);

  collision::Contact contact;
  contact.collisionObject1 = &objectA;
  contact.collisionObject2 = &objectB;
  contact.point = Eigen::Vector3d(0.0, 0.1, 0.0);
  contact.normal = Eigen::Vector3d::UnitY();
  contact.penetrationDepth = 0.01;
  contact.triID1 = 0;
  contact.triID2 = 0;
  contact.userData = nullptr;

  constraint::ContactSurfaceParams params;
  params.mPrimaryFrictionCoeff = 0.8;
  params.mSecondaryFrictionCoeff = 0.4;
  params.mRestitutionCoeff = 0.5;
  params.mFirstFrictionalDirection = Eigen::Vector3d::UnitX();
  params.mContactSurfaceMotionVelocity = Eigen::Vector3d::Zero();

  ExposedContactConstraint constraint(contact, timeStep, params);
  constraint.setFrictionDirection(Eigen::Vector3d::UnitX());
  constraint.update();
  EXPECT_TRUE(constraint.isActive());
  EXPECT_EQ(constraint.getDimension(), 3u);

  constraint::ConstraintInfo info{};
  LcpBuffers buffers;
  prepareConstraintInfo(
      info,
      buffers,
      constraint.getDimension(),
      1.0 / timeStep,
      constraint::ConstraintPhase::Velocity,
      false);
  constraint.getInformation(&info);

  prepareConstraintInfo(
      info,
      buffers,
      constraint.getDimension(),
      1.0 / timeStep,
      constraint::ConstraintPhase::Position,
      true);
  constraint.getInformation(&info);

  constraint.applyUnitImpulse(0);
  std::vector<double> vel(constraint.getDimension(), 0.0);
  constraint.getVelocityChange(vel.data(), true);
  std::vector<double> lambda(constraint.getDimension(), 0.1);
  constraint.applyImpulse(lambda.data());
  constraint.excite();
  constraint.unexcite();
}

TEST(SoftContactConstraint, SoftContactConstraintFromWorld)
{
  // Build a world programmatically with a soft body above a rigid ground plane
  // so that stepping produces contacts.
  auto fixture = makeFixture(0.7, 0.4);

  TestCollisionObject softObj(fixture.detector.get(), fixture.softShapeNode);
  TestCollisionObject rigidObj(fixture.detector.get(), fixture.rigidShapeNode);

  collision::Contact contact;
  contact.collisionObject1 = &softObj;
  contact.collisionObject2 = &rigidObj;
  contact.point = Eigen::Vector3d::Zero();
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.penetrationDepth = 0.01;

  ExposedSoftContactConstraint constraint(contact, 0.001);

  const double prevAllowance
      = constraint::SoftContactConstraint::getErrorAllowance();
  const double prevErp
      = constraint::SoftContactConstraint::getErrorReductionParameter();
  const double prevErv
      = constraint::SoftContactConstraint::getMaxErrorReductionVelocity();
  const double prevCfm
      = constraint::SoftContactConstraint::getConstraintForceMixing();

  constraint::SoftContactConstraint::setErrorAllowance(-0.2);
  constraint::SoftContactConstraint::setErrorReductionParameter(1.5);
  constraint::SoftContactConstraint::setMaxErrorReductionVelocity(-0.1);
  constraint::SoftContactConstraint::setConstraintForceMixing(1e-12);

  constraint::SoftContactConstraint::setErrorAllowance(prevAllowance);
  constraint::SoftContactConstraint::setErrorReductionParameter(prevErp);
  constraint::SoftContactConstraint::setMaxErrorReductionVelocity(prevErv);
  constraint::SoftContactConstraint::setConstraintForceMixing(prevCfm);

  constraint.setFrictionDirection(Eigen::Vector3d::UnitX());
  constraint.update();
  EXPECT_TRUE(constraint.isActive());

  constraint::ConstraintInfo info{};
  LcpBuffers buffers;
  prepareConstraintInfo(
      info,
      buffers,
      constraint.getDimension(),
      1.0 / 0.001,
      constraint::ConstraintPhase::Velocity,
      false);
  constraint.getInformation(&info);

  constraint.applyUnitImpulse(0);
  std::vector<double> vel(constraint.getDimension(), 0.0);
  constraint.getVelocityChange(vel.data(), true);
  std::vector<double> lambda(constraint.getDimension(), 0.05);
  constraint.applyImpulse(lambda.data());
  constraint.excite();
  constraint.unexcite();
}

TEST(SoftContactConstraint, StaticTypeNonEmpty)
{
  auto type = constraint::SoftContactConstraint::getStaticType();
  EXPECT_FALSE(type.empty());
}

TEST(SoftContactConstraint, ErrorAllowanceAndErpClampPaths)
{
  const auto prevAllowance
      = constraint::SoftContactConstraint::getErrorAllowance();
  const auto prevErp
      = constraint::SoftContactConstraint::getErrorReductionParameter();

  constraint::SoftContactConstraint::setErrorAllowance(-0.5);
  EXPECT_LE(constraint::SoftContactConstraint::getErrorAllowance(), 0.0);

  constraint::SoftContactConstraint::setErrorReductionParameter(-0.25);
  EXPECT_LE(
      constraint::SoftContactConstraint::getErrorReductionParameter(), 0.0);

  constraint::SoftContactConstraint::setErrorReductionParameter(1.5);
  EXPECT_GE(
      constraint::SoftContactConstraint::getErrorReductionParameter(), 1.0);

  constraint::SoftContactConstraint::setErrorAllowance(prevAllowance);
  constraint::SoftContactConstraint::setErrorReductionParameter(prevErp);
}

TEST(SoftContactConstraint, ErvAndCfmClampPaths)
{
  const auto prevErv
      = constraint::SoftContactConstraint::getMaxErrorReductionVelocity();
  const auto prevCfm
      = constraint::SoftContactConstraint::getConstraintForceMixing();

  constraint::SoftContactConstraint::setMaxErrorReductionVelocity(-1.0);
  EXPECT_LE(
      constraint::SoftContactConstraint::getMaxErrorReductionVelocity(), 0.0);

  constraint::SoftContactConstraint::setConstraintForceMixing(1e-12);
  EXPECT_LE(
      constraint::SoftContactConstraint::getConstraintForceMixing(), 1e-9);

  constraint::SoftContactConstraint::setMaxErrorReductionVelocity(prevErv);
  constraint::SoftContactConstraint::setConstraintForceMixing(prevCfm);
}

TEST(SoftContactConstraint, FrictionlessApplyImpulsePath)
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
  ASSERT_EQ(constraint.getDimension(), 1u);

  std::vector<double> lambda = {0.2};
  constraint.applyImpulse(lambda.data());

  EXPECT_NEAR(contact.force.z(), 0.2 / timeStep, 1e-12);
}

TEST(SoftContactConstraint, FrictionlessBouncingVelocityCapped)
{
  constexpr double timeStep = 0.001;
  auto fixture = makeFixture(0.0, 1.0);

  auto* softJoint
      = dynamic_cast<dynamics::FreeJoint*>(fixture.softBody->getParentJoint());
  ASSERT_NE(softJoint, nullptr);
  softJoint->setLinearVelocity(Eigen::Vector3d(0.0, 0.0, -200.0));
  fixture.softSkel->computeForwardKinematics(true, true, false);

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
  ASSERT_EQ(constraint.getDimension(), 1u);

  constraint::ConstraintInfo info{};
  std::vector<double> x;
  std::vector<double> lo;
  std::vector<double> hi;
  std::vector<double> b;
  std::vector<double> w;
  std::vector<int> findex;
  fillConstraintInfo(info, x, lo, hi, b, w, findex, 1u, 1.0 / timeStep);

  constraint.getInformation(&info);
  EXPECT_NEAR(b[0], 300.0, 1e-9);
}

TEST(SoftContactConstraint, FrictionalBouncingAndTangentFallback)
{
  constexpr double timeStep = 0.001;
  auto fixture = makeFixture(0.5, 1.0);

  auto* softJoint
      = dynamic_cast<dynamics::FreeJoint*>(fixture.softBody->getParentJoint());
  ASSERT_NE(softJoint, nullptr);
  softJoint->setLinearVelocity(Eigen::Vector3d(0.0, 0.0, -200.0));
  fixture.softSkel->computeForwardKinematics(true, true, false);

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
  constraint.setFrictionDirection(Eigen::Vector3d::UnitZ());
  constraint.update();
  ASSERT_EQ(constraint.getDimension(), 3u);

  constraint::ConstraintInfo info{};
  std::vector<double> x;
  std::vector<double> lo;
  std::vector<double> hi;
  std::vector<double> b;
  std::vector<double> w;
  std::vector<int> findex;
  fillConstraintInfo(info, x, lo, hi, b, w, findex, 3u, 1.0 / timeStep);

  constraint.getInformation(&info);
  EXPECT_NEAR(b[0], 300.0, 1e-9);

  std::vector<double> lambda = {0.1, 0.05, -0.02};
  constraint.applyImpulse(lambda.data());
  EXPECT_TRUE(contact.force.allFinite());
}

TEST(SoftContactConstraint, PointMassImpulseBranches)
{
  constexpr double timeStep = 0.001;
  auto fixture = makeFixture(0.6, 0.2);

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
  constraint.setFrictionDirection(Eigen::Vector3d::UnitX());
  constraint.update();
  ASSERT_GE(constraint.getDimension(), 1u);

  constraint.applyUnitImpulse(0);
  std::vector<double> vel(constraint.getDimension(), 0.0);
  constraint.getVelocityChange(vel.data(), true);

  std::vector<double> lambda(constraint.getDimension(), 0.05);
  constraint.applyImpulse(lambda.data());
  constraint.excite();
  constraint.unexcite();
  EXPECT_TRUE(contact.force.allFinite());

  collision::Contact swapped = contact;
  swapped.collisionObject1 = &rigidObj;
  swapped.collisionObject2 = &softObj;
  swapped.triID1 = 0;
  swapped.triID2 = 0;

  ExposedSoftContactConstraint constraint2(swapped, timeStep);
  constraint2.setFrictionDirection(Eigen::Vector3d::UnitX());
  constraint2.update();
  ASSERT_GE(constraint2.getDimension(), 1u);

  constraint2.applyUnitImpulse(0);
  std::vector<double> vel2(constraint2.getDimension(), 0.0);
  constraint2.getVelocityChange(vel2.data(), true);

  std::vector<double> lambda2(constraint2.getDimension(), 0.02);
  constraint2.applyImpulse(lambda2.data());
  constraint2.excite();
  constraint2.unexcite();
  EXPECT_TRUE(swapped.force.allFinite());
}

TEST(SoftContactConstraint, SelfCollisionApplyUnitImpulsePath)
{
  constexpr double timeStep = 0.001;
  auto detector = collision::DARTCollisionDetector::create();
  ASSERT_NE(detector, nullptr);

  auto skel = dynamics::Skeleton::create("self");
  auto softPair = skel->createJointAndBodyNodePair<
      dynamics::FreeJoint,
      dynamics::SoftBodyNode>();
  auto rigidPair = skel->createJointAndBodyNodePair<dynamics::FreeJoint>();

  auto* softBody = softPair.second;
  dynamics::SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      Eigen::Isometry3d::Identity(),
      1.0,
      10.0,
      10.0,
      0.1);
  auto* softShape = softBody->getShapeNode(0);
  ASSERT_NE(softShape, nullptr);
  auto* softDynamics = softShape->getDynamicsAspect();
  ASSERT_NE(softDynamics, nullptr);
  softDynamics->setFrictionCoeff(0.5);
  softDynamics->setRestitutionCoeff(0.1);

  auto* rigidBody = rigidPair.second;
  auto rigidShape
      = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.5, 0.5, 0.5));
  auto* rigidShapeNode = rigidBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(rigidShape);
  ASSERT_NE(rigidShapeNode, nullptr);
  auto* rigidDynamics = rigidShapeNode->getDynamicsAspect();
  ASSERT_NE(rigidDynamics, nullptr);
  rigidDynamics->setFrictionCoeff(0.5);
  rigidDynamics->setRestitutionCoeff(0.1);

  TestCollisionObject softObj(detector.get(), softShape);
  TestCollisionObject rigidObj(detector.get(), rigidShapeNode);

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
  constraint.applyUnitImpulse(0);
  EXPECT_TRUE(constraint.isActive());
}

TEST(SoftContactConstraint, RootSkeletonSelectionPaths)
{
  constexpr double timeStep = 0.001;
  auto fixture = makeFixture(0.2, 0.0);

  TestCollisionObject softObj(fixture.detector.get(), fixture.softShapeNode);
  TestCollisionObject rigidObj(fixture.detector.get(), fixture.rigidShapeNode);

  collision::Contact softContact;
  softContact.collisionObject1 = &softObj;
  softContact.collisionObject2 = &rigidObj;
  softContact.point = Eigen::Vector3d::Zero();
  softContact.normal = Eigen::Vector3d::UnitZ();
  softContact.penetrationDepth = 0.01;
  softContact.triID1 = 0;
  softContact.triID2 = 0;
  softContact.userData = nullptr;

  ExposedSoftContactConstraint softConstraint(softContact, timeStep);
  EXPECT_EQ(softConstraint.getRootSkeleton(), fixture.softSkel);

  auto detector = collision::DARTCollisionDetector::create();
  auto skelA = dynamics::Skeleton::create("rigidA");
  auto skelB = dynamics::Skeleton::create("rigidB");
  auto pairA = skelA->createJointAndBodyNodePair<dynamics::FreeJoint>();
  auto pairB = skelB->createJointAndBodyNodePair<dynamics::FreeJoint>();
  auto shapeA
      = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.2, 0.2, 0.2));
  auto shapeB
      = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.2, 0.2, 0.2));
  auto* nodeA = pairA.second->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shapeA);
  auto* nodeB = pairB.second->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shapeB);
  ASSERT_NE(nodeA, nullptr);
  ASSERT_NE(nodeB, nullptr);
  skelA->setMobile(false);

  TestCollisionObject objectA(detector.get(), nodeA);
  TestCollisionObject objectB(detector.get(), nodeB);

  collision::Contact rigidContact;
  rigidContact.collisionObject1 = &objectA;
  rigidContact.collisionObject2 = &objectB;
  rigidContact.point = Eigen::Vector3d::Zero();
  rigidContact.normal = Eigen::Vector3d::UnitZ();
  rigidContact.penetrationDepth = 0.01;
  rigidContact.triID1 = 0;
  rigidContact.triID2 = 0;
  rigidContact.userData = nullptr;

  ExposedSoftContactConstraint rigidConstraint(rigidContact, timeStep);
  EXPECT_EQ(rigidConstraint.getRootSkeleton(), skelB);
}

TEST(SoftContactConstraint, UniteSkeletonsUnionSizePaths)
{
  constexpr double timeStep = 0.001;
  auto fixture = makeFixture(0.5, 0.0);

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

  fixture.softSkel->resetUnion();
  fixture.rigidSkel->resetUnion();
  fixture.softSkel->mUnionSize = 1;
  fixture.rigidSkel->mUnionSize = 2;
  constraint.uniteSkeletons();
  EXPECT_EQ(fixture.softSkel->mUnionRootSkeleton.lock(), fixture.rigidSkel);

  fixture.softSkel->resetUnion();
  fixture.rigidSkel->resetUnion();
  fixture.softSkel->mUnionSize = 3;
  fixture.rigidSkel->mUnionSize = 1;
  constraint.uniteSkeletons();
  EXPECT_EQ(fixture.rigidSkel->mUnionRootSkeleton.lock(), fixture.softSkel);
}

TEST(SoftContactConstraint, DynamicsAspectFallbacks)
{
  auto skel = dynamics::Skeleton::create("props");
  auto pair = skel->createJointAndBodyNodePair<dynamics::FreeJoint>();
  auto shape
      = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.1, 0.1, 0.1));

  auto* collisionOnly
      = pair.second->createShapeNodeWith<dynamics::CollisionAspect>(shape);
  ASSERT_NE(collisionOnly, nullptr);

  const double friction
      = ExposedSoftContactConstraint::computeFrictionCoefficient(collisionOnly);
  EXPECT_DOUBLE_EQ(friction, 1.0);

  auto* withDynamics = pair.second->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  ASSERT_NE(withDynamics, nullptr);
  auto* dynAspect = withDynamics->getDynamicsAspect();
  ASSERT_NE(dynAspect, nullptr);
  dynAspect->setRestitutionCoeff(2.0);

  const double restitution
      = ExposedSoftContactConstraint::computeRestitutionCoefficient(
          withDynamics);
  EXPECT_DOUBLE_EQ(restitution, 0.0);
}

namespace {

struct SoftWorldFixture
{
  simulation::WorldPtr world;
  dynamics::SkeletonPtr ground;
  dynamics::SkeletonPtr softSkel;
  dynamics::SoftBodyNode* softBody{nullptr};
};

SoftWorldFixture makeSoftWorld(
    const Eigen::Vector3d& softOffset,
    double frictionCoeff,
    double restitutionCoeff)
{
  SoftWorldFixture fixture;
  fixture.world = simulation::World::create();
  fixture.world->setTimeStep(0.001);
  fixture.world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  fixture.ground = createGround(
      Eigen::Vector3d(2.0, 0.1, 2.0), Eigen::Vector3d(0.0, -0.05, 0.0));
  fixture.ground->setMobile(false);

  fixture.softSkel = dynamics::Skeleton::create("soft_world");
  auto softPair = fixture.softSkel->createJointAndBodyNodePair<
      dynamics::FreeJoint,
      dynamics::SoftBodyNode>();
  fixture.softBody = softPair.second;
  dynamics::SoftBodyNodeHelper::setBox(
      fixture.softBody,
      Eigen::Vector3d(0.4, 0.4, 0.4),
      Eigen::Isometry3d::Identity(),
      1.0,
      10.0,
      10.0,
      0.1);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = softOffset;
  dynamics::FreeJoint::setTransformOf(fixture.softBody, tf);

  for (std::size_t i = 0; i < fixture.softBody->getNumShapeNodes(); ++i) {
    auto* shapeNode = fixture.softBody->getShapeNode(i);
    auto* dynAspect = shapeNode->getDynamicsAspect();
    if (dynAspect) {
      dynAspect->setFrictionCoeff(frictionCoeff);
      dynAspect->setRestitutionCoeff(restitutionCoeff);
    }
  }

  fixture.world->addSkeleton(fixture.ground);
  fixture.world->addSkeleton(fixture.softSkel);

  return fixture;
}

} // namespace

TEST(SoftContactConstraint, WorldStepGeneratesSoftContacts)
{
  auto fixture = makeSoftWorld(Eigen::Vector3d(0.0, 0.05, 0.0), 0.6, 0.2);

  for (int i = 0; i < 120; ++i) {
    fixture.world->step();
  }

  auto solver = fixture.world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);
  EXPECT_GT(solver->getLastCollisionResult().getNumContacts(), 0u);
}

TEST(SoftContactConstraint, WorldStepFrictionlessSoftBody)
{
  auto fixture = makeSoftWorld(Eigen::Vector3d(0.0, 0.05, 0.0), 0.0, 0.0);

  for (int i = 0; i < 120; ++i) {
    fixture.world->step();
  }

  auto solver = fixture.world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);
  EXPECT_GT(solver->getLastCollisionResult().getNumContacts(), 0u);
}

TEST(SoftContactConstraint, WorldStepWithSplitImpulse)
{
  auto fixture = makeSoftWorld(Eigen::Vector3d(0.0, 0.05, 0.0), 0.4, 0.1);

  auto solver = fixture.world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);
  solver->setSplitImpulseEnabled(true);

  for (int i = 0; i < 120; ++i) {
    fixture.world->step();
  }

  EXPECT_TRUE(solver->isSplitImpulseEnabled());
  EXPECT_GT(solver->getLastCollisionResult().getNumContacts(), 0u);
}

TEST(SoftContactConstraint, WorldStepPositionsStayFinite)
{
  auto fixture = makeSoftWorld(Eigen::Vector3d(0.0, 0.05, 0.0), 0.5, 0.3);

  for (int i = 0; i < 120; ++i) {
    fixture.world->step();
  }

  const Eigen::VectorXd positions = fixture.softSkel->getPositions();
  EXPECT_TRUE(positions.array().isFinite().all());
}

TEST(SoftContactConstraint, WorldStepAfterClearingCollisionResult)
{
  auto fixture = makeSoftWorld(Eigen::Vector3d(0.0, 0.05, 0.0), 0.7, 0.2);

  auto solver = fixture.world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);

  for (int i = 0; i < 60; ++i) {
    fixture.world->step();
  }

  EXPECT_GT(solver->getLastCollisionResult().getNumContacts(), 0u);
  solver->clearLastCollisionResult();
  EXPECT_EQ(solver->getLastCollisionResult().getNumContacts(), 0u);

  for (int i = 0; i < 60; ++i) {
    fixture.world->step();
  }

  EXPECT_GT(solver->getLastCollisionResult().getNumContacts(), 0u);
}

} // namespace dart::test
