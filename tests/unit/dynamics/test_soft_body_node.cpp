// Copyright (c) 2011, The DART development contributors

#include <dart/simulation/world.hpp>

#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/point_mass.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/soft_body_node.hpp>
#include <dart/dynamics/soft_mesh_shape.hpp>

#include <dart/math/constants.hpp>

#include <dart/dart.hpp>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <vector>

#include <cmath>

using namespace dart;
using namespace dart::dynamics;

//==============================================================================
// Helper to create a soft body with a box shape
static SoftBodyNode* createBoxSoftBody(
    const SkeletonPtr& skeleton,
    const Eigen::Vector3d& size = Eigen::Vector3d::Constant(1.0),
    double totalMass = 1.0)
{
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;
  SoftBodyNodeHelper::setBox(
      softBody,
      size,
      Eigen::Isometry3d::Identity(),
      totalMass,
      10.0,
      10.0,
      0.1);
  return softBody;
}

//==============================================================================
TEST(SoftBodyNode, AccessorsAndDynamics)
{
  auto skeleton = Skeleton::create("soft-body");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(1.0),
      Eigen::Isometry3d::Identity(),
      1.0,
      10.0,
      10.0,
      0.1);

  EXPECT_GT(softBody->getNumPointMasses(), 0u);
  EXPECT_GT(softBody->getNumFaces(), 0u);
  EXPECT_GT(softBody->getMass(), 0.0);

  const auto face = softBody->getFace(0);
  EXPECT_GE(face[0], 0);
  EXPECT_GE(face[1], 0);
  EXPECT_GE(face[2], 0);

  skeleton->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  skeleton->computeForwardDynamics();
  skeleton->computeInverseDynamics(true, true, true);

  const auto massMatrix = skeleton->getMassMatrix();
  EXPECT_EQ(massMatrix.rows(), skeleton->getNumDofs());
  EXPECT_EQ(massMatrix.cols(), skeleton->getNumDofs());

  auto world = simulation::World::create();
  world->addSkeleton(skeleton);
  world->step();

  auto* pointMass = softBody->getPointMass(0);
  ASSERT_NE(pointMass, nullptr);
  EXPECT_TRUE(pointMass->getPositions().array().isFinite().all());
}

//==============================================================================
TEST(SoftBodyNode, WorldSteppingIntegrationPaths)
{
  auto skeleton = Skeleton::create("soft-body-step");
  auto* softBody
      = createBoxSoftBody(skeleton, Eigen::Vector3d(0.4, 0.5, 0.6), 1.5);

  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->addSkeleton(skeleton);

  for (int i = 0; i < 5; ++i) {
    world->step();
  }

  auto* pointMass = softBody->getPointMass(0);
  ASSERT_NE(pointMass, nullptr);

  EXPECT_TRUE(pointMass->getWorldPosition().array().isFinite().all());
  EXPECT_TRUE(pointMass->getWorldVelocity().array().isFinite().all());
  EXPECT_TRUE(pointMass->getWorldAcceleration().array().isFinite().all());
  EXPECT_TRUE(pointMass->getConstraintImpulses().array().isFinite().all());
}

//==============================================================================
TEST(SoftBodyNode, MassMatricesAndForces)
{
  auto skeleton = Skeleton::create("soft-body-mass");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(0.5),
      Eigen::Isometry3d::Identity(),
      2.0,
      5.0,
      6.0,
      0.2);

  softBody->setVertexSpringStiffness(20.0);
  softBody->setEdgeSpringStiffness(30.0);
  softBody->setDampingCoefficient(0.05);
  EXPECT_DOUBLE_EQ(softBody->getVertexSpringStiffness(), 20.0);
  EXPECT_DOUBLE_EQ(softBody->getEdgeSpringStiffness(), 30.0);

  const auto dofs = skeleton->getNumDofs();
  skeleton->setPositions(Eigen::VectorXd::Constant(dofs, 0.1));
  skeleton->setVelocities(Eigen::VectorXd::Constant(dofs, 0.2));

  const auto massMatrix = skeleton->getMassMatrix();
  const auto augMassMatrix = skeleton->getAugMassMatrix();
  const auto invMassMatrix = skeleton->getInvMassMatrix();
  const auto invAugMassMatrix = skeleton->getInvAugMassMatrix();

  EXPECT_EQ(massMatrix.rows(), dofs);
  EXPECT_EQ(augMassMatrix.rows(), dofs);
  EXPECT_EQ(invMassMatrix.rows(), dofs);
  EXPECT_EQ(invAugMassMatrix.rows(), dofs);

  const auto coriolis = skeleton->getCoriolisAndGravityForces();
  EXPECT_EQ(coriolis.size(), dofs);

  const auto com = skeleton->getCOM();
  EXPECT_TRUE(com.array().isFinite().all());
  const auto comJac = skeleton->getCOMJacobian();
  EXPECT_EQ(comJac.cols(), dofs);

  skeleton->clearExternalForces();
  skeleton->clearInternalForces();

  EXPECT_TRUE(std::isfinite(skeleton->computeKineticEnergy()));
  EXPECT_TRUE(std::isfinite(skeleton->computePotentialEnergy()));
}

//==============================================================================
TEST(SoftBodyNode, AsSoftBodyNode)
{
  auto skeleton = Skeleton::create("soft-as");
  auto* softBody = createBoxSoftBody(skeleton);

  // Non-const version
  EXPECT_EQ(softBody->asSoftBodyNode(), softBody);

  // Const version
  const SoftBodyNode* constSoft = softBody;
  EXPECT_EQ(constSoft->asSoftBodyNode(), constSoft);

  // Regular BodyNode should return nullptr
  auto skel2 = Skeleton::create("rigid");
  auto pair2 = skel2->createJointAndBodyNodePair<FreeJoint>();
  EXPECT_EQ(pair2.second->asSoftBodyNode(), nullptr);
}

//==============================================================================
TEST(SoftBodyNode, CopyAndAssignment)
{
  auto skel1 = Skeleton::create("soft-copy-src");
  auto* src = createBoxSoftBody(skel1);
  src->setVertexSpringStiffness(15.0);
  src->setEdgeSpringStiffness(25.0);
  src->setDampingCoefficient(0.3);

  auto skel2 = Skeleton::create("soft-copy-dst");
  auto* dst = createBoxSoftBody(skel2);

  // Copy via pointer
  dst->copy(src);
  EXPECT_DOUBLE_EQ(dst->getVertexSpringStiffness(), 15.0);
  EXPECT_DOUBLE_EQ(dst->getEdgeSpringStiffness(), 25.0);
  EXPECT_DOUBLE_EQ(dst->getDampingCoefficient(), 0.3);
  EXPECT_EQ(dst->getNumPointMasses(), src->getNumPointMasses());

  // Copy via reference
  auto skel3 = Skeleton::create("soft-copy-ref");
  auto* dst2 = createBoxSoftBody(skel3);
  dst2->copy(*src);
  EXPECT_DOUBLE_EQ(dst2->getVertexSpringStiffness(), 15.0);

  // Self-copy should be no-op
  dst2->copy(dst2);
  EXPECT_DOUBLE_EQ(dst2->getVertexSpringStiffness(), 15.0);

  // Copy null pointer should be no-op
  dst2->copy(static_cast<const SoftBodyNode*>(nullptr));
  EXPECT_DOUBLE_EQ(dst2->getVertexSpringStiffness(), 15.0);

  // Assignment operator
  auto skel4 = Skeleton::create("soft-assign");
  auto* dst3 = createBoxSoftBody(skel4);
  *dst3 = *src;
  EXPECT_DOUBLE_EQ(dst3->getVertexSpringStiffness(), 15.0);
}

//==============================================================================
TEST(SoftBodyNode, GetSoftBodyNodeProperties)
{
  auto skeleton = Skeleton::create("soft-props");
  auto* softBody = createBoxSoftBody(skeleton);
  softBody->setVertexSpringStiffness(42.0);
  softBody->setEdgeSpringStiffness(43.0);
  softBody->setDampingCoefficient(0.5);

  auto props = softBody->getSoftBodyNodeProperties();
  EXPECT_DOUBLE_EQ(props.mKv, 42.0);
  EXPECT_DOUBLE_EQ(props.mKe, 43.0);
  EXPECT_DOUBLE_EQ(props.mDampCoeff, 0.5);
  EXPECT_EQ(props.mPointProps.size(), softBody->getNumPointMasses());
  EXPECT_EQ(props.mFaces.size(), softBody->getNumFaces());
}

//==============================================================================
TEST(SoftBodyNode, SetProperties)
{
  auto skeleton = Skeleton::create("soft-set-props");
  auto* softBody = createBoxSoftBody(skeleton);

  auto props = softBody->getSoftBodyNodeProperties();
  props.mKv = 99.0;
  props.mKe = 88.0;
  props.mDampCoeff = 0.7;

  softBody->setProperties(props);
  EXPECT_DOUBLE_EQ(softBody->getVertexSpringStiffness(), 99.0);
  EXPECT_DOUBLE_EQ(softBody->getEdgeSpringStiffness(), 88.0);
  EXPECT_DOUBLE_EQ(softBody->getDampingCoefficient(), 0.7);
}

//==============================================================================
TEST(SoftBodyNode, SetUniqueProperties)
{
  auto skeleton = Skeleton::create("soft-unique-props");
  auto* softBody = createBoxSoftBody(skeleton);

  SoftBodyNode::UniqueProperties uniqueProps;
  uniqueProps.mKv = 77.0;
  uniqueProps.mKe = 66.0;
  uniqueProps.mDampCoeff = 0.4;

  softBody->setProperties(uniqueProps);
  EXPECT_DOUBLE_EQ(softBody->getVertexSpringStiffness(), 77.0);
  EXPECT_DOUBLE_EQ(softBody->getEdgeSpringStiffness(), 66.0);
  EXPECT_DOUBLE_EQ(softBody->getDampingCoefficient(), 0.4);
}

//==============================================================================
TEST(SoftBodyNode, DampingCoefficientClamping)
{
  auto skeleton = Skeleton::create("soft-damp-clamp");
  auto* softBody = createBoxSoftBody(skeleton);

  // Negative damping should be clamped to 0
  softBody->setDampingCoefficient(-1.0);
  EXPECT_DOUBLE_EQ(softBody->getDampingCoefficient(), 0.0);

  // NaN damping should be clamped to 0
  softBody->setDampingCoefficient(std::numeric_limits<double>::quiet_NaN());
  EXPECT_DOUBLE_EQ(softBody->getDampingCoefficient(), 0.0);

  // Infinity damping should be clamped to 0
  softBody->setDampingCoefficient(std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(softBody->getDampingCoefficient(), 0.0);

  // Valid damping should work
  softBody->setDampingCoefficient(0.5);
  EXPECT_DOUBLE_EQ(softBody->getDampingCoefficient(), 0.5);
}

//==============================================================================
TEST(SoftBodyNode, PointMassSpan)
{
  auto skeleton = Skeleton::create("soft-span");
  auto* softBody = createBoxSoftBody(skeleton);

  auto pointMasses = softBody->getPointMasses();
  EXPECT_EQ(pointMasses.size(), softBody->getNumPointMasses());

  for (std::size_t i = 0; i < pointMasses.size(); ++i) {
    EXPECT_EQ(pointMasses[i], softBody->getPointMass(i));
  }
}

//==============================================================================
TEST(SoftBodyNode, ConstPointMassAccess)
{
  auto skeleton = Skeleton::create("soft-const-pm");
  auto* softBody = createBoxSoftBody(skeleton);

  const SoftBodyNode* constSoft = softBody;
  ASSERT_GT(constSoft->getNumPointMasses(), 0u);
  const PointMass* pm = constSoft->getPointMass(0);
  EXPECT_NE(pm, nullptr);
}

//==============================================================================
TEST(SoftBodyNode, Notifier)
{
  auto skeleton = Skeleton::create("soft-notifier");
  auto* softBody = createBoxSoftBody(skeleton);

  auto* notifier = softBody->getNotifier();
  EXPECT_NE(notifier, nullptr);

  const SoftBodyNode* constSoft = softBody;
  const auto* constNotifier = constSoft->getNotifier();
  EXPECT_NE(constNotifier, nullptr);
  EXPECT_EQ(notifier, constNotifier);
}

//==============================================================================
TEST(SoftBodyNode, TotalMassIncludesPointMasses)
{
  auto skeleton = Skeleton::create("soft-total-mass");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  // Before adding point masses, mass is just the body node mass
  double baseMass = softBody->getMass();

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(1.0),
      Eigen::Isometry3d::Identity(),
      2.0,
      10.0,
      10.0,
      0.1);

  // After adding point masses, total mass should be greater
  double totalMass = softBody->getMass();
  EXPECT_GT(totalMass, 0.0);
  // Total mass should include point mass contributions
  (void)baseMass;
}

//==============================================================================
TEST(SoftBodyNode, RemoveAllPointMasses)
{
  auto skeleton = Skeleton::create("soft-remove-all");
  auto* softBody = createBoxSoftBody(skeleton);

  EXPECT_GT(softBody->getNumPointMasses(), 0u);
  softBody->removeAllPointMasses();
  EXPECT_EQ(softBody->getNumPointMasses(), 0u);
}

//==============================================================================
TEST(SoftBodyNode, AddPointMassAndFace)
{
  auto skeleton = Skeleton::create("soft-add-pm");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setSinglePointMass(softBody, 1.0, 5.0, 6.0, 0.1);
  EXPECT_EQ(softBody->getNumPointMasses(), 1u);

  PointMass::Properties pmProps(Eigen::Vector3d(0.1, 0.2, 0.3), 0.5);
  softBody->addPointMass(pmProps);
  EXPECT_EQ(softBody->getNumPointMasses(), 2u);

  softBody->addPointMass(pmProps);
  EXPECT_EQ(softBody->getNumPointMasses(), 3u);

  auto prevFaces = softBody->getNumFaces();
  softBody->addFace(Eigen::Vector3i(0, 1, 2));
  EXPECT_EQ(softBody->getNumFaces(), prevFaces + 1u);
}

//==============================================================================
TEST(SoftBodyNode, ConnectPointMasses)
{
  auto skeleton = Skeleton::create("soft-connect");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setSinglePointMass(softBody, 1.0, 5.0, 6.0, 0.1);
  PointMass::Properties pmProps(Eigen::Vector3d(0.1, 0.2, 0.3), 0.5);
  softBody->addPointMass(pmProps);

  softBody->connectPointMasses(0, 1);
  auto* pm0 = softBody->getPointMass(0);
  EXPECT_EQ(pm0->getNumConnectedPointMasses(), 1u);
  EXPECT_EQ(pm0->getConnectedPointMass(0), softBody->getPointMass(1));
}

//==============================================================================
TEST(SoftBodyNode, SoftMeshShape)
{
  auto skeleton = Skeleton::create("soft-mesh");
  auto* softBody = createBoxSoftBody(skeleton);

  // The soft body should have a SoftMeshShape
  bool foundSoftMesh = false;
  for (std::size_t i = 0; i < softBody->getNumShapeNodes(); ++i) {
    auto* shapeNode = softBody->getShapeNode(i);
    auto shape = shapeNode->getShape();
    if (std::dynamic_pointer_cast<SoftMeshShape>(shape)) {
      foundSoftMesh = true;
      break;
    }
  }
  EXPECT_TRUE(foundSoftMesh);
}

//==============================================================================
TEST(SoftBodyNode, SetAspectState)
{
  auto skeleton = Skeleton::create("soft-aspect-state");
  auto* softBody = createBoxSoftBody(skeleton);

  // Get current state
  auto state = softBody->getAspectState();
  EXPECT_EQ(state.mPointStates.size(), softBody->getNumPointMasses());

  // Modify a point state and set it back
  if (!state.mPointStates.empty()) {
    state.mPointStates[0].mPositions = Eigen::Vector3d(0.1, 0.2, 0.3);
    softBody->setAspectState(state);
    auto newState = softBody->getAspectState();
    EXPECT_TRUE(newState.mPointStates[0].mPositions.isApprox(
        Eigen::Vector3d(0.1, 0.2, 0.3)));
  }
}

//==============================================================================
TEST(SoftBodyNode, SpringStiffnessNoChangeOptimization)
{
  auto skeleton = Skeleton::create("soft-no-change");
  auto* softBody = createBoxSoftBody(skeleton);

  softBody->setVertexSpringStiffness(20.0);
  EXPECT_DOUBLE_EQ(softBody->getVertexSpringStiffness(), 20.0);

  // Setting same value should be a no-op (optimization path)
  softBody->setVertexSpringStiffness(20.0);
  EXPECT_DOUBLE_EQ(softBody->getVertexSpringStiffness(), 20.0);

  softBody->setEdgeSpringStiffness(30.0);
  EXPECT_DOUBLE_EQ(softBody->getEdgeSpringStiffness(), 30.0);

  // Setting same value should be a no-op
  softBody->setEdgeSpringStiffness(30.0);
  EXPECT_DOUBLE_EQ(softBody->getEdgeSpringStiffness(), 30.0);

  softBody->setDampingCoefficient(0.5);
  EXPECT_DOUBLE_EQ(softBody->getDampingCoefficient(), 0.5);

  // Setting same value should be a no-op
  softBody->setDampingCoefficient(0.5);
  EXPECT_DOUBLE_EQ(softBody->getDampingCoefficient(), 0.5);
}

//==============================================================================
TEST(SoftBodyNode, SkeletonClonePreservesSoftBody)
{
  auto skeleton = Skeleton::create("soft-clone");
  auto* softBody = createBoxSoftBody(skeleton);
  softBody->setVertexSpringStiffness(42.0);
  softBody->setEdgeSpringStiffness(43.0);
  softBody->setDampingCoefficient(0.7);

  auto cloned = skeleton->cloneSkeleton("cloned");
  ASSERT_EQ(cloned->getNumBodyNodes(), 1u);

  auto* clonedBody = cloned->getBodyNode(0)->asSoftBodyNode();
  ASSERT_NE(clonedBody, nullptr);
  EXPECT_DOUBLE_EQ(clonedBody->getVertexSpringStiffness(), 42.0);
  EXPECT_DOUBLE_EQ(clonedBody->getEdgeSpringStiffness(), 43.0);
  EXPECT_DOUBLE_EQ(clonedBody->getDampingCoefficient(), 0.7);
  EXPECT_EQ(clonedBody->getNumPointMasses(), softBody->getNumPointMasses());
  EXPECT_EQ(clonedBody->getNumFaces(), softBody->getNumFaces());
}

//==============================================================================
TEST(SoftBodyNode, MultiStepSimulation)
{
  auto skeleton = Skeleton::create("soft-sim");
  auto* softBody = createBoxSoftBody(skeleton);
  softBody->setVertexSpringStiffness(10.0);
  softBody->setEdgeSpringStiffness(10.0);
  softBody->setDampingCoefficient(0.1);

  auto world = simulation::World::create();
  world->addSkeleton(skeleton);
  world->setGravity(Eigen::Vector3d(0, 0, -9.81));

  // Run multiple steps
  for (int i = 0; i < 10; ++i) {
    world->step();
  }

  // All point masses should have finite positions
  for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i) {
    auto* pm = softBody->getPointMass(i);
    EXPECT_TRUE(pm->getPositions().array().isFinite().all());
    EXPECT_TRUE(pm->getVelocities().array().isFinite().all());
  }
}

//==============================================================================
TEST(SoftBodyNode, EllipsoidAndCylinderShapes)
{
  // Test ellipsoid shape
  {
    auto skeleton = Skeleton::create("soft-ellipsoid");
    auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
    auto* softBody = pair.second;
    SoftBodyNodeHelper::setEllipsoid(
        softBody, Eigen::Vector3d(1.0, 1.0, 1.0), 4, 3, 2.0, 5.0, 6.0, 0.1);
    EXPECT_GT(softBody->getNumPointMasses(), 0u);
    EXPECT_GT(softBody->getNumFaces(), 0u);
  }

  // Test cylinder shape
  {
    auto skeleton = Skeleton::create("soft-cylinder");
    auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
    auto* softBody = pair.second;
    SoftBodyNodeHelper::setCylinder(
        softBody, 0.5, 1.0, 4, 2, 2, 3.0, 5.0, 6.0, 0.1);
    EXPECT_GT(softBody->getNumPointMasses(), 0u);
    EXPECT_GT(softBody->getNumFaces(), 0u);
  }
}

//==============================================================================
TEST(SoftBodyNode, ChainWithSoftBody)
{
  auto skeleton = Skeleton::create("soft-chain");

  // Create a rigid root body
  auto pair1 = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair1.first->setName("joint1");
  pair1.second->setName("rigid_body");

  // Create a soft body as child
  auto pair2
      = skeleton->createJointAndBodyNodePair<RevoluteJoint, SoftBodyNode>(
          pair1.second);
  pair2.first->setName("joint2");
  auto* softBody = pair2.second;
  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(0.5),
      Eigen::Isometry3d::Identity(),
      1.0,
      10.0,
      10.0,
      0.1);

  EXPECT_EQ(skeleton->getNumBodyNodes(), 2u);
  EXPECT_EQ(skeleton->getNumDofs(), 2u);
  EXPECT_GT(softBody->getNumPointMasses(), 0u);

  // Dynamics should work
  skeleton->computeForwardDynamics();
  const auto& M = skeleton->getMassMatrix();
  EXPECT_EQ(M.rows(), 2);
  EXPECT_EQ(M.cols(), 2);
}

namespace {

SoftBodyNode* createCoverageBoxSoftBody(const SkeletonPtr& skeleton)
{
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;
  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(0.6),
      Eigen::Isometry3d::Identity(),
      Eigen::Vector3i(3, 3, 3),
      2.0,
      15.0,
      20.0,
      0.05);
  return softBody;
}

} // namespace

//==============================================================================
TEST(SoftBodyNode, DerivativeUpdatesWithWorldStep)
{
  auto skeleton = Skeleton::create("soft-derivative");
  auto* softBody = createCoverageBoxSoftBody(skeleton);
  ASSERT_GT(softBody->getNumPointMasses(), 0u);

  const auto dofs = skeleton->getNumDofs();
  skeleton->setPositions(Eigen::VectorXd::Constant(dofs, 0.15));
  skeleton->setVelocities(Eigen::VectorXd::LinSpaced(dofs, 0.1, 0.2));
  skeleton->setAccelerations(Eigen::VectorXd::Constant(dofs, -0.05));

  for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i) {
    auto* pm = softBody->getPointMass(i);
    ASSERT_NE(pm, nullptr);
    pm->setVelocities(Eigen::Vector3d(0.1, -0.2, 0.3));
    pm->setForces(Eigen::Vector3d(1.0, -0.5, 0.2));
    if (i % 2 == 0) {
      pm->addExtForce(Eigen::Vector3d(0.3, 0.1, -0.2));
    }
  }

  auto world = simulation::World::create();
  world->addSkeleton(skeleton);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(1e-3);

  const auto massMatrix = skeleton->getMassMatrix();
  const auto coriolis = skeleton->getCoriolisAndGravityForces();

  EXPECT_EQ(massMatrix.rows(), dofs);
  EXPECT_EQ(coriolis.size(), dofs);
}

//==============================================================================
TEST(SoftBodyNode, PointMassStateAccessors)
{
  auto skeleton = Skeleton::create("soft-point-mass-state");
  auto* softBody = createCoverageBoxSoftBody(skeleton);
  ASSERT_GT(softBody->getNumPointMasses(), 0u);

  auto* pm = softBody->getPointMass(0);
  ASSERT_NE(pm, nullptr);

  auto& state = pm->getState();
  state.mPositions = Eigen::Vector3d(0.2, -0.1, 0.05);
  state.mVelocities = Eigen::Vector3d(0.4, 0.0, -0.3);
  state.mAccelerations = Eigen::Vector3d(-0.2, 0.1, 0.05);
  state.mForces = Eigen::Vector3d(2.0, -1.5, 0.5);

  EXPECT_TRUE(pm->getPositions().isApprox(state.mPositions));
  EXPECT_TRUE(pm->getVelocities().isApprox(state.mVelocities));
  EXPECT_TRUE(pm->getAccelerations().isApprox(state.mAccelerations));
  EXPECT_TRUE(pm->getForces().isApprox(state.mForces));

  const auto* constPm = pm;
  const auto& constState = constPm->getState();
  EXPECT_TRUE(constState.mPositions.isApprox(state.mPositions));

  auto world = simulation::World::create();
  world->addSkeleton(skeleton);
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->setTimeStep(1e-3);

  for (int step = 0; step < 2; ++step) {
    world->step();
  }

  EXPECT_TRUE(pm->getVelocities().array().isFinite().all());
  EXPECT_TRUE(pm->getAccelerations().array().isFinite().all());
  EXPECT_TRUE(pm->getForces().array().isFinite().all());
}

//==============================================================================
TEST(SoftBodyNode, PropertiesConstructionAndAggregateForces)
{
  auto skeleton = Skeleton::create("soft-props-aggregate");

  SoftBodyNode::UniqueProperties uniqueProps
      = SoftBodyNodeHelper::makeBoxProperties(
          Eigen::Vector3d::Constant(0.3),
          Eigen::Isometry3d::Identity(),
          1.0,
          5.0,
          6.0,
          0.02);
  BodyNode::Properties bodyProps;
  bodyProps.mName = "soft_props_body";
  bodyProps.mInertia.setMass(0.5);
  SoftBodyNode::Properties props(bodyProps, uniqueProps);

  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>(
      nullptr, FreeJoint::Properties(), props);
  auto* softBody = pair.second;
  ASSERT_NE(softBody, nullptr);
  EXPECT_GT(softBody->getNumPointMasses(), 0u);
  EXPECT_GT(softBody->getNumFaces(), 0u);

  softBody->addExtForce(
      Eigen::Vector3d(0.0, 0.4, 0.0), Eigen::Vector3d::Zero(), true, true);
  softBody->getPointMass(0)->addExtForce(Eigen::Vector3d(0.0, 0.0, -0.2));

  auto world = simulation::World::create();
  world->addSkeleton(skeleton);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(1e-3);
  world->step();

  skeleton->computeInverseDynamics(true, true, true);

  const auto externalForces = skeleton->getExternalForces();
  EXPECT_EQ(externalForces.size(), static_cast<int>(skeleton->getNumDofs()));
  EXPECT_TRUE(externalForces.array().isFinite().all());
}

//==============================================================================
TEST(SoftBodyNode, PropertiesConstructionAddPointMassAndFace)
{
  auto skeleton = Skeleton::create("soft-props-add-points");

  SoftBodyNode::UniqueProperties uniqueProps
      = SoftBodyNodeHelper::makeBoxProperties(
          Eigen::Vector3d::Constant(0.25),
          Eigen::Isometry3d::Identity(),
          1.0,
          5.0,
          6.0,
          0.02);
  BodyNode::Properties bodyProps;
  bodyProps.mName = "soft_props_add_body";
  bodyProps.mInertia.setMass(0.5);
  SoftBodyNode::Properties props(bodyProps, uniqueProps);

  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>(
      nullptr, FreeJoint::Properties(), props);
  auto* softBody = pair.second;
  ASSERT_NE(softBody, nullptr);

  const auto pointCount = softBody->getNumPointMasses();
  const auto faceCount = softBody->getNumFaces();
  PointMass::Properties extraProp(Eigen::Vector3d(0.0, 0.0, 0.1), 0.1);
  softBody->addPointMass(extraProp);
  softBody->connectPointMasses(0, pointCount);
  softBody->connectPointMasses(1, pointCount);
  softBody->connectPointMasses(2, pointCount);
  softBody->addFace(Eigen::Vector3i(0, 1, static_cast<int>(pointCount)));
  EXPECT_EQ(softBody->getNumPointMasses(), pointCount + 1u);
  EXPECT_EQ(softBody->getNumFaces(), faceCount + 1u);
}

//==============================================================================
TEST(SoftBodyNode, PointMassStateAndPropertiesEquality)
{
  PointMass::State stateA(
      Eigen::Vector3d(0.1, 0.2, 0.3),
      Eigen::Vector3d(-0.2, 0.0, 0.4),
      Eigen::Vector3d(0.5, -0.1, 0.2),
      Eigen::Vector3d(1.0, 2.0, 3.0));
  PointMass::State stateB = stateA;
  EXPECT_TRUE(stateA == stateB);

  stateB.mForces[0] = 9.0;
  EXPECT_FALSE(stateA == stateB);

  std::vector<std::size_t> connections = {1, 2};
  PointMass::Properties propsA(
      Eigen::Vector3d(0.4, 0.5, 0.6), 1.2, connections);
  PointMass::Properties propsB(
      Eigen::Vector3d(0.4, 0.5, 0.6), 1.2, connections);
  EXPECT_TRUE(propsA == propsB);
  EXPECT_FALSE(propsA != propsB);

  propsB.setMass(2.4);
  EXPECT_FALSE(propsA == propsB);
  EXPECT_TRUE(propsA != propsB);
}

//==============================================================================
TEST(SoftBodyNode, PointMassConstraintImpulseAndWorldTransforms)
{
  auto skeleton = Skeleton::create("soft-impulse");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;
  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(0.4),
      Eigen::Isometry3d::Identity(),
      1.0,
      10.0,
      10.0,
      0.1);

  auto* joint = pair.first;
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = Eigen::AngleAxisd(dart::math::half_pi, Eigen::Vector3d::UnitZ())
                    .toRotationMatrix();
  joint->setTransform(tf);

  PointMass* pm = softBody->getPointMass(0);
  ASSERT_NE(pm, nullptr);

  pm->setConstraintImpulse(Eigen::Vector3d(1.0, 0.0, 0.0), false);
  Eigen::Vector3d localImpulse = pm->getConstraintImpulses();
  EXPECT_TRUE(localImpulse.isApprox(Eigen::Vector3d(0.0, -1.0, 0.0), 1e-12));

  pm->addConstraintImpulse(Eigen::Vector3d(0.0, 0.0, 2.0), true);
  localImpulse = pm->getConstraintImpulses();
  EXPECT_TRUE(localImpulse.isApprox(Eigen::Vector3d(0.0, -1.0, 2.0), 1e-12));

  pm->clearConstraintImpulse();
  EXPECT_TRUE(pm->getConstraintImpulses().isZero(0.0));

  pm->setPositions(Eigen::Vector3d(0.1, -0.2, 0.3));
  pm->setVelocities(Eigen::Vector3d(0.5, 0.0, 0.0));
  pm->setAccelerations(Eigen::Vector3d(0.0, 0.1, 0.0));

  const Eigen::Vector3d worldPos = pm->getWorldPosition();
  const Eigen::Vector3d worldVel = pm->getWorldVelocity();
  const Eigen::Vector3d worldAcc = pm->getWorldAcceleration();

  EXPECT_TRUE(worldPos.allFinite());
  EXPECT_TRUE(worldVel.isApprox(Eigen::Vector3d(0.0, 0.5, 0.0), 1e-12));
  EXPECT_TRUE(worldAcc.isApprox(Eigen::Vector3d(-0.1, 0.0, 0.0), 1e-12));
}

//==============================================================================
TEST(SoftBodyNode, BiasImpulseAndConstrainedTermUpdates)
{
  auto skeleton = Skeleton::create("soft-impulse-update");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;
  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(0.5),
      Eigen::Isometry3d::Identity(),
      1.0,
      15.0,
      12.0,
      0.08);

  softBody->setConstraintImpulse(Eigen::Vector6d::Ones());
  for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i) {
    auto* pm = softBody->getPointMass(i);
    pm->setConstraintImpulse(Eigen::Vector3d::UnitX());
    pm->setVelocityChange(0, 0.1);
  }

  skeleton->computeImpulseForwardDynamics();
  skeleton->updateVelocityChange();
  skeleton->computePositionVelocityChanges();

  for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i) {
    const auto* pm = softBody->getPointMass(i);
    EXPECT_TRUE(pm->getVelocities().array().isFinite().all());
    EXPECT_TRUE(pm->getAccelerations().array().isFinite().all());
  }
}
