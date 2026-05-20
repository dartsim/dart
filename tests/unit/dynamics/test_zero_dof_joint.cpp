// Copyright (c) 2011, The DART development contributors

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/invalid_index.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/dart.hpp>

#include <gtest/gtest.h>

#include <string>

using namespace dart;
using namespace dart::dynamics;

TEST(ZeroDofJoint, BasicAccessors)
{
  auto skeleton = Skeleton::create("zero_dof");
  auto pair = skeleton->createJointAndBodyNodePair<WeldJoint, BodyNode>();
  auto* joint = pair.first;

  EXPECT_EQ(joint->getNumDofs(), 0u);
  EXPECT_EQ(joint->getCommands().size(), 0);
  EXPECT_EQ(joint->getPositions().size(), 0);
  EXPECT_EQ(joint->getVelocities().size(), 0);
  EXPECT_EQ(joint->getAccelerations().size(), 0);
  EXPECT_EQ(joint->getForces().size(), 0);

  joint->setCommands(Eigen::VectorXd::Zero(0));
  joint->setPositions(Eigen::VectorXd::Zero(0));
  joint->setVelocities(Eigen::VectorXd::Zero(0));
  joint->setAccelerations(Eigen::VectorXd::Zero(0));
  joint->setForces(Eigen::VectorXd::Zero(0));

  EXPECT_DOUBLE_EQ(joint->getCommand(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getPosition(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getVelocity(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getAcceleration(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getForce(0), 0.0);

  Eigen::VectorXd result;
  joint->integratePositions(
      Eigen::VectorXd::Zero(0), Eigen::VectorXd::Zero(0), 0.01, result);
  EXPECT_EQ(result.size(), 0);

  EXPECT_DOUBLE_EQ(joint->computePotentialEnergy(), 0.0);
  EXPECT_TRUE(
      joint->getBodyConstraintWrench().isApprox(pair.second->getBodyForce()));

  EXPECT_EQ(
      skeleton->getIndexOf(static_cast<const BodyNode*>(nullptr), false),
      INVALID_INDEX);
}

TEST(ZeroDofJoint, LimitsAndStateHelpers)
{
  auto skeleton = Skeleton::create("zero_dof_limits");
  auto pair = skeleton->createJointAndBodyNodePair<WeldJoint, BodyNode>();
  auto* joint = pair.first;

  joint->setCommand(0, 1.0);
  joint->setCommands(Eigen::VectorXd::Zero(0));
  EXPECT_EQ(joint->getCommands().size(), 0);
  joint->resetCommands();

  joint->setPositionLowerLimit(0, -1.0);
  EXPECT_DOUBLE_EQ(joint->getPositionLowerLimit(0), 0.0);
  joint->setPositionLowerLimits(Eigen::VectorXd::Zero(0));
  EXPECT_EQ(joint->getPositionLowerLimits().size(), 0);

  joint->setPositionUpperLimit(0, 1.0);
  EXPECT_DOUBLE_EQ(joint->getPositionUpperLimit(0), 0.0);
  joint->setPositionUpperLimits(Eigen::VectorXd::Zero(0));
  EXPECT_EQ(joint->getPositionUpperLimits().size(), 0);

  EXPECT_TRUE(joint->hasPositionLimit(0));
  joint->resetPosition(0);
  joint->resetPositions();
  joint->setInitialPosition(0, 0.0);
  EXPECT_DOUBLE_EQ(joint->getInitialPosition(0), 0.0);
  joint->setInitialPositions(Eigen::VectorXd::Zero(0));
  EXPECT_EQ(joint->getInitialPositions().size(), 0);

  joint->setVelocityLowerLimit(0, -1.0);
  EXPECT_DOUBLE_EQ(joint->getVelocityLowerLimit(0), 0.0);
  joint->setVelocityLowerLimits(Eigen::VectorXd::Zero(0));
  EXPECT_EQ(joint->getVelocityLowerLimits().size(), 0);

  joint->setVelocityUpperLimit(0, 1.0);
  EXPECT_DOUBLE_EQ(joint->getVelocityUpperLimit(0), 0.0);
  joint->setVelocityUpperLimits(Eigen::VectorXd::Zero(0));
  EXPECT_EQ(joint->getVelocityUpperLimits().size(), 0);
  joint->resetVelocity(0);
  joint->resetVelocities();
  joint->setInitialVelocity(0, 0.0);
  EXPECT_DOUBLE_EQ(joint->getInitialVelocity(0), 0.0);
  joint->setInitialVelocities(Eigen::VectorXd::Zero(0));
  EXPECT_EQ(joint->getInitialVelocities().size(), 0);

  joint->setAccelerationLowerLimit(0, -1.0);
  EXPECT_DOUBLE_EQ(joint->getAccelerationLowerLimit(0), 0.0);
  joint->setAccelerationLowerLimits(Eigen::VectorXd::Zero(0));
  EXPECT_EQ(joint->getAccelerationLowerLimits().size(), 0);

  joint->setAccelerationUpperLimit(0, 1.0);
  EXPECT_DOUBLE_EQ(joint->getAccelerationUpperLimit(0), 0.0);
  joint->setAccelerationUpperLimits(Eigen::VectorXd::Zero(0));
  EXPECT_EQ(joint->getAccelerationUpperLimits().size(), 0);
  joint->resetAccelerations();

  joint->setForceLowerLimit(0, -1.0);
  EXPECT_DOUBLE_EQ(joint->getForceLowerLimit(0), 0.0);
  joint->setForceLowerLimits(Eigen::VectorXd::Zero(0));
  EXPECT_EQ(joint->getForceLowerLimits().size(), 0);

  joint->setForceUpperLimit(0, 1.0);
  EXPECT_DOUBLE_EQ(joint->getForceUpperLimit(0), 0.0);
  joint->setForceUpperLimits(Eigen::VectorXd::Zero(0));
  EXPECT_EQ(joint->getForceUpperLimits().size(), 0);
  joint->resetForces();

  joint->setVelocityChange(0, 0.0);
  EXPECT_DOUBLE_EQ(joint->getVelocityChange(0), 0.0);
  joint->resetVelocityChanges();

  joint->setConstraintImpulse(0, 0.0);
  EXPECT_DOUBLE_EQ(joint->getConstraintImpulse(0), 0.0);
  joint->resetConstraintImpulses();

  joint->integratePositions(0.01);
  joint->integrateVelocities(0.01);
  const auto diff = joint->getPositionDifferences(
      Eigen::VectorXd::Zero(0), Eigen::VectorXd::Zero(0));
  EXPECT_EQ(diff.size(), 0);

  joint->setSpringStiffness(0, 1.0);
  EXPECT_DOUBLE_EQ(joint->getSpringStiffness(0), 0.0);
  joint->setRestPosition(0, 0.0);
  EXPECT_DOUBLE_EQ(joint->getRestPosition(0), 0.0);
  joint->setDampingCoefficient(0, 1.0);
  EXPECT_DOUBLE_EQ(joint->getDampingCoefficient(0), 0.0);
  joint->setCoulombFriction(0, 0.5);
  EXPECT_DOUBLE_EQ(joint->getCoulombFriction(0), 0.0);

  skeleton->computeForwardDynamics();
  skeleton->computeInverseDynamics(true, true, true);
  const auto massMatrix = skeleton->getMassMatrix();
  EXPECT_EQ(massMatrix.rows(), 0);
  EXPECT_EQ(massMatrix.cols(), 0);
}

namespace {

template <typename JointType>
SkeletonPtr createSkeletonWithJoint(const std::string& name)
{
  auto skel = Skeleton::create(name);

  BodyNode::Properties bodyProps;
  bodyProps.mName = name + "_body";
  bodyProps.mInertia.setMass(1.0);

  typename JointType::Properties jointProps;
  jointProps.mName = name + "_joint";

  skel->createJointAndBodyNodePair<JointType>(nullptr, jointProps, bodyProps);

  return skel;
}

template <typename JointType>
JointType* getJoint(const SkeletonPtr& skel)
{
  return static_cast<JointType*>(skel->getJoint(0));
}

class ZeroDofJointHarness final : public WeldJoint
{
public:
  using Properties = WeldJoint::Properties;

  explicit ZeroDofJointHarness(const Properties& properties)
    : WeldJoint(properties)
  {
    // Do nothing
  }

  using ZeroDofJoint::addAccelerationTo;
  using ZeroDofJoint::addChildArtInertiaImplicitTo;
  using ZeroDofJoint::addChildArtInertiaTo;
  using ZeroDofJoint::addChildBiasForceForInvAugMassMatrix;
  using ZeroDofJoint::addChildBiasForceForInvMassMatrix;
  using ZeroDofJoint::addInvMassMatrixSegmentTo;
  using ZeroDofJoint::addVelocityChangeTo;
  using ZeroDofJoint::addVelocityTo;
  using ZeroDofJoint::getInvAugMassMatrixSegment;
  using ZeroDofJoint::getInvMassMatrixSegment;
  using ZeroDofJoint::getRelativeJacobian;
  using ZeroDofJoint::getRelativeJacobianTimeDeriv;
  using ZeroDofJoint::getSpatialToGeneralized;
  using ZeroDofJoint::registerDofs;
  using ZeroDofJoint::setPartialAccelerationTo;
  using ZeroDofJoint::updateAcceleration;
  using ZeroDofJoint::updateConstrainedTerms;
  using ZeroDofJoint::updateDegreeOfFreedomNames;
  using ZeroDofJoint::updateForceFD;
  using ZeroDofJoint::updateForceID;
  using ZeroDofJoint::updateImpulseFD;
  using ZeroDofJoint::updateImpulseID;
  using ZeroDofJoint::updateInvProjArtInertia;
  using ZeroDofJoint::updateInvProjArtInertiaImplicit;
  using ZeroDofJoint::updateTotalForceForInvMassMatrix;
  using ZeroDofJoint::updateVelocityChange;
};

} // namespace

TEST(ZeroDofJoint, ZeroDofAccessorsReturnEmpty)
{
  auto skel = createSkeletonWithJoint<WeldJoint>("zero_dof_coverage");
  auto* joint = getJoint<WeldJoint>(skel);

  EXPECT_EQ(joint->getNumDofs(), 0u);
  EXPECT_EQ(joint->getDofName(0), "");
  EXPECT_FALSE(joint->isDofNamePreserved(0));
  joint->preserveDofName(0, true);
  EXPECT_FALSE(joint->isDofNamePreserved(0));
  EXPECT_EQ(joint->setDofName(0, "ignored", true), "");
  // getIndexInSkeleton/getIndexInTree assert(false) in Debug for ZeroDofJoint

  joint->setCommand(0, 1.0);
  joint->setPosition(0, 2.0);
  joint->setVelocity(0, 3.0);
  joint->setAcceleration(0, 4.0);
  joint->setForce(0, 5.0);

  EXPECT_DOUBLE_EQ(joint->getCommand(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getPosition(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getVelocity(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getAcceleration(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getForce(0), 0.0);

  EXPECT_EQ(joint->getCommands().size(), 0);
  EXPECT_EQ(joint->getPositions().size(), 0);
  EXPECT_EQ(joint->getVelocities().size(), 0);
  EXPECT_EQ(joint->getAccelerations().size(), 0);
  EXPECT_EQ(joint->getForces().size(), 0);

  EXPECT_EQ(joint->getPositionLowerLimits().size(), 0);
  EXPECT_EQ(joint->getPositionUpperLimits().size(), 0);
  EXPECT_EQ(joint->getVelocityLowerLimits().size(), 0);
  EXPECT_EQ(joint->getVelocityUpperLimits().size(), 0);
  EXPECT_EQ(joint->getAccelerationLowerLimits().size(), 0);
  EXPECT_EQ(joint->getAccelerationUpperLimits().size(), 0);
  EXPECT_EQ(joint->getForceLowerLimits().size(), 0);
  EXPECT_EQ(joint->getForceUpperLimits().size(), 0);

  EXPECT_DOUBLE_EQ(joint->getSpringStiffness(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getRestPosition(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getDampingCoefficient(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getCoulombFriction(0), 0.0);
}

TEST(ZeroDofJoint, ConstAccessorsAndMatrices)
{
  auto skel = createSkeletonWithJoint<WeldJoint>("zero_dof_const");
  auto* joint = getJoint<WeldJoint>(skel);
  const Joint& constJoint = *joint;

  EXPECT_EQ(constJoint.getNumDofs(), 0u);
  EXPECT_EQ(constJoint.getCommands().size(), 0);
  EXPECT_EQ(constJoint.getPositions().size(), 0);
  EXPECT_EQ(constJoint.getVelocities().size(), 0);
  EXPECT_EQ(constJoint.getAccelerations().size(), 0);
  EXPECT_EQ(constJoint.getForces().size(), 0);

  EXPECT_EQ(constJoint.getPositionLowerLimits().size(), 0);
  EXPECT_EQ(constJoint.getPositionUpperLimits().size(), 0);
  EXPECT_EQ(constJoint.getVelocityLowerLimits().size(), 0);
  EXPECT_EQ(constJoint.getVelocityUpperLimits().size(), 0);
  EXPECT_EQ(constJoint.getAccelerationLowerLimits().size(), 0);
  EXPECT_EQ(constJoint.getAccelerationUpperLimits().size(), 0);
  EXPECT_EQ(constJoint.getForceLowerLimits().size(), 0);
  EXPECT_EQ(constJoint.getForceUpperLimits().size(), 0);

  skel->computeForwardDynamics();
  skel->computeImpulseForwardDynamics();

  const auto invMass = skel->getInvMassMatrix();
  EXPECT_EQ(invMass.rows(), 0);
  EXPECT_EQ(invMass.cols(), 0);

  const auto invAug = skel->getInvAugMassMatrix();
  EXPECT_EQ(invAug.rows(), 0);
  EXPECT_EQ(invAug.cols(), 0);
}

TEST(ZeroDofJoint, RecursiveHooksRemainNoOpsForZeroDofJoint)
{
  auto skel = createSkeletonWithJoint<ZeroDofJointHarness>(
      "zero_dof_recursive_hooks");
  auto* joint = getJoint<ZeroDofJointHarness>(skel);

  joint->registerDofs();
  joint->updateDegreeOfFreedomNames();

  EXPECT_EQ(joint->getRelativeJacobian().rows(), 6);
  EXPECT_EQ(joint->getRelativeJacobian().cols(), 0);
  EXPECT_EQ(joint->getRelativeJacobian(Eigen::VectorXd::Zero(0)).rows(), 6);
  EXPECT_EQ(joint->getRelativeJacobian(Eigen::VectorXd::Zero(0)).cols(), 0);
  EXPECT_EQ(joint->getRelativeJacobianTimeDeriv().rows(), 6);
  EXPECT_EQ(joint->getRelativeJacobianTimeDeriv().cols(), 0);

  Eigen::Vector6d spatial = Eigen::Vector6d::LinSpaced(1.0, 6.0);
  const Eigen::Vector6d originalSpatial = spatial;
  joint->addVelocityTo(spatial);
  EXPECT_TRUE(spatial.isApprox(originalSpatial));
  joint->addVelocityChangeTo(spatial);
  EXPECT_TRUE(spatial.isApprox(originalSpatial));
  joint->addAccelerationTo(spatial);
  EXPECT_TRUE(spatial.isApprox(originalSpatial));

  Eigen::Vector6d partial = Eigen::Vector6d::Ones();
  joint->setPartialAccelerationTo(partial, originalSpatial);
  EXPECT_TRUE(partial.isZero());

  Eigen::Matrix6d parentArtInertia = Eigen::Matrix6d::Zero();
  const Eigen::Matrix6d childArtInertia = Eigen::Matrix6d::Identity();
  joint->addChildArtInertiaTo(parentArtInertia, childArtInertia);
  EXPECT_TRUE(parentArtInertia.isApprox(childArtInertia));

  parentArtInertia.setZero();
  joint->addChildArtInertiaImplicitTo(parentArtInertia, childArtInertia);
  EXPECT_TRUE(parentArtInertia.isApprox(childArtInertia));

  joint->updateInvProjArtInertia(childArtInertia);
  joint->updateInvProjArtInertiaImplicit(childArtInertia, 0.01);
  joint->updateAcceleration(childArtInertia, originalSpatial);
  joint->updateVelocityChange(childArtInertia, originalSpatial);
  joint->updateForceID(originalSpatial, 0.01, true, true);
  joint->updateForceFD(originalSpatial, 0.01, true, true);
  joint->updateImpulseID(originalSpatial);
  joint->updateImpulseFD(originalSpatial);
  joint->updateConstrainedTerms(0.01);

  Eigen::Vector6d parentBias = originalSpatial;
  joint->addChildBiasForceForInvMassMatrix(
      parentBias, childArtInertia, Eigen::Vector6d::Ones());
  EXPECT_TRUE(parentBias.isApprox(originalSpatial));
  joint->addChildBiasForceForInvAugMassMatrix(
      parentBias, childArtInertia, Eigen::Vector6d::Ones());
  EXPECT_TRUE(parentBias.isApprox(originalSpatial));
  joint->updateTotalForceForInvMassMatrix(originalSpatial);

  Eigen::MatrixXd invMass = Eigen::MatrixXd::Constant(2, 2, 3.0);
  const Eigen::MatrixXd originalInvMass = invMass;
  joint->getInvMassMatrixSegment(invMass, 0, childArtInertia, originalSpatial);
  EXPECT_TRUE(invMass.isApprox(originalInvMass));
  joint->getInvAugMassMatrixSegment(
      invMass, 0, childArtInertia, originalSpatial);
  EXPECT_TRUE(invMass.isApprox(originalInvMass));

  spatial = originalSpatial;
  joint->addInvMassMatrixSegmentTo(spatial);
  EXPECT_TRUE(spatial.isApprox(originalSpatial));

  EXPECT_EQ(joint->getSpatialToGeneralized(originalSpatial).size(), 0);
}
