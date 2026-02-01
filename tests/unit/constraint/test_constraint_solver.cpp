/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD-style License.
 */

#include "helpers/dynamics_helpers.hpp"

#include "helpers/gtest_utils.hpp"

#include "dart/collision/dart/dart_collision_detector.hpp"
#include "dart/constraint/constraint_solver.hpp"
#include "dart/constraint/contact_constraint.hpp"
#include "dart/constraint/contact_surface.hpp"
#include "dart/constraint/coupler_constraint.hpp"
#include "dart/constraint/revolute_joint_constraint.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/mimic_dof_properties.hpp"
#include "dart/dynamics/revolute_joint.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/math/lcp/lcp_solver.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"
#include "dart/math/lcp/projection/pgs_solver.hpp"
#include "dart/simulation/world.hpp"

#include <dart/dart.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <type_traits>
#include <vector>

#include <cmath>

using namespace dart;
using namespace dart::simulation;
using namespace dart::test;

namespace {

// Minimal constraint used to exercise the LCP pipeline.
class DummyConstraint : public constraint::ConstraintBase
{
public:
  explicit DummyConstraint(dynamics::SkeletonPtr skeleton)
    : mSkeleton(std::move(skeleton))
  {
    mDim = 1;
  }

  void update() override {}

  void getInformation(constraint::ConstraintInfo* info) override
  {
    info->lo[0] = 0.0;
    info->hi[0] = 1.0;
    info->b[0] = 0.0;
    info->w[0] = 0.0;
    info->findex[0] = -1;
  }

  void applyUnitImpulse(std::size_t) override {}

  void getVelocityChange(double* vel, bool) override
  {
    vel[0] = 1.0;
  }

  void excite() override {}
  void unexcite() override {}

  void applyImpulse(double* lambda) override
  {
    lastAppliedImpulse = lambda[0];
  }

  bool isActive() const override
  {
    return true;
  }

  dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return mSkeleton;
  }

  double lastAppliedImpulse{std::numeric_limits<double>::quiet_NaN()};

private:
  dynamics::SkeletonPtr mSkeleton;
};

class PhaseTrackingConstraint : public constraint::ConstraintBase
{
public:
  explicit PhaseTrackingConstraint(dynamics::SkeletonPtr skeleton)
    : mSkeleton(std::move(skeleton))
  {
    mDim = 1;
  }

  void update() override {}

  void getInformation(constraint::ConstraintInfo* info) override
  {
    phases.push_back(info->phase);
    useSplitImpulse.push_back(info->useSplitImpulse);
    ++getInformationCalls;

    info->lo[0] = 0.0;
    info->hi[0] = 0.0;
    info->b[0] = 0.0;
    info->w[0] = 0.0;
    info->findex[0] = -1;
  }

  void applyUnitImpulse(std::size_t) override {}

  void getVelocityChange(double* vel, bool) override
  {
    vel[0] = 1.0;
  }

  void excite() override {}
  void unexcite() override {}
  void applyImpulse(double*) override {}

  bool isActive() const override
  {
    return true;
  }

  dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return mSkeleton;
  }

  std::vector<constraint::ConstraintPhase> phases;
  std::vector<bool> useSplitImpulse;
  std::size_t getInformationCalls{0};

private:
  dynamics::SkeletonPtr mSkeleton;
};

class NanLcpSolver : public math::LcpSolver
{
public:
  math::LcpResult solve(
      const math::LcpProblem& problem,
      Eigen::VectorXd& x,
      const math::LcpOptions&) override
  {
    x = Eigen::VectorXd::Constant(
        problem.A.rows(), std::numeric_limits<double>::quiet_NaN());
    math::LcpResult res;
    res.status = math::LcpSolverStatus::Success;
    return res;
  }

  std::string getName() const override
  {
    return "NanLcpSolver";
  }
  std::string getCategory() const override
  {
    return "Test";
  }
};

class ConstantLcpSolver : public math::LcpSolver
{
public:
  explicit ConstantLcpSolver(double value) : mValue(value) {}

  math::LcpResult solve(
      const math::LcpProblem& problem,
      Eigen::VectorXd& x,
      const math::LcpOptions&) override
  {
    x = Eigen::VectorXd::Constant(problem.A.rows(), mValue);
    math::LcpResult res;
    res.status = math::LcpSolverStatus::Success;
    return res;
  }

  std::string getName() const override
  {
    return "ConstantLcpSolver";
  }
  std::string getCategory() const override
  {
    return "Test";
  }

private:
  double mValue;
};

class ConstantFailingLcpSolver : public math::LcpSolver
{
public:
  explicit ConstantFailingLcpSolver(double value) : mValue(value) {}

  math::LcpResult solve(
      const math::LcpProblem& problem,
      Eigen::VectorXd& x,
      const math::LcpOptions&) override
  {
    x = Eigen::VectorXd::Constant(problem.A.rows(), mValue);
    math::LcpResult res;
    res.status = math::LcpSolverStatus::Failed;
    return res;
  }

  std::string getName() const override
  {
    return "ConstantFailingLcpSolver";
  }
  std::string getCategory() const override
  {
    return "Test";
  }

private:
  double mValue;
};

class ExposedConstraintSolver : public constraint::ConstraintSolver
{
public:
  bool containsConstraint(
      const constraint::ConstConstraintBasePtr& constraint) const
  {
    return containConstraint(constraint);
  }

  bool checkAndAddSkeletonPublic(const dynamics::SkeletonPtr& skeleton)
  {
    return checkAndAddSkeleton(skeleton);
  }

  bool checkAndAddConstraintPublic(const constraint::ConstraintBasePtr& con)
  {
    return checkAndAddConstraint(con);
  }

  bool hasSkeletonPublic(const dynamics::ConstSkeletonPtr& skeleton) const
  {
    return hasSkeleton(skeleton);
  }
};

class CouplerTrackingSolver final : public constraint::ConstraintSolver
{
public:
  using ConstraintSolver::updateConstraints;

  std::size_t getNumCouplerConstraints() const
  {
    return mCouplerConstraints.size();
  }

  std::size_t getNumMimicMotorConstraints() const
  {
    return mMimicMotorConstraints.size();
  }
};

class PartialNanLcpSolver : public math::LcpSolver
{
public:
  explicit PartialNanLcpSolver(double value) : mValue(value) {}

  math::LcpResult solve(
      const math::LcpProblem& problem,
      Eigen::VectorXd& x,
      const math::LcpOptions&) override
  {
    x = Eigen::VectorXd::Constant(problem.A.rows(), mValue);
    x[0] = std::numeric_limits<double>::quiet_NaN();
    math::LcpResult res;
    res.status = math::LcpSolverStatus::Failed;
    return res;
  }

  std::string getName() const override
  {
    return "PartialNanLcpSolver";
  }
  std::string getCategory() const override
  {
    return "Test";
  }

private:
  double mValue;
};

class FailingLcpSolver : public math::LcpSolver
{
public:
  math::LcpResult solve(
      const math::LcpProblem&,
      Eigen::VectorXd&,
      const math::LcpOptions&) override
  {
    math::LcpResult res;
    res.status = math::LcpSolverStatus::Failed;
    return res;
  }

  std::string getName() const override
  {
    return "FailingLcpSolver";
  }
  std::string getCategory() const override
  {
    return "Test";
  }
};

class CoverageConstraintSolver final : public constraint::ConstraintSolver
{
public:
  using ConstraintSolver::isSymmetric;
  using ConstraintSolver::print;
};

} // namespace

//==============================================================================
TEST(ConstraintSolver, SecondarySolverUsedWhenPrimaryFails)
{
  constraint::ConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("dummy");
  solver.addSkeleton(skeleton);
  solver.addConstraint(std::make_shared<DummyConstraint>(skeleton));

  solver.setLcpSolver(std::make_shared<FailingLcpSolver>());
  solver.setSecondaryLcpSolver(std::make_shared<ConstantLcpSolver>(0.5));

  solver.solve();

  auto constraint
      = std::dynamic_pointer_cast<DummyConstraint>(solver.getConstraint(0));
  ASSERT_TRUE(constraint);
  EXPECT_DOUBLE_EQ(constraint->lastAppliedImpulse, 0.5);
}

//==============================================================================
TEST(ConstraintSolver, RemoveSkeletonErasesEntry)
{
  constraint::ConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("dummy");
  solver.addSkeleton(skeleton);

  ASSERT_EQ(solver.getSkeletons().size(), 1u);
  solver.removeSkeleton(skeleton);
  EXPECT_EQ(solver.getSkeletons().size(), 0u);
}

//==============================================================================
TEST(ConstraintSolver, SecondarySolverUsedWhenPrimaryReturnsNan)
{
  constraint::ConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("dummy");
  solver.addSkeleton(skeleton);
  solver.addConstraint(std::make_shared<DummyConstraint>(skeleton));

  solver.setLcpSolver(std::make_shared<NanLcpSolver>());
  solver.setSecondaryLcpSolver(std::make_shared<ConstantLcpSolver>(0.7));

  solver.solve();

  auto constraint
      = std::dynamic_pointer_cast<DummyConstraint>(solver.getConstraint(0));
  ASSERT_TRUE(constraint);
  EXPECT_DOUBLE_EQ(constraint->lastAppliedImpulse, 0.7);
}

//==============================================================================
TEST(ConstraintSolver, SecondaryIgnoredWhenPrimarySucceeds)
{
  constraint::ConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("dummy");
  solver.addSkeleton(skeleton);
  solver.addConstraint(std::make_shared<DummyConstraint>(skeleton));

  solver.setLcpSolver(std::make_shared<ConstantLcpSolver>(0.3));
  solver.setSecondaryLcpSolver(std::make_shared<ConstantLcpSolver>(1.0));

  solver.solve();

  auto constraint
      = std::dynamic_pointer_cast<DummyConstraint>(solver.getConstraint(0));
  ASSERT_TRUE(constraint);
  EXPECT_DOUBLE_EQ(constraint->lastAppliedImpulse, 0.3);
}

//==============================================================================
TEST(ConstraintSolver, PartialNanHandled)
{
  constraint::ConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("dummy");
  solver.addSkeleton(skeleton);
  solver.addConstraint(std::make_shared<DummyConstraint>(skeleton));

  solver.setLcpSolver(std::make_shared<PartialNanLcpSolver>(0.8));
  solver.setSecondaryLcpSolver(nullptr);

  solver.solve();

  auto constraint
      = std::dynamic_pointer_cast<DummyConstraint>(solver.getConstraint(0));
  ASSERT_TRUE(constraint);
  EXPECT_DOUBLE_EQ(constraint->lastAppliedImpulse, 0.0); // sanitized
}

//==============================================================================
TEST(ConstraintSolver, SplitImpulseSkipsNonContactPositionPass)
{
  constraint::ConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("dummy");
  solver.addSkeleton(skeleton);
  auto constraint = std::make_shared<PhaseTrackingConstraint>(skeleton);
  solver.addConstraint(constraint);

  solver.setLcpSolver(std::make_shared<ConstantLcpSolver>(0.0));
  solver.setSecondaryLcpSolver(nullptr);
  solver.setSplitImpulseEnabled(true);

  solver.solve();

  ASSERT_EQ(constraint->getInformationCalls, 1u);
  ASSERT_EQ(constraint->phases.size(), 1u);
  EXPECT_EQ(constraint->phases[0], constraint::ConstraintPhase::Velocity);
  ASSERT_EQ(constraint->useSplitImpulse.size(), 1u);
  EXPECT_TRUE(constraint->useSplitImpulse[0]);
}

//==============================================================================
TEST(ConstraintSolver, SolveWithNoSkeletonsDoesNothing)
{
  constraint::ConstraintSolver solver;

  EXPECT_EQ(solver.getSkeletons().size(), 0u);
  EXPECT_NO_THROW(solver.solve());
}

//==============================================================================
TEST(ConstraintSolver, AddDuplicateSkeletonIgnored)
{
  constraint::ConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("duplicate_test");

  solver.addSkeleton(skeleton);
  ASSERT_EQ(solver.getSkeletons().size(), 1u);

  solver.addSkeleton(skeleton);
  EXPECT_EQ(solver.getSkeletons().size(), 1u);
}

//==============================================================================
TEST(ConstraintSolver, RemoveNonExistentSkeletonNoOp)
{
  constraint::ConstraintSolver solver;
  auto skeleton1 = dynamics::Skeleton::create("skeleton1");
  auto skeleton2 = dynamics::Skeleton::create("skeleton2");

  solver.addSkeleton(skeleton1);
  ASSERT_EQ(solver.getSkeletons().size(), 1u);

  EXPECT_NO_THROW(solver.removeSkeleton(skeleton2));
  EXPECT_EQ(solver.getSkeletons().size(), 1u);
}

//==============================================================================
TEST(ConstraintSolver, SetTimeStepAffectsSolver)
{
  constraint::ConstraintSolver solver;

  EXPECT_DOUBLE_EQ(solver.getTimeStep(), 0.001);

  solver.setTimeStep(0.002);
  EXPECT_DOUBLE_EQ(solver.getTimeStep(), 0.002);

  solver.setTimeStep(0.0001);
  EXPECT_DOUBLE_EQ(solver.getTimeStep(), 0.0001);
}

//==============================================================================
TEST(ConstraintSolver, SetNullCollisionDetectorIgnored)
{
  constraint::ConstraintSolver solver;

  auto originalDetector = solver.getCollisionDetector();
  ASSERT_NE(originalDetector, nullptr);

  solver.setCollisionDetector(nullptr);
  EXPECT_EQ(solver.getCollisionDetector(), originalDetector);
}

//==============================================================================
TEST(ConstraintSolver, ContactSurfaceHandlerChain)
{
  constraint::ConstraintSolver solver;
  auto defaultHandler = solver.getLastContactSurfaceHandler();
  ASSERT_NE(defaultHandler, nullptr);

  auto handlerA = std::make_shared<constraint::ContactSurfaceHandler>();
  solver.addContactSurfaceHandler(handlerA);
  EXPECT_EQ(solver.getLastContactSurfaceHandler(), handlerA);
  EXPECT_EQ(handlerA->getParent(), defaultHandler);

  solver.addContactSurfaceHandler(handlerA);
  EXPECT_EQ(handlerA->getParent(), defaultHandler);

  auto handlerB = std::make_shared<constraint::ContactSurfaceHandler>();
  solver.addContactSurfaceHandler(handlerB);
  EXPECT_EQ(handlerB->getParent(), handlerA);

  EXPECT_TRUE(solver.removeContactSurfaceHandler(handlerB));
  EXPECT_EQ(solver.getLastContactSurfaceHandler(), handlerA);

  EXPECT_TRUE(solver.removeContactSurfaceHandler(handlerA));
  EXPECT_EQ(solver.getLastContactSurfaceHandler(), defaultHandler);

  auto missing = std::make_shared<constraint::ContactSurfaceHandler>();
  EXPECT_FALSE(solver.removeContactSurfaceHandler(missing));
  EXPECT_EQ(solver.getLastContactSurfaceHandler(), defaultHandler);
}

//==============================================================================
TEST(ConstraintSolver, AddRemoveConstraint)
{
  ExposedConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("constraint");
  auto constraint = std::make_shared<DummyConstraint>(skeleton);

  EXPECT_EQ(solver.getNumConstraints(), 0u);
  solver.addConstraint(constraint);
  EXPECT_EQ(solver.getNumConstraints(), 1u);
  EXPECT_TRUE(solver.containsConstraint(constraint));

  solver.addConstraint(constraint);
  EXPECT_EQ(solver.getNumConstraints(), 1u);

  solver.removeConstraint(constraint);
  EXPECT_EQ(solver.getNumConstraints(), 0u);
  EXPECT_FALSE(solver.containsConstraint(constraint));

  EXPECT_NO_THROW(solver.removeConstraint(constraint));
}

//==============================================================================
TEST(ConstraintSolver, AutoCreatesCouplerConstraintsForMimicJoints)
{
  CouplerTrackingSolver solver;
  auto skeleton = dynamics::Skeleton::create("auto_coupler");

  dynamics::BodyNode::Properties bodyProps;
  bodyProps.mInertia.setMass(1.0);

  dynamics::RevoluteJoint::Properties refJointProps;
  refJointProps.mName = "ref_joint";
  refJointProps.mAxis = Eigen::Vector3d::UnitZ();
  auto refPair = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
      nullptr, refJointProps, bodyProps);

  dynamics::RevoluteJoint::Properties depJointProps;
  depJointProps.mName = "dep_joint";
  depJointProps.mAxis = Eigen::Vector3d::UnitZ();
  auto depPair = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
      refPair.second, depJointProps, bodyProps);

  auto* refJoint = refPair.first;
  auto* depJoint = depPair.first;
  ASSERT_NE(refJoint, nullptr);
  ASSERT_NE(depJoint, nullptr);

  depJoint->setActuatorType(dynamics::Joint::MIMIC);

  dynamics::MimicDofProperties mimicProp;
  mimicProp.mReferenceJoint = refJoint;
  mimicProp.mReferenceDofIndex = 0;
  mimicProp.mMultiplier = 1.0;
  mimicProp.mOffset = 0.0;
  mimicProp.mConstraintType = dynamics::MimicConstraintType::Coupler;
  depJoint->setMimicJointDof(0, mimicProp);

  solver.addSkeleton(skeleton);
  solver.updateConstraints();

  EXPECT_EQ(solver.getNumCouplerConstraints(), 1u);
  EXPECT_EQ(solver.getNumMimicMotorConstraints(), 0u);
}

//==============================================================================
TEST(ConstraintSolver, SetFromOtherConstraintSolver)
{
  constraint::ConstraintSolver source;
  source.setSplitImpulseEnabled(true);

  auto skeleton1 = dynamics::Skeleton::create("skel1");
  auto skeleton2 = dynamics::Skeleton::create("skel2");
  source.addSkeleton(skeleton1);
  source.addSkeleton(skeleton2);

  auto constraint = std::make_shared<DummyConstraint>(skeleton1);
  source.addConstraint(constraint);

  constraint::ConstraintSolver target;
  target.setFromOtherConstraintSolver(source);

  EXPECT_TRUE(target.isSplitImpulseEnabled());
  EXPECT_EQ(target.getSkeletons().size(), 2u);
  EXPECT_EQ(target.getNumConstraints(), 1u);
}

//==============================================================================
TEST(ConstraintSolver, AddSkeletonsBatch)
{
  constraint::ConstraintSolver solver;

  auto skeleton1 = dynamics::Skeleton::create("batch1");
  auto skeleton2 = dynamics::Skeleton::create("batch2");
  auto skeleton3 = dynamics::Skeleton::create("batch3");

  std::vector<dynamics::SkeletonPtr> skeletons
      = {skeleton1, skeleton2, skeleton3};
  solver.addSkeletons(skeletons);

  EXPECT_EQ(solver.getSkeletons().size(), 3u);
}

//==============================================================================
TEST(ConstraintSolver, RemoveSkeletonsBatch)
{
  constraint::ConstraintSolver solver;

  auto skeleton1 = dynamics::Skeleton::create("remove1");
  auto skeleton2 = dynamics::Skeleton::create("remove2");
  auto skeleton3 = dynamics::Skeleton::create("remove3");

  std::vector<dynamics::SkeletonPtr> skeletons
      = {skeleton1, skeleton2, skeleton3};
  solver.addSkeletons(skeletons);
  ASSERT_EQ(solver.getSkeletons().size(), 3u);

  std::vector<dynamics::SkeletonPtr> toRemove = {skeleton1, skeleton3};
  solver.removeSkeletons(toRemove);

  EXPECT_EQ(solver.getSkeletons().size(), 1u);
  EXPECT_EQ(solver.getSkeletons()[0], skeleton2);
}

//==============================================================================
TEST(ConstraintSolver, RemoveAllSkeletons)
{
  constraint::ConstraintSolver solver;

  auto skeleton1 = dynamics::Skeleton::create("all1");
  auto skeleton2 = dynamics::Skeleton::create("all2");
  solver.addSkeleton(skeleton1);
  solver.addSkeleton(skeleton2);
  ASSERT_EQ(solver.getSkeletons().size(), 2u);

  solver.removeAllSkeletons();

  EXPECT_EQ(solver.getSkeletons().size(), 0u);
}

//==============================================================================
TEST(ConstraintSolver, SkeletonCountTracking)
{
  constraint::ConstraintSolver solver;

  EXPECT_EQ(solver.getSkeletons().size(), 0u);

  auto skeleton1 = dynamics::Skeleton::create("num1");
  solver.addSkeleton(skeleton1);
  EXPECT_EQ(solver.getSkeletons().size(), 1u);

  auto skeleton2 = dynamics::Skeleton::create("num2");
  solver.addSkeleton(skeleton2);
  EXPECT_EQ(solver.getSkeletons().size(), 2u);

  solver.removeSkeleton(skeleton1);
  EXPECT_EQ(solver.getSkeletons().size(), 1u);
}

//==============================================================================
TEST(ConstraintSolver, GetLastCollisionResult)
{
  constraint::ConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("collision_test");
  solver.addSkeleton(skeleton);

  // Before solve, collision result should be empty
  const auto& result = solver.getLastCollisionResult();
  EXPECT_EQ(result.getNumContacts(), 0u);

  // Solve with no collisions
  solver.solve();

  // After solve with no colliding bodies, still no contacts
  EXPECT_EQ(solver.getLastCollisionResult().getNumContacts(), 0u);
}

//==============================================================================
TEST(ConstraintSolver, ClearLastCollisionResult)
{
  constraint::ConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("clear_test");
  solver.addSkeleton(skeleton);

  // Solve to populate collision result
  solver.solve();

  // Clear the collision result
  solver.clearLastCollisionResult();

  // After clearing, should be empty
  EXPECT_EQ(solver.getLastCollisionResult().getNumContacts(), 0u);
}

//==============================================================================
TEST(ConstraintSolver, SetCollisionDetectorWithValidDetector)
{
  constraint::ConstraintSolver solver;

  auto skeleton = dynamics::Skeleton::create("detector_test");
  solver.addSkeleton(skeleton);

  auto originalDetector = solver.getCollisionDetector();
  ASSERT_NE(originalDetector, nullptr);

  // Create a different collision detector (DART collision detector)
  auto newDetector = collision::DARTCollisionDetector::create();
  ASSERT_NE(newDetector, nullptr);
  ASSERT_NE(newDetector, originalDetector);

  // Set the new detector
  solver.setCollisionDetector(newDetector);

  // Verify the detector was changed
  EXPECT_EQ(solver.getCollisionDetector(), newDetector);

  // Verify collision group was recreated
  auto collisionGroup = solver.getCollisionGroup();
  ASSERT_NE(collisionGroup, nullptr);
}

//==============================================================================
TEST(ConstraintSolver, SetSameCollisionDetectorIgnored)
{
  constraint::ConstraintSolver solver;

  auto detector = solver.getCollisionDetector();
  auto group = solver.getCollisionGroup();

  // Setting the same detector should be a no-op
  solver.setCollisionDetector(detector);

  EXPECT_EQ(solver.getCollisionDetector(), detector);
  // Group should remain the same since detector didn't change
  EXPECT_EQ(solver.getCollisionGroup(), group);
}

//==============================================================================
TEST(ConstraintSolver, RemoveAllConstraints)
{
  ExposedConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("remove_all");

  auto constraint1 = std::make_shared<DummyConstraint>(skeleton);
  auto constraint2 = std::make_shared<DummyConstraint>(skeleton);
  auto constraint3 = std::make_shared<DummyConstraint>(skeleton);

  solver.addConstraint(constraint1);
  solver.addConstraint(constraint2);
  solver.addConstraint(constraint3);
  ASSERT_EQ(solver.getNumConstraints(), 3u);

  solver.removeAllConstraints();

  EXPECT_EQ(solver.getNumConstraints(), 0u);
  EXPECT_FALSE(solver.containsConstraint(constraint1));
  EXPECT_FALSE(solver.containsConstraint(constraint2));
  EXPECT_FALSE(solver.containsConstraint(constraint3));
}

//==============================================================================
TEST(ConstraintSolver, HasSkeleton)
{
  constraint::ConstraintSolver solver;

  auto skeleton1 = dynamics::Skeleton::create("has_skel1");
  auto skeleton2 = dynamics::Skeleton::create("has_skel2");

  EXPECT_FALSE(std::ranges::any_of(
      solver.getSkeletons(),
      [&](const auto& skel) { return skel == skeleton1; }));

  solver.addSkeleton(skeleton1);

  EXPECT_TRUE(std::ranges::any_of(solver.getSkeletons(), [&](const auto& skel) {
    return skel == skeleton1;
  }));
  EXPECT_FALSE(std::ranges::any_of(
      solver.getSkeletons(),
      [&](const auto& skel) { return skel == skeleton2; }));

  solver.addSkeleton(skeleton2);

  EXPECT_TRUE(std::ranges::any_of(solver.getSkeletons(), [&](const auto& skel) {
    return skel == skeleton1;
  }));
  EXPECT_TRUE(std::ranges::any_of(solver.getSkeletons(), [&](const auto& skel) {
    return skel == skeleton2;
  }));
}

//==============================================================================
TEST(ConstraintSolver, GetCollisionOption)
{
  constraint::ConstraintSolver solver;

  // Test non-const version
  collision::CollisionOption& option = solver.getCollisionOption();
  EXPECT_TRUE(option.enableContact);

  // Modify the option
  option.maxNumContacts = 500u;
  EXPECT_EQ(solver.getCollisionOption().maxNumContacts, 500u);

  // Test const version
  const constraint::ConstraintSolver& constSolver = solver;
  const collision::CollisionOption& constOption
      = constSolver.getCollisionOption();
  EXPECT_EQ(constOption.maxNumContacts, 500u);
}

//==============================================================================
TEST(ConstraintSolver, GetCollisionGroup)
{
  constraint::ConstraintSolver solver;

  // Test non-const version
  auto group = solver.getCollisionGroup();
  ASSERT_NE(group, nullptr);

  // Test const version
  const constraint::ConstraintSolver& constSolver = solver;
  auto constGroup = constSolver.getCollisionGroup();
  ASSERT_NE(constGroup, nullptr);
  EXPECT_EQ(group, constGroup);
}

//==============================================================================
TEST(ConstraintSolver, GetCollisionDetectorConst)
{
  constraint::ConstraintSolver solver;

  // Test non-const version
  auto detector = solver.getCollisionDetector();
  ASSERT_NE(detector, nullptr);

  // Test const version
  const constraint::ConstraintSolver& constSolver = solver;
  auto constDetector = constSolver.getCollisionDetector();
  ASSERT_NE(constDetector, nullptr);
  EXPECT_EQ(detector, constDetector);
}

//==============================================================================
TEST(ConstraintSolver, GetConstraintConst)
{
  constraint::ConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("const_constraint");
  auto constraint = std::make_shared<DummyConstraint>(skeleton);

  solver.addConstraint(constraint);

  // Test non-const version
  auto retrieved = solver.getConstraint(0);
  EXPECT_EQ(retrieved, constraint);

  // Test const version
  const constraint::ConstraintSolver& constSolver = solver;
  auto constRetrieved = constSolver.getConstraint(0);
  EXPECT_EQ(constRetrieved, constraint);
}

//==============================================================================
TEST(ConstraintSolver, ConstructorWithPrimaryLcpSolver)
{
  auto primarySolver = std::make_shared<ConstantLcpSolver>(0.5);
  constraint::ConstraintSolver solver(primarySolver);

  EXPECT_EQ(solver.getLcpSolver(), primarySolver);
}

//==============================================================================
TEST(ConstraintSolver, ConstructorWithPrimaryAndSecondaryLcpSolver)
{
  auto primarySolver = std::make_shared<ConstantLcpSolver>(0.5);
  auto secondarySolver = std::make_shared<ConstantLcpSolver>(0.3);
  constraint::ConstraintSolver solver(primarySolver, secondarySolver);

  EXPECT_EQ(solver.getLcpSolver(), primarySolver);
  EXPECT_EQ(solver.getSecondaryLcpSolver(), secondarySolver);
}

//==============================================================================
TEST(ConstraintSolver, ConstructorWithNullPrimaryLcpSolver)
{
  // Passing nullptr should use default solver
  constraint::ConstraintSolver solver(nullptr);

  // Should have a default solver
  EXPECT_NE(solver.getLcpSolver(), nullptr);
}

//==============================================================================
TEST(ConstraintSolver, SetNullLcpSolverIgnored)
{
  constraint::ConstraintSolver solver;
  auto originalSolver = solver.getLcpSolver();
  ASSERT_NE(originalSolver, nullptr);

  solver.setLcpSolver(nullptr);

  // Should still have the original solver
  EXPECT_EQ(solver.getLcpSolver(), originalSolver);
}

//==============================================================================
TEST(ConstraintSolver, SetSecondaryLcpSolverSameAsPrimary)
{
  constraint::ConstraintSolver solver;
  auto solverImpl = std::make_shared<ConstantLcpSolver>(0.2);

  solver.setLcpSolver(solverImpl);
  solver.setSecondaryLcpSolver(solverImpl);

  EXPECT_EQ(solver.getLcpSolver(), solverImpl);
  EXPECT_EQ(solver.getSecondaryLcpSolver(), solverImpl);
}

//==============================================================================
TEST(ConstraintSolver, SetPrimaryLcpSolverSameAsSecondary)
{
  constraint::ConstraintSolver solver;
  auto solverImpl = std::make_shared<ConstantLcpSolver>(0.4);

  solver.setSecondaryLcpSolver(solverImpl);
  solver.setLcpSolver(solverImpl);

  EXPECT_EQ(solver.getLcpSolver(), solverImpl);
  EXPECT_EQ(solver.getSecondaryLcpSolver(), solverImpl);
}

//==============================================================================
TEST(ConstraintSolver, GetLastCollisionResultNonConst)
{
  constraint::ConstraintSolver solver;

  // Test non-const version returns modifiable reference
  collision::CollisionResult& result = solver.getLastCollisionResult();
  EXPECT_EQ(result.getNumContacts(), 0u);

  // Clear it
  result.clear();
  EXPECT_EQ(solver.getLastCollisionResult().getNumContacts(), 0u);
}

//==============================================================================
TEST(ConstraintSolver, ProtectedHelpersAndConstCollisionResult)
{
  ExposedConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("helper_skel");

  EXPECT_TRUE(solver.checkAndAddSkeletonPublic(skeleton));
  EXPECT_FALSE(solver.checkAndAddSkeletonPublic(skeleton));
  EXPECT_TRUE(solver.hasSkeletonPublic(skeleton));

  auto constraint = std::make_shared<DummyConstraint>(skeleton);
  EXPECT_TRUE(solver.checkAndAddConstraintPublic(constraint));
  EXPECT_FALSE(solver.checkAndAddConstraintPublic(constraint));

  const constraint::ConstraintSolver& constSolver = solver;
  const auto& result = constSolver.getLastCollisionResult();
  EXPECT_EQ(result.getNumContacts(), 0u);
}

//==============================================================================
TEST(ConstraintSolver, SolverConfigurationAndDebugUtilities)
{
  auto world = simulation::World::create();
  ASSERT_NE(world, nullptr);
  world->setTimeStep(0.002);

  auto ground = createGround(
      Eigen::Vector3d(2.0, 0.1, 2.0), Eigen::Vector3d(0.0, -0.05, 0.0));
  ground->setMobile(false);
  auto box = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.0, 0.05, 0.0));

  world->addSkeleton(ground);
  world->addSkeleton(box);

  auto solver = world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);
  solver->setTimeStep(0.001);
  EXPECT_DOUBLE_EQ(solver->getTimeStep(), 0.001);

  solver->setLcpSolver(std::make_shared<math::DantzigSolver>());
  solver->setSecondaryLcpSolver(std::make_shared<math::PgsSolver>());
  EXPECT_NE(solver->getLcpSolver(), nullptr);
  EXPECT_NE(solver->getSecondaryLcpSolver(), nullptr);

  for (int i = 0; i < 5; ++i) {
    world->step();
  }
  EXPECT_GT(solver->getLastCollisionResult().getNumContacts(), 0u);

  solver->clearLastCollisionResult();
  EXPECT_EQ(solver->getLastCollisionResult().getNumContacts(), 0u);

  constraint::ConstraintSolver scratch;
  auto skelA = dynamics::Skeleton::create("scratch_a");
  auto skelB = dynamics::Skeleton::create("scratch_b");
  scratch.addSkeleton(skelA);
  scratch.addSkeleton(skelB);
  scratch.removeSkeleton(skelA);
  EXPECT_EQ(scratch.getSkeletons().size(), 1u);
  scratch.removeAllSkeletons();
  EXPECT_EQ(scratch.getSkeletons().size(), 0u);

  CoverageConstraintSolver debugSolver;
  constexpr std::size_t n = 2;
  const std::size_t nSkip = (n + 3) & ~static_cast<std::size_t>(3);
  std::vector<double> A(nSkip * n, 0.0);
  A[0] = 1.0;
  A[nSkip + 1] = 1.0;
  A[1] = 0.2;
  A[nSkip] = 0.2;
  EXPECT_TRUE(debugSolver.isSymmetric(n, A.data()));
  EXPECT_TRUE(debugSolver.isSymmetric(n, A.data(), 0, 1));

  std::vector<double> x(n, 0.0);
  std::vector<double> b(n, 0.0);
  std::vector<double> w(n, 0.0);
  std::vector<int> findex(n, -1);
  debugSolver.print(
      n,
      A.data(),
      x.data(),
      nullptr,
      nullptr,
      b.data(),
      w.data(),
      findex.data());
}

namespace {

struct WorldContactFixture
{
  simulation::WorldPtr world;
  dynamics::SkeletonPtr ground;
  dynamics::SkeletonPtr boxA;
  dynamics::SkeletonPtr boxB;
};

void setFrictionCoefficients(
    const dynamics::SkeletonPtr& skeleton,
    double primaryCoeff,
    double secondaryCoeff)
{
  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    auto* bodyNode = skeleton->getBodyNode(i);
    for (std::size_t j = 0; j < bodyNode->getNumShapeNodes(); ++j) {
      auto* shapeNode = bodyNode->getShapeNode(j);
      auto* dynamicsAspect = shapeNode->getDynamicsAspect();
      if (!dynamicsAspect) {
        continue;
      }
      dynamicsAspect->setPrimaryFrictionCoeff(primaryCoeff);
      dynamicsAspect->setSecondaryFrictionCoeff(secondaryCoeff);
      dynamicsAspect->setFrictionCoeff(0.5 * (primaryCoeff + secondaryCoeff));
    }
  }
}

WorldContactFixture makeGroundBoxWorld(bool addSecondBox)
{
  WorldContactFixture fixture;
  fixture.world = simulation::World::create();
  fixture.world->setTimeStep(0.001);

  fixture.ground = createGround(
      Eigen::Vector3d(2.0, 0.1, 2.0), Eigen::Vector3d(0.0, -0.05, 0.0));
  fixture.ground->setMobile(false);

  fixture.boxA = createBox(
      Eigen::Vector3d(0.3, 0.3, 0.3), Eigen::Vector3d(0.0, 0.05, 0.0));

  fixture.world->addSkeleton(fixture.ground);
  fixture.world->addSkeleton(fixture.boxA);

  if (addSecondBox) {
    fixture.boxB = createBox(
        Eigen::Vector3d(0.25, 0.25, 0.25), Eigen::Vector3d(0.1, 0.05, 0.0));
    fixture.world->addSkeleton(fixture.boxB);
  }

  return fixture;
}

} // namespace

TEST(ConstraintSolver, WorldStepGeneratesContacts)
{
  auto fixture = makeGroundBoxWorld(false);
  fixture.world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  for (int i = 0; i < 120; ++i) {
    fixture.world->step();
  }

  auto solver = fixture.world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);
  EXPECT_GT(solver->getLastCollisionResult().getNumContacts(), 0u);
}

TEST(ConstraintSolver, WorldStepFrictionlessContact)
{
  auto fixture = makeGroundBoxWorld(false);
  fixture.world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  setFrictionCoefficients(fixture.ground, 0.0, 0.0);
  setFrictionCoefficients(fixture.boxA, 0.0, 0.0);

  for (int i = 0; i < 120; ++i) {
    fixture.world->step();
  }

  auto solver = fixture.world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);
  EXPECT_GT(solver->getLastCollisionResult().getNumContacts(), 0u);
}

TEST(ConstraintSolver, WorldStepFrictionalContact)
{
  auto fixture = makeGroundBoxWorld(false);
  fixture.world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  setFrictionCoefficients(fixture.ground, 0.6, 0.4);
  setFrictionCoefficients(fixture.boxA, 0.8, 0.5);

  for (int i = 0; i < 120; ++i) {
    fixture.world->step();
  }

  auto solver = fixture.world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);
  EXPECT_GT(solver->getLastCollisionResult().getNumContacts(), 0u);
}

TEST(ConstraintSolver, WorldStepWithSplitImpulse)
{
  auto fixture = makeGroundBoxWorld(false);
  fixture.world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  auto solver = fixture.world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);
  solver->setSplitImpulseEnabled(true);

  for (int i = 0; i < 120; ++i) {
    fixture.world->step();
  }

  EXPECT_TRUE(solver->isSplitImpulseEnabled());
  EXPECT_GT(solver->getLastCollisionResult().getNumContacts(), 0u);
}

TEST(ConstraintSolver, WorldStepMultipleContactPairs)
{
  auto fixture = makeGroundBoxWorld(true);
  fixture.world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  for (int i = 0; i < 120; ++i) {
    fixture.world->step();
  }

  auto solver = fixture.world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);
  EXPECT_GT(solver->getLastCollisionResult().getNumContacts(), 0u);
}

TEST(ConstraintSolver, WorldStepAfterTimeStepAdjustment)
{
  auto fixture = makeGroundBoxWorld(false);
  fixture.world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  auto solver = fixture.world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);
  solver->setTimeStep(0.0005);
  EXPECT_DOUBLE_EQ(solver->getTimeStep(), 0.0005);

  for (int i = 0; i < 120; ++i) {
    fixture.world->step();
  }

  EXPECT_GT(solver->getLastCollisionResult().getNumContacts(), 0u);
}

//=============================================================================
TEST(ConstraintSolver, WorldStepFrictionalSplitImpulseMultipleContacts)
{
  auto fixture = makeGroundBoxWorld(false);
  fixture.world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  setFrictionCoefficients(fixture.ground, 0.9, 0.7);
  setFrictionCoefficients(fixture.boxA, 0.8, 0.6);

  auto solver = fixture.world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);
  solver->setSplitImpulseEnabled(true);
  solver->getCollisionOption().maxNumContacts = 8u;

  for (int i = 0; i < 120; ++i) {
    fixture.world->step();
  }

  EXPECT_TRUE(solver->isSplitImpulseEnabled());
  EXPECT_GT(solver->getLastCollisionResult().getNumContacts(), 0u);
}

//=============================================================================
TEST(ConstraintSolver, WorldStepFrictionlessSplitImpulse)
{
  auto fixture = makeGroundBoxWorld(false);
  fixture.world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  setFrictionCoefficients(fixture.ground, 0.0, 0.0);
  setFrictionCoefficients(fixture.boxA, 0.0, 0.0);

  auto solver = fixture.world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);
  solver->setSplitImpulseEnabled(true);
  solver->getCollisionOption().maxNumContacts = 8u;

  for (int i = 0; i < 120; ++i) {
    fixture.world->step();
  }

  EXPECT_TRUE(solver->isSplitImpulseEnabled());
  EXPECT_GT(solver->getLastCollisionResult().getNumContacts(), 0u);
}

//=============================================================================
TEST(ConstraintSolver, WorldStepExpandedContactManifold)
{
  auto world = simulation::World::create();
  world->setTimeStep(0.001);
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  auto ground = createGround(
      Eigen::Vector3d(2.0, 0.1, 2.0), Eigen::Vector3d(0.0, -0.05, 0.0));
  ground->setMobile(false);
  auto box = createBox(
      Eigen::Vector3d(0.6, 0.2, 0.6), Eigen::Vector3d(0.0, 0.05, 0.0));

  world->addSkeleton(ground);
  world->addSkeleton(box);

  auto solver = world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);
  solver->getCollisionOption().maxNumContacts = 8u;

  for (int i = 0; i < 120; ++i) {
    world->step();
  }

  EXPECT_GE(solver->getLastCollisionResult().getNumContacts(), 2u);
}

TEST(ConstraintSolver, WorldStepDynamicDynamicContact)
{
  auto world = simulation::World::create();
  world->setTimeStep(0.001);
  world->setGravity(Eigen::Vector3d::Zero());

  auto boxA
      = createBox(Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d::Zero());
  auto boxB = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.15, 0.0, 0.0));

  setFrictionCoefficients(boxA, 0.7, 0.5);
  setFrictionCoefficients(boxB, 0.6, 0.4);

  world->addSkeleton(boxA);
  world->addSkeleton(boxB);

  world->step();

  auto solver = world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);
  EXPECT_GT(solver->getLastCollisionResult().getNumContacts(), 0u);
}

TEST(ContactConstraint, FrictionImpulseVelocityChangeWithSlip)
{
  class VelocityContactConstraint final : public constraint::ContactConstraint
  {
  public:
    VelocityContactConstraint(
        collision::Contact& contact,
        double timeStep,
        const constraint::ContactSurfaceParams& params)
      : constraint::ContactConstraint(contact, timeStep, params)
    {
    }

    using constraint::ContactConstraint::applyImpulse;
    using constraint::ContactConstraint::applyPositionImpulse;
    using constraint::ContactConstraint::applyUnitImpulse;
    using constraint::ContactConstraint::getVelocityChange;
    using constraint::ContactConstraint::isActive;
    using constraint::ContactConstraint::update;
  };

  auto detector = collision::DARTCollisionDetector::create();
  ASSERT_NE(detector, nullptr);

  auto boxA
      = createBox(Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d::Zero());
  auto boxB = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.0, 0.0, 0.18));

  auto* shapeNodeA = boxA->getBodyNode(0)->getShapeNode(0);
  auto* shapeNodeB = boxB->getBodyNode(0)->getShapeNode(0);
  ASSERT_NE(shapeNodeA, nullptr);
  ASSERT_NE(shapeNodeB, nullptr);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(shapeNodeA);
  group->addShapeFrame(shapeNodeB);

  collision::CollisionOption option;
  option.maxNumContacts = 1u;
  collision::CollisionResult result;
  const bool collided = group->collide(option, &result);
  ASSERT_TRUE(collided);
  ASSERT_GT(result.getNumContacts(), 0u);

  collision::Contact contact = result.getContact(0);

  constraint::ContactSurfaceParams params;
  params.mPrimaryFrictionCoeff = 0.6;
  params.mSecondaryFrictionCoeff = 0.4;
  params.mFirstFrictionalDirection = Eigen::Vector3d::UnitX();
  params.mContactSurfaceMotionVelocity = Eigen::Vector3d(0.1, -0.2, 0.0);

  VelocityContactConstraint constraint(contact, 0.001, params);
  constraint.update();
  EXPECT_TRUE(constraint.isActive());

  constraint.applyUnitImpulse(1);
  std::vector<double> vel(constraint.getDimension(), 0.0);
  constraint.getVelocityChange(vel.data(), true);
  EXPECT_TRUE(std::isfinite(vel[1]));

  double lambda[3] = {0.4, 0.1, -0.05};
  constraint.applyImpulse(lambda);
  constraint.applyPositionImpulse(lambda);
  EXPECT_TRUE(boxA->isPositionImpulseApplied());
  EXPECT_TRUE(boxB->isPositionImpulseApplied());
}

TEST(ContactConstraint, StaticSettersAndGetters)
{
  const double origAllowance
      = constraint::ContactConstraint::getErrorAllowance();
  const double origErp
      = constraint::ContactConstraint::getErrorReductionParameter();
  const double origErv
      = constraint::ContactConstraint::getMaxErrorReductionVelocity();
  const double origCfm
      = constraint::ContactConstraint::getConstraintForceMixing();

  constraint::ContactConstraint::setErrorAllowance(0.005);
  EXPECT_DOUBLE_EQ(constraint::ContactConstraint::getErrorAllowance(), 0.005);

  constraint::ContactConstraint::setErrorAllowance(-1.0);
  EXPECT_GE(constraint::ContactConstraint::getErrorAllowance(), 0.0);

  constraint::ContactConstraint::setErrorReductionParameter(0.5);
  EXPECT_DOUBLE_EQ(
      constraint::ContactConstraint::getErrorReductionParameter(), 0.5);

  constraint::ContactConstraint::setErrorReductionParameter(-0.1);
  EXPECT_GE(constraint::ContactConstraint::getErrorReductionParameter(), 0.0);

  constraint::ContactConstraint::setErrorReductionParameter(1.5);
  EXPECT_LE(constraint::ContactConstraint::getErrorReductionParameter(), 1.5);

  constraint::ContactConstraint::setMaxErrorReductionVelocity(10.0);
  EXPECT_DOUBLE_EQ(
      constraint::ContactConstraint::getMaxErrorReductionVelocity(), 10.0);

  constraint::ContactConstraint::setMaxErrorReductionVelocity(-5.0);
  EXPECT_GE(constraint::ContactConstraint::getMaxErrorReductionVelocity(), 0.0);

  constraint::ContactConstraint::setConstraintForceMixing(1e-3);
  EXPECT_DOUBLE_EQ(
      constraint::ContactConstraint::getConstraintForceMixing(), 1e-3);

  constraint::ContactConstraint::setConstraintForceMixing(1e-12);
  EXPECT_GE(constraint::ContactConstraint::getConstraintForceMixing(), 1e-9);

  EXPECT_EQ(
      constraint::ContactConstraint::getStaticType(), "ContactConstraint");

  constraint::ContactConstraint::setErrorAllowance(origAllowance);
  constraint::ContactConstraint::setErrorReductionParameter(origErp);
  constraint::ContactConstraint::setMaxErrorReductionVelocity(origErv);
  constraint::ContactConstraint::setConstraintForceMixing(origCfm);
}

TEST(ContactConstraint, FrictionDirectionSetGet)
{
  auto detector = collision::DARTCollisionDetector::create();
  ASSERT_NE(detector, nullptr);

  auto boxA
      = createBox(Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d::Zero());
  auto boxB = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.0, 0.0, 0.18));

  auto* shapeNodeA = boxA->getBodyNode(0)->getShapeNode(0);
  auto* shapeNodeB = boxB->getBodyNode(0)->getShapeNode(0);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(shapeNodeA);
  group->addShapeFrame(shapeNodeB);

  collision::CollisionOption option;
  option.maxNumContacts = 1u;
  collision::CollisionResult result;
  const bool collided = group->collide(option, &result);
  ASSERT_TRUE(collided);
  ASSERT_GT(result.getNumContacts(), 0u);

  collision::Contact contact = result.getContact(0);

  constraint::ContactSurfaceParams params;
  params.mPrimaryFrictionCoeff = 0.5;
  params.mSecondaryFrictionCoeff = 0.3;

  constraint::ContactConstraint cc(contact, 0.001, params);

  EXPECT_EQ(cc.getType(), constraint::ContactConstraint::getStaticType());

  Eigen::Vector3d dir(1.0, 2.0, 3.0);
  cc.setFrictionDirection(dir);
  const Eigen::Vector3d& retrieved = cc.getFrictionDirection1();
  EXPECT_NEAR(retrieved.norm(), 1.0, 1e-10);
  EXPECT_NEAR(retrieved.dot(dir.normalized()), 1.0, 1e-10);
}

TEST(ContactConstraint, ApplyImpulseAndPositionImpulse)
{
  class TestContactConstraint final : public constraint::ContactConstraint
  {
  public:
    TestContactConstraint(
        collision::Contact& contact,
        double timeStep,
        const constraint::ContactSurfaceParams& params)
      : constraint::ContactConstraint(contact, timeStep, params)
    {
    }

    using constraint::ContactConstraint::applyImpulse;
    using constraint::ContactConstraint::applyPositionImpulse;
    using constraint::ContactConstraint::isActive;
    using constraint::ContactConstraint::update;
  };

  auto detector = collision::DARTCollisionDetector::create();
  ASSERT_NE(detector, nullptr);

  auto boxA
      = createBox(Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d::Zero());
  auto boxB = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.0, 0.0, 0.18));

  auto* shapeNodeA = boxA->getBodyNode(0)->getShapeNode(0);
  auto* shapeNodeB = boxB->getBodyNode(0)->getShapeNode(0);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(shapeNodeA);
  group->addShapeFrame(shapeNodeB);

  collision::CollisionOption option;
  option.maxNumContacts = 1u;
  collision::CollisionResult result;
  const bool collided = group->collide(option, &result);
  ASSERT_TRUE(collided);
  ASSERT_GT(result.getNumContacts(), 0u);

  collision::Contact contact = result.getContact(0);

  constraint::ContactSurfaceParams params;
  params.mPrimaryFrictionCoeff = 0.8;
  params.mSecondaryFrictionCoeff = 0.4;
  params.mFirstFrictionalDirection = contact.normal;
  params.mContactSurfaceMotionVelocity = Eigen::Vector3d(0.0, 0.1, -0.1);

  TestContactConstraint constraint(contact, 0.001, params);
  constraint.update();
  EXPECT_TRUE(constraint.isActive());

  auto* jointA = dynamic_cast<dynamics::FreeJoint*>(boxA->getJoint(0));
  auto* jointB = dynamic_cast<dynamics::FreeJoint*>(boxB->getJoint(0));
  ASSERT_NE(jointA, nullptr);
  ASSERT_NE(jointB, nullptr);

  Eigen::Vector6d velA = Eigen::Vector6d::Zero();
  velA[5] = 0.2;
  jointA->setSpatialVelocity(velA, Frame::World(), Frame::World());

  Eigen::Vector6d velB = Eigen::Vector6d::Zero();
  velB[3] = -0.1;
  jointB->setSpatialVelocity(velB, Frame::World(), Frame::World());

  double lambda[3] = {0.6, 0.15, -0.05};
  constraint.applyImpulse(lambda);
  EXPECT_GT(contact.force.norm(), 0.0);

  constraint.applyPositionImpulse(lambda);
  EXPECT_TRUE(boxA->isPositionImpulseApplied());
  EXPECT_TRUE(boxB->isPositionImpulseApplied());
}

//=============================================================================
TEST(ConstraintSolver, EachConstraintConstAndNonConst)
{
  constraint::ConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("each_constraint");
  auto constraintA = std::make_shared<DummyConstraint>(skeleton);
  auto constraintB = std::make_shared<DummyConstraint>(skeleton);

  solver.addConstraint(constraintA);
  solver.addConstraint(constraintB);

  std::vector<constraint::ConstraintBase*> seen;
  solver.eachConstraint([&](auto constraint) {
    using ConstraintType = std::decay_t<decltype(constraint)>;
    if constexpr (std::is_pointer_v<ConstraintType>) {
      seen.push_back(constraint);
    } else {
      seen.push_back(constraint.get());
    }
  });
  EXPECT_EQ(seen.size(), 2u);

  const constraint::ConstraintSolver& constSolver = solver;
  std::size_t callbackCount = 0;
  constSolver.eachConstraint([&](auto) {
    ++callbackCount;
    return callbackCount < 1u;
  });
  EXPECT_EQ(callbackCount, 1u);
}
