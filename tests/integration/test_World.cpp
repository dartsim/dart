/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "TestHelpers.hpp"
#include "dart/collision/CollisionDetector.hpp"
#include "dart/collision/CollisionFilter.hpp"
#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/detail/CollisionFilterSnapshotTracker.hpp"
#include "dart/common/Factory.hpp"
#include "dart/common/Macros.hpp"
#include "dart/common/Uri.hpp"
#include "dart/common/detail/ModelMutation.hpp"
#include "dart/config.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/BoxedLcpSolver.hpp"
#include "dart/constraint/ConstrainedGroup.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/constraint/ContactConstraint.hpp"
#include "dart/constraint/ContactSurface.hpp"
#include "dart/constraint/CouplerConstraint.hpp"
#include "dart/constraint/DynamicJointConstraint.hpp"
#include "dart/constraint/FbfFrictionSolver.hpp"
#include "dart/constraint/JointConstraint.hpp"
#include "dart/constraint/JointCoulombFrictionConstraint.hpp"
#include "dart/constraint/JointLimitConstraint.hpp"
#include "dart/constraint/MimicMotorConstraint.hpp"
#include "dart/constraint/NsgsFrictionSolver.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "dart/constraint/ServoMotorConstraint.hpp"
#include "dart/constraint/SoftContactConstraint.hpp"
#include "dart/constraint/WeldJointConstraint.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/ConvexMeshShape.hpp"
#include "dart/dynamics/EndEffector.hpp"
#include "dart/dynamics/MeshShape.hpp"
#if HAVE_OCTOMAP
  #include "dart/dynamics/VoxelGridShape.hpp"
#endif
#include "dart/dynamics/FixedFrame.hpp"
#include "dart/dynamics/Group.hpp"
#include "dart/dynamics/LineSegmentShape.hpp"
#include "dart/dynamics/Linkage.hpp"
#include "dart/dynamics/PlanarJoint.hpp"
#include "dart/dynamics/PointCloudShape.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/TranslationalJoint2D.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/math/Random.hpp"
#include "dart/simulation/DeactivationOptions.hpp"
#include "dart/simulation/Recording.hpp"
#include "dart/utils/SkelParser.hpp"

#include <Eigen/Core>
#include <assimp/scene.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <cstddef>
#include <cstdlib>
#include <cstring>
#if HAVE_BULLET
  #include "dart/collision/bullet/bullet.hpp"
#endif
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/collision/fcl/FCLCollisionDetector.hpp"
#if HAVE_ODE
  #include "dart/collision/ode/OdeCollisionDetector.hpp"
#endif
#include "dart/constraint/BallJointConstraint.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/DantzigBoxedLcpSolver.hpp"
#include "dart/constraint/FbfFrictionSolver.hpp"
#include "dart/constraint/NsgsFrictionSolver.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "dart/constraint/RevoluteJointConstraint.hpp"
#include "dart/simulation/World.hpp"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;

// Private registry probes defined in World.cpp for tests only.
namespace dart::simulation::detail {
std::size_t getIndexedWorldCount();
std::size_t getRegistryLockCount();
// The nth index allocation from now throws std::bad_alloc; 0 disarms.
void failIndexAllocation(std::size_t nth);
void setRechainUnlockedHook(std::function<void()> hook);
} // namespace dart::simulation::detail

namespace {

// Its destructor follows function-local static objects first initialized by
// tests. Memory-checker runs use this to cover Worlds retained by pre-main
// globals.
WorldPtr gLateDestroyedWorld;

struct SkeletonVersionAccess : Skeleton
{
  using Skeleton::getGlobalStructuralVersion;
};

class NamedFixedFrame final : public FixedFrame
{
public:
  explicit NamedFixedFrame(Frame* parent)
    : Entity(ConstructFrame), Frame(parent), FixedFrame(parent)
  {
  }

  const std::string& setName(const std::string& name) override
  {
    mName = name;
    return mName;
  }

  const std::string& getName() const override
  {
    return mName;
  }

private:
  std::string mName;
};

// A valid native Frame need not also inherit Composite. Its owned descendants
// must still protect its child sets before constructors or reparenting run.
class BareFrame final : public Frame, public Detachable
{
public:
  explicit BareFrame(Frame* parent)
    : Entity(ConstructFrame), Frame(parent), Detachable(parent, false)
  {
  }

  const std::string& setName(const std::string& name) override
  {
    mName = name;
    return mName;
  }

  const std::string& getName() const override
  {
    return mName;
  }

  const Eigen::Isometry3d& getRelativeTransform() const override
  {
    return mTransform;
  }

  const Eigen::Vector6d& getRelativeSpatialVelocity() const override
  {
    return mZero;
  }

  const Eigen::Vector6d& getRelativeSpatialAcceleration() const override
  {
    return mZero;
  }

  const Eigen::Vector6d& getPrimaryRelativeAcceleration() const override
  {
    return mZero;
  }

  const Eigen::Vector6d& getPartialAcceleration() const override
  {
    return mZero;
  }

private:
  std::string mName;
  Eigen::Isometry3d mTransform = Eigen::Isometry3d::Identity();
  Eigen::Vector6d mZero = Eigen::Vector6d::Zero();
};

class ScopedCollisionFactoryDisabler
{
public:
  using Factory = collision::CollisionDetector::Factory;
  using Creator = Factory::Creator;

  ScopedCollisionFactoryDisabler(std::string key, Creator restorer)
    : mFactory(collision::CollisionDetector::getFactory()),
      mKey(std::move(key)),
      mRestorer(std::move(restorer))
  {
    if (!mFactory || !mFactory->canCreate(mKey))
      return;

    mDisabled = true;
    mFactory->unregisterCreator(mKey);
  }

  ScopedCollisionFactoryDisabler(const ScopedCollisionFactoryDisabler&)
      = delete;
  ScopedCollisionFactoryDisabler& operator=(
      const ScopedCollisionFactoryDisabler&)
      = delete;

  ~ScopedCollisionFactoryDisabler()
  {
    if (mFactory && mDisabled && mRestorer)
      mFactory->registerCreator(mKey, mRestorer);
  }

  bool wasDisabled() const
  {
    return mDisabled;
  }

private:
  Factory* mFactory;
  std::string mKey;
  Creator mRestorer;
  bool mDisabled{false};
};

} // namespace

namespace {

//==============================================================================
class CountingBoxedLcpConstraintSolver
  : public constraint::BoxedLcpConstraintSolver
{
public:
  explicit CountingBoxedLcpConstraintSolver(
      constraint::BoxedLcpSolverPtr solver)
    : constraint::BoxedLcpConstraintSolver(std::move(solver), nullptr)
  {
  }

  std::size_t getMaxConcurrentSolves() const
  {
    return mMaxConcurrentSolves.load();
  }

protected:
  void solveConstrainedGroup(constraint::ConstrainedGroup&) override
  {
    const std::size_t active = mActiveSolves.fetch_add(1u) + 1u;
    std::size_t observed = mMaxConcurrentSolves.load();
    while (active > observed
           && !mMaxConcurrentSolves.compare_exchange_weak(observed, active)) {
      // Retry until the larger observed value is recorded.
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    mActiveSolves.fetch_sub(1u);
  }

private:
  std::atomic<std::size_t> mActiveSolves{0u};
  std::atomic<std::size_t> mMaxConcurrentSolves{0u};
};

//==============================================================================
std::size_t countConcurrentIndependentSolves(bool randomizePgs)
{
  auto world = World::create();
  DeactivationOptions disabled;
  disabled.mEnabled = false;
  world->setDeactivationOptions(disabled);

  auto pgs = std::make_shared<constraint::PgsBoxedLcpSolver>();
  auto option = pgs->getOption();
  option.mRandomizeConstraintOrder = randomizePgs;
  pgs->setOption(option);

  auto solver
      = std::make_unique<CountingBoxedLcpConstraintSolver>(std::move(pgs));
  auto* solverPtr = solver.get();
  world->setConstraintSolver(std::move(solver));
  world->setNumSimulationThreads(4u);

  constexpr std::size_t numGroups = 130u;
  for (std::size_t i = 0; i < numGroups; ++i) {
    auto body = createObject(Eigen::Vector3d(static_cast<double>(i), 0.0, 0.0));
    auto* bodyNode = body->getBodyNode(0);
    world->addSkeleton(body);

    auto constraint = std::make_shared<constraint::BallJointConstraint>(
        bodyNode, bodyNode->getTransform().translation());
    world->getConstraintSolver()->addConstraint(constraint);
  }

  world->step();
  return solverPtr->getMaxConcurrentSolves();
}

//==============================================================================
WorldPtr createCloneContactWorld()
{
  auto world = World::create();
  world->setCollisionDetector(CollisionDetectorType::Dart);
  DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  auto ground = createBox(
      Eigen::Vector3d(5.0, 5.0, 0.1), Eigen::Vector3d(0.0, 0.0, -0.05));
  ground->setMobile(false);
  world->addSkeleton(ground);
  auto sphere = createSphere(0.5, Eigen::Vector3d(0.0, 0.0, 0.55));
  sphere->setVelocity(3, 0.2);
  world->addSkeleton(sphere);
  return world;
}

//==============================================================================
void expectCloneContactStepsIdentical(
    const WorldPtr& source, const WorldPtr& clone)
{
  bool hadContacts = false;
  for (int step = 0; step < 200; ++step) {
    SCOPED_TRACE(step);
    source->step();
    clone->step();
    ASSERT_EQ(source->getNumSkeletons(), clone->getNumSkeletons());
    for (std::size_t i = 0; i < source->getNumSkeletons(); ++i) {
      const auto original = source->getSkeleton(i);
      const auto copy = clone->getSkeleton(i);
      const auto expectBitsEqual = [](const Eigen::VectorXd& a,
                                      const Eigen::VectorXd& b) {
        ASSERT_EQ(a.size(), b.size());
        ASSERT_TRUE(a.allFinite());
        ASSERT_TRUE(b.allFinite());
        EXPECT_EQ(
            0, std::memcmp(a.data(), b.data(), a.size() * sizeof(double)));
      };
      expectBitsEqual(original->getPositions(), copy->getPositions());
      expectBitsEqual(original->getVelocities(), copy->getVelocities());
      expectBitsEqual(original->getAccelerations(), copy->getAccelerations());
      expectBitsEqual(
          original->getConstraintForces(), copy->getConstraintForces());
    }
    const auto& originalContacts
        = source->getConstraintSolver()->getLastCollisionResult();
    const auto& clonedContacts
        = clone->getConstraintSolver()->getLastCollisionResult();
    ASSERT_EQ(
        originalContacts.getNumContacts(), clonedContacts.getNumContacts());
    hadContacts = hadContacts || originalContacts.getNumContacts() > 0u;
  }
  EXPECT_TRUE(hadContacts);
}

//==============================================================================
void expectPgsOptionsEqual(
    const constraint::ConstBoxedLcpSolverPtr& backend,
    const constraint::PgsBoxedLcpSolver::Option& expected)
{
  const auto pgs
      = std::dynamic_pointer_cast<const constraint::PgsBoxedLcpSolver>(backend);
  ASSERT_NE(nullptr, pgs);
  const auto& actual = pgs->getOption();
  EXPECT_EQ(expected.mMaxIteration, actual.mMaxIteration);
  EXPECT_EQ(expected.mDeltaXThreshold, actual.mDeltaXThreshold);
  EXPECT_EQ(expected.mRelativeDeltaXTolerance, actual.mRelativeDeltaXTolerance);
  EXPECT_EQ(expected.mEpsilonForDivision, actual.mEpsilonForDivision);
  EXPECT_EQ(
      expected.mRandomizeConstraintOrder, actual.mRandomizeConstraintOrder);
}

// A derived built-in is still a custom backend and must not be sliced.
class CustomCloneBackend : public constraint::DantzigBoxedLcpSolver
{
public:
  const std::string& getType() const override
  {
    static const std::string type = "CustomCloneBackend";
    return type;
  }
};

class CustomCloneConstraintSolver : public constraint::ConstraintSolver
{
protected:
  void solveConstrainedGroup(constraint::ConstrainedGroup&) override {}
};

} // namespace

//==============================================================================
TEST(World, ClonePgsSolverOptions)
{
  auto source = createCloneContactWorld();
  auto* solver = static_cast<constraint::BoxedLcpConstraintSolver*>(
      source->getConstraintSolver());
  auto pgs = std::make_shared<constraint::PgsBoxedLcpSolver>();
  const constraint::PgsBoxedLcpSolver::Option options(
      75, 1e-8, 2e-5, 1e-10, false);
  pgs->setOption(options);
  solver->setBoxedLcpSolver(pgs);
  solver->setSecondaryBoxedLcpSolver(nullptr);

  auto clone = source->clone();
  const auto* copy = dynamic_cast<const constraint::BoxedLcpConstraintSolver*>(
      clone->getConstraintSolver());
  ASSERT_NE(nullptr, copy);
  expectPgsOptionsEqual(copy->getBoxedLcpSolver(), options);
  EXPECT_NE(pgs, copy->getBoxedLcpSolver());
  EXPECT_EQ(nullptr, copy->getSecondaryBoxedLcpSolver());
  expectCloneContactStepsIdentical(source, clone);

  pgs->setOption(constraint::PgsBoxedLcpSolver::Option{});
  expectPgsOptionsEqual(copy->getBoxedLcpSolver(), options);
}

//==============================================================================
TEST(World, CloneRandomizedPgsSolverOptions)
{
  auto source = World::create();
  auto* solver = static_cast<constraint::BoxedLcpConstraintSolver*>(
      source->getConstraintSolver());
  auto pgs = std::make_shared<constraint::PgsBoxedLcpSolver>();
  const constraint::PgsBoxedLcpSolver::Option options(
      65, 2e-8, 3e-5, 2e-10, true);
  pgs->setOption(options);
  solver->setBoxedLcpSolver(pgs);
  auto clone = source->clone();
  const auto* copy = static_cast<const constraint::BoxedLcpConstraintSolver*>(
      clone->getConstraintSolver());
  expectPgsOptionsEqual(copy->getBoxedLcpSolver(), options);
}

//==============================================================================
TEST(World, CloneNsgsPrimaryAndSecondaryOptions)
{
  for (auto law :
       {constraint::NsgsFrictionSolver::Law::Coulomb,
        constraint::NsgsFrictionSolver::Law::Associated,
        constraint::NsgsFrictionSolver::Law::Box}) {
    SCOPED_TRACE(static_cast<int>(law));
    auto source = createCloneContactWorld();
    auto* solver = static_cast<constraint::BoxedLcpConstraintSolver*>(
        source->getConstraintSolver());
    constraint::NsgsFrictionSolver::Options options;
    options.law = law;
    options.boxForAnisotropic = false;
    options.maxSweeps = 71;
    options.tolerance = 2e-7;
    auto primary = std::make_shared<constraint::NsgsFrictionSolver>(options);
    auto secondaryOptions = options;
    secondaryOptions.law = constraint::NsgsFrictionSolver::Law::Box;
    secondaryOptions.boxForAnisotropic = true;
    secondaryOptions.maxSweeps = 33;
    secondaryOptions.tolerance = 3e-6;
    auto secondary
        = std::make_shared<constraint::NsgsFrictionSolver>(secondaryOptions);
    solver->setBoxedLcpSolver(primary);
    solver->setSecondaryBoxedLcpSolver(secondary);
    auto clone = source->clone();
    const auto* copiedSolver
        = static_cast<const constraint::BoxedLcpConstraintSolver*>(
            clone->getConstraintSolver());
    const auto copied
        = std::dynamic_pointer_cast<const constraint::NsgsFrictionSolver>(
            copiedSolver->getBoxedLcpSolver());
    const auto copiedSecondary
        = std::dynamic_pointer_cast<const constraint::NsgsFrictionSolver>(
            copiedSolver->getSecondaryBoxedLcpSolver());
    ASSERT_NE(nullptr, copied);
    ASSERT_NE(nullptr, copiedSecondary);
    EXPECT_NE(primary, copied);
    EXPECT_NE(secondary, copiedSecondary);
    const auto expectOptions = [](const auto& actual, const auto& expected) {
      EXPECT_EQ(expected.law, actual.law);
      EXPECT_EQ(expected.boxForAnisotropic, actual.boxForAnisotropic);
      EXPECT_EQ(expected.maxSweeps, actual.maxSweeps);
      EXPECT_EQ(expected.tolerance, actual.tolerance);
    };
    expectOptions(copied->getOptions(), options);
    expectOptions(copiedSecondary->getOptions(), secondaryOptions);
    EXPECT_EQ(0u, copied->getStats().numSolves);
    expectCloneContactStepsIdentical(source, clone);
    EXPECT_GT(copied->getStats().numSolves, 0u);
    auto advancedClone = source->clone();
    const auto* advancedSolver
        = static_cast<const constraint::BoxedLcpConstraintSolver*>(
            advancedClone->getConstraintSolver());
    const auto advanced
        = std::dynamic_pointer_cast<const constraint::NsgsFrictionSolver>(
            advancedSolver->getBoxedLcpSolver());
    ASSERT_NE(nullptr, advanced);
    EXPECT_EQ(0u, advanced->getStats().numSolves);
    primary->setOptions(constraint::NsgsFrictionSolver::Options{});
    expectOptions(copied->getOptions(), options);
  }
}

//==============================================================================
TEST(World, CloneFbfPrimaryAndSecondaryOptions)
{
  using Fbf = constraint::FbfFrictionSolver;
  auto source = createCloneContactWorld();
  auto* solver = static_cast<constraint::BoxedLcpConstraintSolver*>(
      source->getConstraintSolver());
  const Fbf::Options options{false, 71, 2e-7, 0.7, 9, 0.05};
  const Fbf::Options secondaryOptions{true, 33, 3e-6, 0.4, 5, 0.2};
  auto primary = std::make_shared<Fbf>(options);
  auto secondary = std::make_shared<Fbf>(secondaryOptions);
  solver->setBoxedLcpSolver(primary);
  solver->setSecondaryBoxedLcpSolver(secondary);
  auto clone = source->clone();
  const auto* copiedSolver
      = static_cast<const constraint::BoxedLcpConstraintSolver*>(
          clone->getConstraintSolver());
  const auto copied
      = std::dynamic_pointer_cast<const Fbf>(copiedSolver->getBoxedLcpSolver());
  const auto copiedSecondary = std::dynamic_pointer_cast<const Fbf>(
      copiedSolver->getSecondaryBoxedLcpSolver());
  ASSERT_NE(nullptr, copied);
  ASSERT_NE(nullptr, copiedSecondary);
  EXPECT_NE(primary, copied);
  EXPECT_NE(secondary, copiedSecondary);
  const auto expectOptions = [](const auto& actual, const auto& expected) {
    EXPECT_EQ(expected.boxForAnisotropic, actual.boxForAnisotropic);
    EXPECT_EQ(expected.maxOuterIterations, actual.maxOuterIterations);
    EXPECT_EQ(expected.tolerance, actual.tolerance);
    EXPECT_EQ(expected.stepScale, actual.stepScale);
    EXPECT_EQ(expected.maxInnerSweeps, actual.maxInnerSweeps);
    EXPECT_EQ(expected.innerToleranceFactor, actual.innerToleranceFactor);
  };
  expectOptions(copied->getOptions(), options);
  expectOptions(copiedSecondary->getOptions(), secondaryOptions);
  const auto expectFreshStats = [](const auto& stats) {
    EXPECT_EQ(0u, stats.numSolves);
    EXPECT_EQ(0u, stats.numConverged);
    EXPECT_EQ(0u, stats.numAcceptedAtCap);
    EXPECT_EQ(0u, stats.numFailed);
    EXPECT_EQ(0u, stats.numContacts);
    EXPECT_EQ(0u, stats.numBoxContacts);
    EXPECT_EQ(0u, stats.numLocalFallbacks);
    EXPECT_EQ(0u, stats.numIterations);
    EXPECT_EQ(0u, stats.numInnerIterations);
    EXPECT_EQ(0u, stats.numStepShrinks);
    EXPECT_EQ(0u, stats.numInnerCaps);
    EXPECT_EQ(0.0, stats.maxViolation);
  };
  expectFreshStats(copied->getStats());
  expectFreshStats(copiedSecondary->getStats());
  expectCloneContactStepsIdentical(source, clone);
  EXPECT_GT(copied->getStats().numSolves, 0u);
  auto advancedClone = source->clone();
  const auto* advancedSolver
      = static_cast<const constraint::BoxedLcpConstraintSolver*>(
          advancedClone->getConstraintSolver());
  const auto advanced = std::dynamic_pointer_cast<const Fbf>(
      advancedSolver->getBoxedLcpSolver());
  ASSERT_NE(nullptr, advanced);
  expectFreshStats(advanced->getStats());
  primary->setOptions(Fbf::Options{});
  secondary->setOptions(Fbf::Options{});
  expectOptions(copied->getOptions(), options);
  expectOptions(copiedSecondary->getOptions(), secondaryOptions);
}

//==============================================================================
TEST(World, CloneDantzigPgsSolverOptions)
{
  auto source = createCloneContactWorld();
  auto* solver = static_cast<constraint::BoxedLcpConstraintSolver*>(
      source->getConstraintSolver());
  auto dantzig = std::make_shared<constraint::DantzigBoxedLcpSolver>();
  auto pgs = std::make_shared<constraint::PgsBoxedLcpSolver>();
  const constraint::PgsBoxedLcpSolver::Option options(
      90, 3e-8, 4e-5, 3e-10, false);
  pgs->setOption(options);
  solver->setBoxedLcpSolver(dantzig);
  solver->setSecondaryBoxedLcpSolver(pgs);
  auto clone = source->clone();
  const auto* copy = static_cast<const constraint::BoxedLcpConstraintSolver*>(
      clone->getConstraintSolver());
  EXPECT_NE(
      nullptr,
      dynamic_cast<const constraint::DantzigBoxedLcpSolver*>(
          copy->getBoxedLcpSolver().get()));
  EXPECT_NE(dantzig, copy->getBoxedLcpSolver());
  EXPECT_NE(pgs, copy->getSecondaryBoxedLcpSolver());
  expectPgsOptionsEqual(copy->getSecondaryBoxedLcpSolver(), options);
  expectCloneContactStepsIdentical(source, clone);
}

//==============================================================================
TEST(World, ClonePgsDantzigSolvers)
{
  auto source = createCloneContactWorld();
  auto* solver = static_cast<constraint::BoxedLcpConstraintSolver*>(
      source->getConstraintSolver());
  auto pgs = std::make_shared<constraint::PgsBoxedLcpSolver>();
  auto dantzig = std::make_shared<constraint::DantzigBoxedLcpSolver>();
  solver->setBoxedLcpSolver(pgs);
  solver->setSecondaryBoxedLcpSolver(dantzig);
  auto clone = source->clone();
  const auto* copy = static_cast<const constraint::BoxedLcpConstraintSolver*>(
      clone->getConstraintSolver());
  expectPgsOptionsEqual(copy->getBoxedLcpSolver(), pgs->getOption());
  EXPECT_NE(pgs, copy->getBoxedLcpSolver());
  EXPECT_NE(
      nullptr,
      dynamic_cast<const constraint::DantzigBoxedLcpSolver*>(
          copy->getSecondaryBoxedLcpSolver().get()));
  EXPECT_NE(dantzig, copy->getSecondaryBoxedLcpSolver());
  expectCloneContactStepsIdentical(source, clone);
}

//==============================================================================
TEST(World, CloneSplitImpulse)
{
  auto source = createCloneContactWorld();
  source->getConstraintSolver()->setSplitImpulseEnabled(true);
  auto clone = source->clone();
  EXPECT_TRUE(clone->getConstraintSolver()->isSplitImpulseEnabled());
  expectCloneContactStepsIdentical(source, clone);
}

//==============================================================================
TEST(World, CloneMatrixFreeSolverOptions)
{
  auto source = createCloneContactWorld();
  auto* solver = static_cast<constraint::BoxedLcpConstraintSolver*>(
      source->getConstraintSolver());
  constraint::BoxedLcpConstraintSolver::MatrixFreeContactSolverOptions options;
  options.mEnabled = true;
  options.mMinRows = 3u;
  options.mMaxIterations = 80;
  options.mSor = 0.75;
  options.mDeltaTolerance = 2e-8;
  options.mRelativeDeltaTolerance = 5e-5;
  options.mEpsilonForDivision = 4e-10;
  solver->setMatrixFreeContactSolverOptions(options);
  auto clone = source->clone();
  const auto* copy = static_cast<const constraint::BoxedLcpConstraintSolver*>(
      clone->getConstraintSolver());
  const auto actual = copy->getMatrixFreeContactSolverOptions();
  EXPECT_EQ(options.mEnabled, actual.mEnabled);
  EXPECT_EQ(options.mMinRows, actual.mMinRows);
  EXPECT_EQ(options.mMaxIterations, actual.mMaxIterations);
  EXPECT_EQ(options.mSor, actual.mSor);
  EXPECT_EQ(options.mDeltaTolerance, actual.mDeltaTolerance);
  EXPECT_EQ(options.mRelativeDeltaTolerance, actual.mRelativeDeltaTolerance);
  EXPECT_EQ(options.mEpsilonForDivision, actual.mEpsilonForDivision);
  expectCloneContactStepsIdentical(source, clone);
}

//==============================================================================
TEST(World, CloneCollisionOptions)
{
  auto source = createCloneContactWorld();
  auto& options = source->getConstraintSolver()->getCollisionOption();
  options.enableContact = false;
  options.maxNumContacts = 23u;
  options.maxNumContactsPerPair = 2u;
  options.allowNegativePenetrationDepthContacts = true;
  auto clone = source->clone();
  const auto& actual = clone->getConstraintSolver()->getCollisionOption();
  EXPECT_EQ(options.enableContact, actual.enableContact);
  EXPECT_EQ(options.maxNumContacts, actual.maxNumContacts);
  EXPECT_EQ(options.maxNumContactsPerPair, actual.maxNumContactsPerPair);
  EXPECT_EQ(
      options.allowNegativePenetrationDepthContacts,
      actual.allowNegativePenetrationDepthContacts);
  EXPECT_NE(options.collisionFilter, actual.collisionFilter);

  options.enableContact = true;
  clone = source->clone();
  expectCloneContactStepsIdentical(source, clone);
}

//==============================================================================
TEST(World, CloneSolverTimeStepAndThreads)
{
  auto source = World::create();
  source->getConstraintSolver()->setTimeStep(0.002);
  source->getConstraintSolver()->setNumSimulationThreads(2u);
  auto clone = source->clone();
  EXPECT_EQ(source->getTimeStep(), clone->getTimeStep());
  EXPECT_EQ(
      source->getNumSimulationThreads(), clone->getNumSimulationThreads());
  EXPECT_EQ(0.002, clone->getConstraintSolver()->getTimeStep());
  EXPECT_EQ(2u, clone->getConstraintSolver()->getNumSimulationThreads());
}

//==============================================================================
TEST(World, CloneCustomBackendKeepsDefaultWithWarning)
{
  for (bool primary : {true, false}) {
    SCOPED_TRACE(primary);
    auto source = createCloneContactWorld();
    auto* solver = static_cast<constraint::BoxedLcpConstraintSolver*>(
        source->getConstraintSolver());
    auto custom = std::make_shared<CustomCloneBackend>();
    if (primary)
      solver->setBoxedLcpSolver(custom);
    else
      solver->setSecondaryBoxedLcpSolver(custom);

    testing::internal::CaptureStderr();
    auto clone = source->clone();
    const auto warning = testing::internal::GetCapturedStderr();
    EXPECT_NE(std::string::npos, warning.find("CustomCloneBackend"));
    const auto firstWarning = warning.find("[World::clone]");
    ASSERT_NE(std::string::npos, firstWarning);
    EXPECT_EQ(
        std::string::npos, warning.find("[World::clone]", firstWarning + 1u));
    const auto* copy = static_cast<const constraint::BoxedLcpConstraintSolver*>(
        clone->getConstraintSolver());
    EXPECT_EQ(
        constraint::DantzigBoxedLcpSolver::getStaticType(),
        copy->getBoxedLcpSolver()->getType());
    EXPECT_EQ(
        constraint::PgsBoxedLcpSolver::getStaticType(),
        copy->getSecondaryBoxedLcpSolver()->getType());
    expectCloneContactStepsIdentical(source, clone);
  }
}

//==============================================================================
TEST(World, CloneNonBoxedConstraintSolverKeepsDefault)
{
  auto source = World::create();
  source->setConstraintSolver(std::make_unique<CustomCloneConstraintSolver>());
  source->getConstraintSolver()->setSplitImpulseEnabled(true);
  source->getConstraintSolver()->getCollisionOption().maxNumContacts = 17u;
  testing::internal::CaptureStderr();
  auto clone = source->clone();
  const auto warning = testing::internal::GetCapturedStderr();
  EXPECT_TRUE(warning.empty());
  EXPECT_NE(
      nullptr,
      dynamic_cast<constraint::BoxedLcpConstraintSolver*>(
          clone->getConstraintSolver()));
  EXPECT_FALSE(clone->getConstraintSolver()->isSplitImpulseEnabled());
  EXPECT_EQ(
      1000u, clone->getConstraintSolver()->getCollisionOption().maxNumContacts);
}

//==============================================================================
TEST(World, AddingAndRemovingSkeletons)
{
  // World
  WorldPtr world = World::create();

  //-------------------- Test World::removeSkeleton() ------------------------
  SkeletonPtr skeleton1 = createThreeLinkRobot(
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_X,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_Y,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_Z,
      false,
      false);

  SkeletonPtr skeleton2 = createThreeLinkRobot(
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_X,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_Y,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_Z,
      false,
      false);

  SkeletonPtr skeleton3 = createThreeLinkRobot(
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_X,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_Y,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_Z,
      false,
      false);

  SkeletonPtr skeleton4 = createThreeLinkRobot(
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_X,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_Y,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_Z,
      false,
      false);

  int nSteps = 20;

  // Empty world
  for (int i = 0; i < nSteps; ++i)
    world->step();

  EXPECT_FALSE(world->hasSkeleton(skeleton1));
  EXPECT_FALSE(world->hasSkeleton(skeleton2));
  EXPECT_FALSE(world->hasSkeleton(skeleton3));
  EXPECT_FALSE(world->hasSkeleton(skeleton4));

  // Add skeleton1, skeleton2
  world->addSkeleton(skeleton1);
  EXPECT_TRUE(world->hasSkeleton(skeleton1));
  world->addSkeleton(skeleton2);
  EXPECT_TRUE(world->hasSkeleton(skeleton2));
  EXPECT_TRUE(world->getNumSkeletons() == 2);
  for (int i = 0; i < nSteps; ++i)
    world->step();

  std::string s1name = skeleton1->getName();
  std::string s2name = skeleton2->getName();
  EXPECT_TRUE(skeleton1 == world->getSkeleton(s1name));
  EXPECT_TRUE(skeleton2 == world->getSkeleton(s2name));

  // Remove skeleton2
  world->removeSkeleton(skeleton2);
  EXPECT_TRUE(world->getNumSkeletons() == 1);
  for (int i = 0; i < nSteps; ++i)
    world->step();

  EXPECT_TRUE(skeleton1 == world->getSkeleton(s1name));
  EXPECT_FALSE(skeleton2 == world->getSkeleton(s2name));
  EXPECT_TRUE(world->getSkeleton(s2name) == nullptr);

  // Add skeleton3, skeleton4
  world->addSkeleton(skeleton3);
  EXPECT_TRUE(world->hasSkeleton(skeleton3));
  world->addSkeleton(skeleton4);
  EXPECT_TRUE(world->hasSkeleton(skeleton4));
  EXPECT_TRUE(world->getNumSkeletons() == 3);
  for (int i = 0; i < nSteps; ++i)
    world->step();

  std::string s3name = skeleton3->getName();
  std::string s4name = skeleton4->getName();

  EXPECT_TRUE(s3name == s2name);
  EXPECT_TRUE(skeleton3 == world->getSkeleton(s3name));
  EXPECT_TRUE(skeleton4 == world->getSkeleton(s4name));

  skeleton4->setName(skeleton1->getName());
  EXPECT_FALSE(skeleton4->getName() == skeleton1->getName());

  // Remove skeleton1
  world->removeSkeleton(skeleton1);
  EXPECT_TRUE(world->getNumSkeletons() == 2);
  for (int i = 0; i < nSteps; ++i)
    world->step();

  EXPECT_FALSE(skeleton1 == world->getSkeleton(s1name));
  EXPECT_TRUE(world->getSkeleton(s1name) == nullptr);

  // Remove all the skeletons
  world->removeAllSkeletons();
  EXPECT_EQ((int)world->getNumSkeletons(), 0);
  for (int i = 0; i < nSteps; ++i)
    world->step();

  EXPECT_FALSE(skeleton3 == world->getSkeleton(s3name));
  EXPECT_TRUE(world->getSkeleton(s3name) == nullptr);

  EXPECT_FALSE(skeleton4 == world->getSkeleton(s4name));
  EXPECT_TRUE(world->getSkeleton(s4name) == nullptr);

  // An error will be thrown here if Skeletons are not being removed correctly
  skeleton1->setName(skeleton4->getName());
}

//==============================================================================
TEST(World, Cloning)
{
  // Seed random generator for deterministic tests
  Random::setSeed(42);

  // Create a list of skel files to test with
  std::vector<common::Uri> fileList;
  fileList.push_back("dart://sample/skel/test/chainwhipa.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum.skel");
  fileList.push_back(
      "dart://sample/skel/test/single_pendulum_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum.skel");
  fileList.push_back(
      "dart://sample/skel/test/double_pendulum_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum_ball_joint.skel");
  fileList.push_back(
      "dart://sample/skel/test/serial_chain_revolute_joint.skel");
  fileList.push_back(
      "dart://sample/skel/test/serial_chain_eulerxyz_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint_20.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint_40.skel");
  fileList.push_back("dart://sample/skel/test/simple_tree_structure.skel");
  fileList.push_back(
      "dart://sample/skel/test/simple_tree_structure_euler_joint.skel");
  fileList.push_back(
      "dart://sample/skel/test/simple_tree_structure_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure_ball_joint.skel");
  fileList.push_back("dart://sample/skel/fullbody1.skel");

  std::vector<dart::simulation::WorldPtr> worlds;
  for (std::size_t i = 0; i < fileList.size(); ++i)
    worlds.push_back(utils::SkelParser::readWorld(fileList[i]));

  for (std::size_t i = 0; i < worlds.size(); ++i) {
    dart::simulation::WorldPtr original = worlds[i];
    std::vector<dart::simulation::WorldPtr> clones;
    clones.push_back(original);
    for (std::size_t j = 1; j < 5; ++j)
      clones.push_back(clones[j - 1]->clone());

#ifdef NDEBUG
    std::size_t numIterations = 500;
#else
    std::size_t numIterations = 3;
#endif

    for (std::size_t j = 0; j < numIterations; ++j) {
      for (std::size_t k = 0; k < original->getNumSkeletons(); ++k) {
        dart::dynamics::SkeletonPtr skel = original->getSkeleton(k);

        // Generate a random command vector
        Eigen::VectorXd commands = skel->getCommands();
        for (int q = 0; q < commands.size(); ++q)
          commands[q] = Random::uniform(-0.1, 0.1);

        // Assign the command vector to each clone of the kth skeleton
        for (std::size_t c = 0; c < clones.size(); ++c) {
          dart::dynamics::SkeletonPtr skelClone = clones[c]->getSkeleton(k);
          skelClone->setCommands(commands);
        }
      }

      // Step each clone forward
      for (std::size_t c = 0; c < clones.size(); ++c)
        clones[c]->step(false);
    }

    for (std::size_t c = 0; c < clones.size(); ++c) {
      for (std::size_t k = 0; k < original->getNumSkeletons(); ++k) {
        dart::dynamics::SkeletonPtr skel = original->getSkeleton(k);
        dart::dynamics::SkeletonPtr clone = clones[c]->getSkeleton(k);

        EXPECT_TRUE(equals(skel->getPositions(), clone->getPositions(), 0));
        EXPECT_TRUE(equals(skel->getVelocities(), clone->getVelocities(), 0));
        EXPECT_TRUE(
            equals(skel->getAccelerations(), clone->getAccelerations(), 0));
        EXPECT_TRUE(equals(skel->getForces(), clone->getForces(), 0));
      }
    }
  }
}

//==============================================================================
TEST(World, ValidatingClones)
{
  // Create a list of skel files to test with
  std::vector<common::Uri> fileList;
  fileList.push_back("dart://sample/skel/test/chainwhipa.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum.skel");
  fileList.push_back(
      "dart://sample/skel/test/single_pendulum_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum.skel");
  fileList.push_back(
      "dart://sample/skel/test/double_pendulum_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum_ball_joint.skel");
  fileList.push_back(
      "dart://sample/skel/test/serial_chain_revolute_joint.skel");
  fileList.push_back(
      "dart://sample/skel/test/serial_chain_eulerxyz_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint_20.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint_40.skel");
  fileList.push_back("dart://sample/skel/test/simple_tree_structure.skel");
  fileList.push_back(
      "dart://sample/skel/test/simple_tree_structure_euler_joint.skel");
  fileList.push_back(
      "dart://sample/skel/test/simple_tree_structure_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure_ball_joint.skel");
  fileList.push_back("dart://sample/skel/fullbody1.skel");

  std::vector<dart::simulation::WorldPtr> worlds;
  for (std::size_t i = 0; i < fileList.size(); ++i) {
    worlds.push_back(utils::SkelParser::readWorld(fileList[i]));

    // Set non default collision detector
#if HAVE_BULLET
    worlds.back()->setCollisionDetector(CollisionDetectorType::Bullet);
#else
    worlds.back()->setCollisionDetector(CollisionDetectorType::Dart);
#endif
  }

  for (std::size_t i = 0; i < worlds.size(); ++i) {
    dart::simulation::WorldPtr original = worlds[i];
    std::vector<dart::simulation::WorldPtr> clones;
    clones.push_back(original);
    for (std::size_t j = 1; j < 5; ++j) {
      clones.push_back(clones[j - 1]->clone());

      auto originalCD = original->getCollisionDetector();
      auto cloneCD = clones.back()->getCollisionDetector();

      std::string originalCDType = originalCD->getType();
      std::string cloneCDType = cloneCD->getType();

      EXPECT_EQ(originalCDType, cloneCDType);
    }
  }
}

//==============================================================================
TEST(World, SetCollisionDetectorByType)
{
  auto factory = collision::CollisionDetector::getFactory();
  ASSERT_NE(factory, nullptr);

  if (!factory->canCreate("dart"))
    GTEST_SKIP() << "dart collision detector is not available in this build";

  auto world = World::create();
  world->setCollisionDetector(CollisionDetectorType::Dart);

  ASSERT_TRUE(world->getCollisionDetector());
  EXPECT_EQ(world->getCollisionDetector()->getType(), "dart");
}

//==============================================================================
TEST(World, ConfiguresCollisionDetectorViaConfig)
{
  auto factory = collision::CollisionDetector::getFactory();
  ASSERT_NE(factory, nullptr);

  if (!factory->canCreate("dart"))
    GTEST_SKIP() << "dart collision detector is not available in this build";

  WorldConfig config;
  config.name = "configured-world";
  config.collisionDetector = CollisionDetectorType::Dart;
  auto world = World::create(config);
  ASSERT_TRUE(world->getCollisionDetector());
  EXPECT_EQ(world->getCollisionDetector()->getType(), "dart");
}

//==============================================================================
TEST(World, DefaultWorldUsesFclPrimitive)
{
  auto factory = collision::CollisionDetector::getFactory();
  ASSERT_NE(factory, nullptr);

  if (!factory->canCreate("fcl"))
    GTEST_SKIP() << "fcl collision detector is not available in this build";

  // The default World construction path must remain unchanged: it keeps the
  // constraint solver's default FCL detector configured with PRIMITIVE shapes.
  auto world = World::create();
  auto fclDetector = std::dynamic_pointer_cast<collision::FCLCollisionDetector>(
      world->getCollisionDetector());
  ASSERT_TRUE(fclDetector);
  EXPECT_EQ(
      fclDetector->getPrimitiveShapeType(),
      collision::FCLCollisionDetector::PRIMITIVE);
}

//==============================================================================
TEST(World, UnreleasedNativeSkelAliasFallsBackToFcl)
{
  auto factory = collision::CollisionDetector::getFactory();
  ASSERT_NE(factory, nullptr);

  ASSERT_TRUE(
      factory->canCreate(collision::DARTCollisionDetector::getStaticType()));
  ASSERT_FALSE(factory->canCreate("native"));

  const std::string skel = R"(
<skel version="1.0">
  <world name="world">
    <physics>
      <time_step>0.001</time_step>
      <gravity>0 0 -9.81</gravity>
      <collision_detector>native</collision_detector>
    </physics>
  </world>
</skel>
)";

  auto world = utils::SkelParser::readWorldXML(skel);
  ASSERT_NE(nullptr, world);

  auto fclDetector = std::dynamic_pointer_cast<collision::FCLCollisionDetector>(
      world->getCollisionDetector());
  ASSERT_TRUE(fclDetector);
  EXPECT_EQ(
      collision::FCLCollisionDetector::PRIMITIVE,
      fclDetector->getPrimitiveShapeType());
}

//==============================================================================
TEST(World, TypedSetterToFclKeepsPrimitive)
{
  auto factory = collision::CollisionDetector::getFactory();
  ASSERT_NE(factory, nullptr);

  if (!factory->canCreate("fcl"))
    GTEST_SKIP() << "fcl collision detector is not available in this build";

  auto world = World::create();
  world->setCollisionDetector(CollisionDetectorType::Dart);
  world->setCollisionDetector(CollisionDetectorType::Fcl);

  // The typed setter must produce a detector with the same default shape type
  // as the existing FCL factory default (PRIMITIVE), not change the established
  // default.
  auto fclDetector = std::dynamic_pointer_cast<collision::FCLCollisionDetector>(
      world->getCollisionDetector());
  ASSERT_TRUE(fclDetector);
  EXPECT_EQ(
      fclDetector->getPrimitiveShapeType(),
      collision::FCLCollisionDetector::PRIMITIVE);
}

//==============================================================================
TEST(World, CollisionDetectorChangeInvalidatesSimulationMode)
{
  auto world = World::create();
  world->enterSimulationMode();
  ASSERT_TRUE(world->isInSimulationMode());

  world->setCollisionDetector(collision::DARTCollisionDetector::create());
  EXPECT_FALSE(world->isInSimulationMode());

  world->enterSimulationMode();
  ASSERT_TRUE(world->isInSimulationMode());
  const auto* preparedCollisionGroup
      = world->getConstraintSolver()->getCollisionGroup().get();

  world->getConstraintSolver()->setCollisionDetector(
      collision::DARTCollisionDetector::create());
  ASSERT_NE(
      preparedCollisionGroup,
      world->getConstraintSolver()->getCollisionGroup().get());
  EXPECT_FALSE(world->isInSimulationMode());
}

//==============================================================================
TEST(World, TypedSetterKeepsCurrentWhenDetectorUnavailable)
{
  ScopedCollisionFactoryDisabler disableDart(
      collision::DARTCollisionDetector::getStaticType(),
      []() -> collision::CollisionDetectorPtr {
        return collision::DARTCollisionDetector::create();
      });

  if (!disableDart.wasDisabled())
    GTEST_SKIP() << "dart collision detector is not registered in this build";

  auto world = World::create();
  auto original = world->getCollisionDetector();
  ASSERT_TRUE(original);

  world->setCollisionDetector(CollisionDetectorType::Dart);

  auto current = world->getCollisionDetector();
  ASSERT_TRUE(current);
  EXPECT_EQ(current->getType(), original->getType());
}

//==============================================================================
TEST(World, ConfigKeepsDefaultWhenPreferredDetectorUnavailable)
{
  ScopedCollisionFactoryDisabler disableDart(
      collision::DARTCollisionDetector::getStaticType(),
      []() -> collision::CollisionDetectorPtr {
        return collision::DARTCollisionDetector::create();
      });

  if (!disableDart.wasDisabled())
    GTEST_SKIP() << "dart collision detector is not registered in this build";

  WorldConfig config;
  config.name = "fallback-pref";
  config.collisionDetector = CollisionDetectorType::Dart;

  // When the preferred detector is unavailable, the World keeps the constraint
  // solver's existing default detector (FCL), unchanged.
  auto world = World::create(config);
  ASSERT_TRUE(world->getCollisionDetector());
  EXPECT_EQ(
      world->getCollisionDetector()->getType(),
      collision::FCLCollisionDetector::getStaticType());
}

//==============================================================================
simulation::WorldPtr createWorld()
{
  // Create and initialize the world
  simulation::WorldPtr world
      = utils::SkelParser::readWorld("dart://sample/skel/chain.skel");
  DART_ASSERT(world != nullptr);

  // Create and initialize the world
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->setTimeStep(1.0 / 2000);

  const auto dof = world->getSkeleton(0)->getNumDofs();
  Eigen::VectorXd initPose = Eigen::VectorXd::Zero(static_cast<int>(dof));
  initPose[20] = 3.14159 * 0.4;
  initPose[23] = 3.14159 * 0.4;
  initPose[26] = 3.14159 * 0.4;
  initPose[29] = 3.14159 * 0.4;
  world->getSkeleton(0)->setPositions(initPose);

  // Create a ball joint constraint
  BodyNode* bd1 = world->getSkeleton(0)->getBodyNode("link 6");
  BodyNode* bd2 = world->getSkeleton(0)->getBodyNode("link 10");
  EXPECT_TRUE(bd1 != nullptr);
  EXPECT_TRUE(bd2 != nullptr);
  const Eigen::Vector3d offset(0.0, 0.025, 0.0);
  const Eigen::Vector3d jointPos = bd1->getTransform() * offset;
  auto cl
      = std::make_shared<constraint::BallJointConstraint>(bd1, bd2, jointPos);
  world->getConstraintSolver()->addConstraint(cl);

  return world;
}

//==============================================================================
TEST(World, SetNewConstraintSolver)
{
  auto world = createWorld();
  EXPECT_TRUE(world->getConstraintSolver()->getSkeletons().size() == 1);
  EXPECT_TRUE(world->getConstraintSolver()->getNumConstraints() == 1);

  auto solver1 = std::make_unique<constraint::BoxedLcpConstraintSolver>(
      std::make_shared<constraint::DantzigBoxedLcpSolver>());
  EXPECT_TRUE(solver1->getSkeletons().size() == 0);
  EXPECT_TRUE(solver1->getNumConstraints() == 0);

  world->setConstraintSolver(std::move(solver1));
  EXPECT_TRUE(world->getConstraintSolver()->getSkeletons().size() == 1);
  EXPECT_TRUE(world->getConstraintSolver()->getNumConstraints() == 1);

  auto solver2 = std::make_unique<constraint::BoxedLcpConstraintSolver>(
      std::make_shared<constraint::PgsBoxedLcpSolver>());
  EXPECT_TRUE(solver2->getSkeletons().size() == 0);
  EXPECT_TRUE(solver2->getNumConstraints() == 0);

  world->setConstraintSolver(std::move(solver2));
  EXPECT_TRUE(world->getConstraintSolver()->getSkeletons().size() == 1);
  EXPECT_TRUE(world->getConstraintSolver()->getNumConstraints() == 1);
}

//==============================================================================
simulation::WorldPtr createWorldWithRevoluteConstraint()
{
  simulation::WorldPtr world
      = utils::SkelParser::readWorld("dart://sample/skel/chain.skel");
  DART_ASSERT(world != nullptr);

  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->setTimeStep(1.0 / 2000);

  const auto dof = world->getSkeleton(0)->getNumDofs();
  Eigen::VectorXd initPose = Eigen::VectorXd::Zero(static_cast<int>(dof));
  initPose[20] = 3.14159 * 0.4;
  initPose[23] = 3.14159 * 0.4;
  initPose[26] = 3.14159 * 0.4;
  initPose[29] = 3.14159 * 0.4;
  world->getSkeleton(0)->setPositions(initPose);

  BodyNode* bd1 = world->getSkeleton(0)->getBodyNode("link 6");
  BodyNode* bd2 = world->getSkeleton(0)->getBodyNode("link 10");
  EXPECT_TRUE(bd1 != nullptr);
  EXPECT_TRUE(bd2 != nullptr);

  const Eigen::Vector3d offset(0.0, 0.025, 0.0);
  const Eigen::Vector3d jointPos = bd1->getTransform() * offset;

  auto hinge = std::make_shared<constraint::RevoluteJointConstraint>(
      bd1, bd2, jointPos, Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
  world->getConstraintSolver()->addConstraint(hinge);

  return world;
}

//==============================================================================
TEST(World, RevoluteJointConstraintBasics)
{
  auto world = createWorldWithRevoluteConstraint();
  ASSERT_TRUE(world);
  ASSERT_EQ(world->getConstraintSolver()->getNumConstraints(), 1);

  auto* bd1 = world->getSkeleton(0)->getBodyNode("link 6");
  auto* bd2 = world->getSkeleton(0)->getBodyNode("link 10");
  const Eigen::Vector3d offset(0.0, 0.025, 0.0);

  for (int i = 0; i < 200; ++i)
    world->step();

  const Eigen::Vector3d pos1 = bd1->getTransform() * offset;
  const Eigen::Vector3d pos2 = bd2->getTransform() * offset;
  EXPECT_LT((pos1 - pos2).norm(), 5e-2);

  const Eigen::Vector3d axis1
      = bd1->getTransform().linear() * Eigen::Vector3d::UnitY();
  const Eigen::Vector3d axis2
      = bd2->getTransform().linear() * Eigen::Vector3d::UnitY();
  EXPECT_GT(axis1.normalized().dot(axis2.normalized()), 0.2);
}

//==============================================================================
// Exercises the single-body RevoluteJointConstraint constructor, which anchors
// one body to a fixed point in the world about a hinge axis. The body-to-body
// test above does not cover this overload.
TEST(World, RevoluteJointConstraintSingleBodyAnchorsToWorld)
{
  simulation::WorldPtr world
      = utils::SkelParser::readWorld("dart://sample/skel/chain.skel");
  ASSERT_TRUE(world != nullptr);

  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->setTimeStep(1.0 / 2000);

  BodyNode* body = world->getSkeleton(0)->getBodyNode("link 6");
  ASSERT_TRUE(body != nullptr);

  const Eigen::Vector3d offset(0.0, 0.025, 0.0);
  const Eigen::Vector3d anchorWorld = body->getTransform() * offset;

  auto hinge = std::make_shared<constraint::RevoluteJointConstraint>(
      body, anchorWorld, Eigen::Vector3d::UnitZ());
  world->getConstraintSolver()->addConstraint(hinge);
  ASSERT_EQ(world->getConstraintSolver()->getNumConstraints(), 1);

  for (int i = 0; i < 200; ++i)
    world->step();

  // The anchored material point on the body must stay near the fixed world
  // anchor despite gravity (the revolute constraint pins it to the world).
  const Eigen::Vector3d anchorNow = body->getTransform() * offset;
  EXPECT_LT((anchorNow - anchorWorld).norm(), 5e-2);
}

//==============================================================================
TEST(World, SimulationThreadCount)
{
  auto world = World::create();
  EXPECT_EQ(world->getNumSimulationThreads(), 1u);

  world->setNumSimulationThreads(2u);
  EXPECT_EQ(world->getNumSimulationThreads(), 2u);

  // Add enough independent skeletons to exercise the parallel per-skeleton
  // path. Empty skeletons keep the test focused on World's scheduling surface.
  for (std::size_t i = 0; i < 130u; ++i) {
    world->addSkeleton(Skeleton::create("empty_" + std::to_string(i)));
  }

  world->step();
  EXPECT_EQ(world->getSimFrames(), 1);

  auto clone = world->clone();
  ASSERT_TRUE(clone != nullptr);
  EXPECT_EQ(clone->getNumSimulationThreads(), 2u);

  world->setNumSimulationThreads(1u);
  EXPECT_EQ(world->getNumSimulationThreads(), 1u);
}

//==============================================================================
namespace {

struct UnexpectedStepTopologyMutation
{
};

void expectStepRejectsChildCreation(
    const SkeletonPtr& skeleton, BodyNode* parent)
{
  try {
    skeleton->createJointAndBodyNodePair<WeldJoint>(parent);
  } catch (const std::runtime_error& error) {
    EXPECT_EQ(std::string(error.what()).find("WorldBusyError: "), 0u);
    return;
  }
  ADD_FAILURE() << "Topology changed during a simulation callback";
  // Abort the step before its remaining stages consume stale model buffers.
  throw UnexpectedStepTopologyMutation{};
}

} // namespace

//==============================================================================
TEST(World, LazyStepCallbacksKeepBusyGuardOnSimulationWorkers)
{
  for (const auto threads : {1u, 2u}) {
    for (const bool deactivation : {false, true}) {
      SCOPED_TRACE(threads);
      SCOPED_TRACE(deactivation);
      auto world = World::create();
      world->setNumSimulationThreads(threads);
      auto options = world->getDeactivationOptions();
      options.mEnabled = deactivation;
      world->setDeactivationOptions(options);
      world->setGravity(Eigen::Vector3d(0, 0, -9.81));
      for (std::size_t i = 0; i < 128u; ++i) {
        auto skeleton = Skeleton::create();
        skeleton->createJointAndBodyNodePair<FreeJoint>();
        world->addSkeleton(skeleton);
      }
      auto target = world->getSkeleton(127u);
      auto* body = target->getBodyNode(0u);
      // BodyNode does not emit its own velocity signal; its child Frame does.
      auto frame = std::make_shared<SimpleFrame>(body);
      world->enterSimulationMode();
      body->getSpatialVelocity();
      ASSERT_FALSE(world->isCompiled());
      ASSERT_TRUE(world->isInSimulationMode());
      const auto topology = world->getTopologyRevision();
      const auto parameters = world->getParameterRevision();
      const auto caller = std::this_thread::get_id();
      std::size_t callerCallbacks = 0u;
      std::size_t workerCallbacks = 0u;
      auto connection = frame->onVelocityChanged.connect([&](const Entity*) {
        if (std::this_thread::get_id() == caller)
          ++callerCallbacks;
        else
          ++workerCallbacks;
        expectStepRejectsChildCreation(target, body);
      });
      EXPECT_NO_THROW(world->step());
      connection.disconnect();
      // Velocity and position integration must both keep the guard.
      if (threads == 2u) {
        EXPECT_GE(workerCallbacks, 2u);
        EXPECT_EQ(callerCallbacks, 0u);
      } else {
        EXPECT_GE(callerCallbacks, 2u);
        EXPECT_EQ(workerCallbacks, 0u);
      }
      EXPECT_EQ(target->getNumBodyNodes(), 1u);
      EXPECT_EQ(target->getNumDofs(), 6u);
      std::size_t bodies = 0u;
      for (std::size_t i = 0; i < world->getNumSkeletons(); ++i)
        bodies += world->getSkeleton(i)->getNumBodyNodes();
      EXPECT_EQ(bodies, 128u);
      EXPECT_EQ(world->getTopologyRevision(), topology);
      EXPECT_EQ(world->getParameterRevision(), parameters);
      EXPECT_FALSE(world->isCompiled());
      EXPECT_TRUE(world->isInSimulationMode());
      EXPECT_NO_THROW(target->createJointAndBodyNodePair<WeldJoint>(body));
      EXPECT_EQ(target->getNumBodyNodes(), 2u);
    }
  }
}

//==============================================================================
TEST(World, NestedWorkerWorldStepsRestoreBusyGuardAfterExceptions)
{
  struct CallbackFailure
  {
  };
  for (const bool deactivation : {false, true}) {
    SCOPED_TRACE(deactivation);
    auto outer = World::create();
    outer->setNumSimulationThreads(2u);
    auto inner = World::create();
    auto options = outer->getDeactivationOptions();
    options.mEnabled = deactivation;
    outer->setDeactivationOptions(options);
    inner->setDeactivationOptions(options);
    for (std::size_t i = 0; i < 128u; ++i) {
      auto skeleton = Skeleton::create();
      skeleton->createJointAndBodyNodePair<FreeJoint>();
      outer->addSkeleton(skeleton);
    }
    auto target = outer->getSkeleton(127u);
    auto* body = target->getBodyNode(0u);
    auto outerFrame = std::make_shared<SimpleFrame>(body);
    auto nested = Skeleton::create();
    auto* nestedBody = nested->createJointAndBodyNodePair<FreeJoint>().second;
    inner->addSkeleton(nested);
    auto innerFrame = std::make_shared<SimpleFrame>(nestedBody);
    outer->enterSimulationMode();
    inner->enterSimulationMode();
    body->getSpatialVelocity();
    nestedBody->getSpatialVelocity();
    const auto caller = std::this_thread::get_id();
    bool throwOnce = true;
    std::size_t innerCallbacks = 0u;
    std::size_t outerCallbacks = 0u;
    auto innerConnection
        = innerFrame->onVelocityChanged.connect([&](const Entity*) {
            ++innerCallbacks;
            EXPECT_NE(std::this_thread::get_id(), caller);
            expectStepRejectsChildCreation(target, body);
            expectStepRejectsChildCreation(nested, nestedBody);
            if (std::exchange(throwOnce, false))
              throw CallbackFailure{};
          });
    auto outerConnection
        = outerFrame->onVelocityChanged.connect([&](const Entity*) {
            ++outerCallbacks;
            EXPECT_NE(std::this_thread::get_id(), caller);
            try {
              inner->step();
            } catch (const CallbackFailure&) {
              // The inner BusyScope must restore this worker's outer chain.
              expectStepRejectsChildCreation(target, body);
              throw;
            }
            expectStepRejectsChildCreation(target, body);
          });
    EXPECT_THROW(outer->step(), CallbackFailure);
    EXPECT_GT(innerCallbacks, 0u);
    EXPECT_GT(outerCallbacks, 0u);
    EXPECT_EQ(target->getNumBodyNodes(), 1u);
    EXPECT_EQ(nested->getNumBodyNodes(), 1u);
    EXPECT_TRUE(outer->isInSimulationMode());
    EXPECT_TRUE(inner->isInSimulationMode());
    EXPECT_FALSE(outer->isCompiled());
    EXPECT_FALSE(inner->isCompiled());
    const auto previousInnerCallbacks = innerCallbacks;
    const auto previousOuterCallbacks = outerCallbacks;
    body->getSpatialVelocity();
    nestedBody->getSpatialVelocity();
    EXPECT_NO_THROW(outer->step());
    EXPECT_GT(innerCallbacks, previousInnerCallbacks);
    EXPECT_GT(outerCallbacks, previousOuterCallbacks);
    EXPECT_EQ(target->getNumBodyNodes(), 1u);
    EXPECT_EQ(nested->getNumBodyNodes(), 1u);
    outerConnection.disconnect();
    innerConnection.disconnect();
    EXPECT_NO_THROW(target->createJointAndBodyNodePair<WeldJoint>(body));
    EXPECT_NO_THROW(nested->createJointAndBodyNodePair<WeldJoint>(nestedBody));
    EXPECT_EQ(target->getNumBodyNodes(), 2u);
    EXPECT_EQ(nested->getNumBodyNodes(), 2u);
  }
}

//==============================================================================
TEST(World, ManualConstraintGroupsUseSerialIndependentGroupSolve)
{
  EXPECT_EQ(countConcurrentIndependentSolves(false), 1u);
  EXPECT_EQ(countConcurrentIndependentSolves(true), 1u);
}

//==============================================================================
// Regression test for https://github.com/dartsim/dart/issues/2426
TEST(World, GetIndexBoundsCheck)
{
  auto world = World::create();

  auto skel1 = Skeleton::create("skel1");
  skel1->createJointAndBodyNodePair<RevoluteJoint>();
  world->addSkeleton(skel1);

  auto skel2 = Skeleton::create("skel2");
  skel2->createJointAndBodyNodePair<RevoluteJoint>();
  world->addSkeleton(skel2);

  EXPECT_EQ(world->getIndex(0), 0);
  EXPECT_EQ(world->getIndex(1), 1);
  EXPECT_EQ(world->getIndex(2), 2);

#ifdef NDEBUG
  // Release mode: DART_ASSERT doesn't abort, so we can test return values
  EXPECT_EQ(world->getIndex(-1), -1);
  EXPECT_EQ(world->getIndex(100), -1);
#else
  // Debug mode: DART_ASSERT(false) aborts, so use death tests to verify
  // the error handling code path is exercised (improves coverage)
  EXPECT_DEATH(world->getIndex(-1), "");
  EXPECT_DEATH(world->getIndex(100), "");
#endif
}

//==============================================================================
// Regression test for https://github.com/dartsim/dart/issues/3497
TEST(World, GetIndexTracksRemovalAndTopologyChanges)
{
  const auto makeSkeleton = [](const std::string& name, std::size_t numDofs) {
    auto skeleton = Skeleton::create(name);
    for (std::size_t i = 0; i < numDofs; ++i)
      skeleton->createJointAndBodyNodePair<RevoluteJoint>();
    return skeleton;
  };

  auto world = World::create();
  auto first = makeSkeleton("first", 2u);
  auto middle = makeSkeleton("middle", 3u);
  auto last = makeSkeleton("last", 4u);
  world->addSkeleton(first);
  world->addSkeleton(middle);
  world->addSkeleton(last);
  EXPECT_EQ(world->getIndex(3), 9);

  // Growing a contained skeleton shifts every later boundary.
  middle->createJointAndBodyNodePair<RevoluteJoint>(); // middle: 3 -> 4
  EXPECT_EQ(world->getIndex(1), 2);
  EXPECT_EQ(world->getIndex(2), 6);
  EXPECT_EQ(world->getIndex(3), 10);

  // Removing a middle skeleton whose width changed after it was added
  // repairs the whole range, including the terminal total.
  world->removeSkeleton(middle);
  ASSERT_EQ(world->getNumSkeletons(), 2u);
  EXPECT_EQ(world->getIndex(0), 0);
  EXPECT_EQ(world->getIndex(1), 2);
  EXPECT_EQ(world->getIndex(2), 6);

  // Growth of a remaining skeleton after removal is reflected too.
  last->createJointAndBodyNodePair<RevoluteJoint>(); // last: 4 -> 5
  EXPECT_EQ(world->getIndex(2), 7);

  // Stepping (which rebuilds the internal cache) must agree.
  world->step();
  EXPECT_EQ(world->getIndex(2), 7);

  world->removeAllSkeletons();
  EXPECT_EQ(world->getIndex(0), 0);
}

//==============================================================================
// Regression test for https://github.com/dartsim/dart/issues/3498
TEST(World, RecordingDropsFramesWhenSkeletonDofLayoutChanges)
{
  const auto makeSkeleton = [](const std::string& name, double position) {
    auto skeleton = Skeleton::create(name);
    skeleton->createJointAndBodyNodePair<RevoluteJoint>();
    skeleton->setPosition(0, position);
    return skeleton;
  };

  auto world = World::create();
  auto first = makeSkeleton("first", 11.0);
  auto second = makeSkeleton("second", 22.0);
  world->addSkeleton(first);
  world->addSkeleton(second);

  Recording* recording = world->getRecording();
  world->bake();
  ASSERT_EQ(recording->getNumFrames(), 1);
  ASSERT_EQ(recording->getNumSkeletons(), 2);
  EXPECT_DOUBLE_EQ(recording->getGenCoord(0, 0, 0), 11.0);
  EXPECT_DOUBLE_EQ(recording->getGenCoord(0, 1, 0), 22.0);

  // Removing a skeleton changes the per-skeleton DOF layout, so the frame
  // packed under the old layout must not survive to be re-sliced under the
  // new one (row zero would otherwise report 11 for the surviving skeleton).
  world->removeSkeleton(first);
  EXPECT_EQ(recording->getNumSkeletons(), 1);
  EXPECT_EQ(recording->getNumFrames(), 0);

  world->bake();
  EXPECT_EQ(recording->getNumFrames(), 1);
  EXPECT_EQ(recording->getNumDofs(0), 1);
  EXPECT_DOUBLE_EQ(recording->getGenCoord(0, 0, 0), 22.0);

  // Growing a contained skeleton in place is only visible to the recording at
  // the next bake, which must refresh the layout and drop the stale frame.
  second->createJointAndBodyNodePair<RevoluteJoint>(); // second: 1 -> 2 DOFs
  second->setPosition(1, 33.0);
  world->bake();
  EXPECT_EQ(recording->getNumFrames(), 1);
  ASSERT_EQ(recording->getNumDofs(0), 2);
  EXPECT_DOUBLE_EQ(recording->getGenCoord(0, 0, 0), 22.0);
  EXPECT_DOUBLE_EQ(recording->getGenCoord(0, 0, 1), 33.0);

  // A bake without any layout change must keep accumulating frames.
  world->bake();
  EXPECT_EQ(recording->getNumFrames(), 2);

  // Adding a skeleton while frames exist is a layout change as well.
  world->addSkeleton(makeSkeleton("third", 44.0));
  EXPECT_EQ(recording->getNumSkeletons(), 2);
  EXPECT_EQ(recording->getNumFrames(), 0);
}

//==============================================================================
TEST(World, CompileFreezesOwnedGraphBeforeMutation)
{
  auto world = World::create();
  auto robot = createBox(Eigen::Vector3d::Ones());
  world->addSkeleton(robot);
  auto* body = robot->getBodyNode(0);
  auto* shape = body->getShapeNodeWith<CollisionAspect>(0);
  ASSERT_NE(shape, nullptr);
  const auto originalShape = shape->getShape();
  const auto bodies = robot->getNumBodyNodes();
  const auto nodes = body->getNumShapeNodes();
  auto* solver = world->getConstraintSolver();
  const auto detector = world->getCollisionDetector();
  world->compile();
  ASSERT_TRUE(world->isCompiled());

  EXPECT_THROW(
      robot->createJointAndBodyNodePair<RevoluteJoint>(body),
      std::runtime_error);
  EXPECT_EQ(robot->getNumBodyNodes(), bodies);
  EXPECT_THROW(
      body->createShapeNodeWith<CollisionAspect>(originalShape),
      std::runtime_error);
  EXPECT_EQ(body->getNumShapeNodes(), nodes);
  EXPECT_THROW(
      shape->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones())),
      std::runtime_error);
  EXPECT_EQ(shape->getShape(), originalShape);
  EXPECT_THROW(shape->remove(), std::runtime_error);
  EXPECT_EQ(body->getNumShapeNodes(), nodes);
  EXPECT_THROW(shape->removeCollisionAspect(), std::runtime_error);
  EXPECT_TRUE(shape->has<CollisionAspect>());
  EXPECT_THROW(shape->createCollisionAspect(), std::runtime_error);
  EXPECT_THROW(body->remove(), std::runtime_error);
  EXPECT_EQ(robot->getNumBodyNodes(), bodies);
  EXPECT_THROW(world->removeSkeleton(robot), std::runtime_error);
  EXPECT_TRUE(world->hasSkeleton(robot));
  EXPECT_THROW(
      world->setCollisionDetector(CollisionDetectorType::Dart),
      std::runtime_error);
  EXPECT_EQ(world->getCollisionDetector(), detector);
  EXPECT_THROW(
      solver->setCollisionDetector(collision::DARTCollisionDetector::create()),
      std::runtime_error);
  EXPECT_EQ(world->getCollisionDetector(), detector);
  auto constraint = std::make_shared<constraint::BallJointConstraint>(
      body, body->getTransform().translation());
  EXPECT_THROW(solver->addConstraint(constraint), std::runtime_error);
  EXPECT_EQ(solver->getNumConstraints(), 0u);
}

namespace {

// An Aspect type that no model object specializes.
class UnusedAspect final : public common::Aspect
{
public:
  std::unique_ptr<common::Aspect> cloneAspect() const override
  {
    return std::make_unique<UnusedAspect>();
  }
};

} // namespace

//==============================================================================
TEST(World, CompiledSolverAbsentRemovalRemainsNoOp)
{
  auto world = World::create();
  auto resident = createBox(Eigen::Vector3d::Ones());
  auto absent = createBox(Eigen::Vector3d::Ones());
  absent->setName("absent");
  world->addSkeleton(resident);
  world->compile();
  auto* solver = world->getConstraintSolver();
  const auto topology = world->getTopologyRevision();
  const auto parameter = world->getParameterRevision();
  EXPECT_NO_THROW(solver->removeSkeleton(absent));
  EXPECT_NO_THROW(solver->removeSkeletons({absent}));
  ASSERT_EQ(solver->getSkeletons().size(), 1u);
  EXPECT_EQ(solver->getSkeletons()[0], resident);
  EXPECT_EQ(world->getTopologyRevision(), topology);
  EXPECT_EQ(world->getParameterRevision(), parameter);
  EXPECT_TRUE(world->isCompiled());
  // A resident skeleton is still protected.
  EXPECT_THROW(solver->removeSkeleton(resident), std::runtime_error);
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
TEST(World, FrozenModelAcceptsClearingAbsentAspects)
{
  auto world = World::create();
  auto robot = Skeleton::create("robot");
  auto* body = robot->createJointAndBodyNodePair<RevoluteJoint>().second;
  auto* shape = body->createShapeNodeWith<CollisionAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  world->addSkeleton(robot);
  world->compile();
  ASSERT_FALSE(shape->has<VisualAspect>());

  // Clearing an absent aspect replaces nothing, through every set() overload.
  EXPECT_NO_THROW(body->set<UnusedAspect>(nullptr));
  EXPECT_NO_THROW(body->set<UnusedAspect>(std::unique_ptr<UnusedAspect>()));
  EXPECT_NO_THROW(shape->setVisualAspect(nullptr));
  EXPECT_NO_THROW(shape->setVisualAspect(std::unique_ptr<VisualAspect>()));
  EXPECT_FALSE(body->has<UnusedAspect>());
  EXPECT_FALSE(shape->has<VisualAspect>());

  // Adding or removing one still needs World::edit(); a rejected aspect stays
  // with the caller.
  auto aspect = std::make_unique<UnusedAspect>();
  EXPECT_THROW(body->set<UnusedAspect>(std::move(aspect)), std::runtime_error);
  EXPECT_NE(aspect.get(), nullptr);
  EXPECT_FALSE(body->has<UnusedAspect>());
  EXPECT_THROW(shape->setCollisionAspect(nullptr), std::runtime_error);
  EXPECT_THROW(
      shape->setCollisionAspect(std::unique_ptr<CollisionAspect>()),
      std::runtime_error);
  EXPECT_TRUE(shape->has<CollisionAspect>());
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
TEST(World, EditRecompilesChangedDofLayoutAndKeepsObjects)
{
  auto world = World::create();
  auto robot = Skeleton::create("robot");
  auto* root = robot->createJointAndBodyNodePair<RevoluteJoint>().second;
  world->addSkeleton(robot);
  world->compile();
  const auto revision = world->getTopologyRevision();
  {
    auto edit = world->edit();
    EXPECT_FALSE(world->isCompiled());
    EXPECT_THROW(world->step(), std::runtime_error);
    robot->createJointAndBodyNodePair<RevoluteJoint>(root);
    {
      auto nested = world->edit();
      EXPECT_FALSE(world->isCompiled());
    }
    EXPECT_FALSE(world->isCompiled());
  }
  EXPECT_TRUE(world->isCompiled());
  EXPECT_EQ(robot->getBodyNode(0), root);
  EXPECT_EQ(world->getIndex(1), 2);
  EXPECT_GT(world->getTopologyRevision(), revision);
  EXPECT_NO_THROW(world->step());
  EXPECT_TRUE(robot->getPositions().allFinite());
}

//==============================================================================
TEST(World, FrozenMoveAndCopyPreserveSourceAndDestination)
{
  auto frozenWorld = World::create();
  auto frozen = Skeleton::create("frozen");
  auto* frozenBody = frozen->createJointAndBodyNodePair<RevoluteJoint>().second;
  frozenWorld->addSkeleton(frozen);
  frozenWorld->compile();
  auto loose = Skeleton::create("loose");
  auto* looseBody = loose->createJointAndBodyNodePair<RevoluteJoint>().second;
  EXPECT_THROW(looseBody->moveTo(frozen, frozenBody), std::runtime_error);
  EXPECT_THROW(
      looseBody->moveTo<RevoluteJoint>(frozen, frozenBody), std::runtime_error);
  EXPECT_THROW(frozenBody->moveTo(loose, looseBody), std::runtime_error);
  EXPECT_THROW(looseBody->copyTo(frozen, frozenBody), std::runtime_error);
  EXPECT_THROW(
      looseBody->copyTo<RevoluteJoint>(frozen, frozenBody), std::runtime_error);
  EXPECT_EQ(looseBody->getSkeleton(), loose);
  EXPECT_EQ(frozenBody->getSkeleton(), frozen);
  EXPECT_EQ(loose->getNumBodyNodes(), 1u);
  EXPECT_EQ(frozen->getNumBodyNodes(), 1u);
  EXPECT_NO_THROW(frozenBody->copyTo(loose, looseBody));
  EXPECT_EQ(loose->getNumBodyNodes(), 2u);
}

namespace {

// A Frame-bearing Node type that World knows nothing about.
struct EmptyNodeData
{
};

class CustomJacobianNode final : public FixedJacobianNode
{
  using Empty = EmptyNodeData;

public:
  CustomJacobianNode(BodyNode* parent, const Eigen::Isometry3d& transform)
    : Entity(ConstructFrame),
      Frame(parent),
      FixedFrame(parent, transform),
      FixedJacobianNode(parent, transform)
  {
  }

  // Skeleton cloning requires every Node to hold a (here empty) State and
  // Properties.
  std::unique_ptr<Node::State> getNodeState() const override
  {
    return std::make_unique<Node::MakeState<Empty>>();
  }

  std::unique_ptr<Node::Properties> getNodeProperties() const override
  {
    return std::make_unique<Node::MakeProperties<Empty>>();
  }

protected:
  Node* cloneNode(BodyNode* parent) const override
  {
    return new CustomJacobianNode(parent, getRelativeTransform());
  }
};

} // namespace

//==============================================================================
TEST(World, CompiledCloneRemapsCustomJacobianFrames)
{
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(0.1, -0.2, 0.3);
  auto world = World::create();
  auto robot = Skeleton::create("robot");
  auto* first = robot->createJointAndBodyNodePair<FreeJoint>().second;
  first->setName("first_body");
  auto* second = robot->createJointAndBodyNodePair<FreeJoint>().second;
  second->setName("second_body");
  // Mixed node types in a creation order that differs from the body order.
  auto* secondCustom = second->createNode<CustomJacobianNode>(offset);
  second->createEndEffector("second_tip");
  auto* firstCustom = first->createNode<CustomJacobianNode>(offset);
  auto* firstOther = first->createNode<CustomJacobianNode>(offset);
  first->createEndEffector("first_tip");
  world->addSkeleton(robot);
  std::vector<SimpleFramePtr> frames;
  for (auto* parent :
       std::vector<Frame*>{secondCustom, firstCustom, firstOther})
    frames.push_back(std::make_shared<SimpleFrame>(
        parent, "on_" + std::to_string(frames.size()), offset));
  for (const auto& frame : frames) {
    frame->setRelativeSpatialVelocity(Eigen::Vector6d::LinSpaced(0.1, 0.6));
    frame->setRelativeSpatialAcceleration(Eigen::Vector6d::LinSpaced(0.7, 1.2));
    world->addSimpleFrame(frame);
  }
  world->compile();
  const auto sourceChildren = firstCustom->getChildFrames();

  WorldPtr clone;
  ASSERT_NO_THROW(clone = world->clone());
  const auto clonedRobot = clone->getSkeleton(0);
  const std::vector<const Node*> expected{
      clonedRobot->getBodyNode("second_body")->getNodes().front(),
      clonedRobot->getBodyNode("first_body")->getNodes().front(),
      clonedRobot->getBodyNode("first_body")->getNodes()[1]};
  for (std::size_t i = 0; i < frames.size(); ++i) {
    SCOPED_TRACE(i);
    const auto copy = clone->getSimpleFrame(frames[i]->getName());
    ASSERT_NE(copy, nullptr);
    EXPECT_EQ(copy->getParentFrame(), dynamic_cast<const Frame*>(expected[i]));
    EXPECT_NE(copy->getParentFrame(), frames[i]->getParentFrame());
    EXPECT_TRUE(
        copy->getWorldTransform().isApprox(frames[i]->getWorldTransform()));
    EXPECT_TRUE(copy->getRelativeSpatialVelocity().isApprox(
        frames[i]->getRelativeSpatialVelocity()));
    EXPECT_TRUE(copy->getRelativeSpatialAcceleration().isApprox(
        frames[i]->getRelativeSpatialAcceleration()));
  }
  EXPECT_EQ(firstCustom->getChildFrames(), sourceChildren);
  clone.reset();
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
TEST(World, CompiledCloneRejectsDetachedFrameBeforeKinematicRead)
{
  for (const bool shapeless : {true, false}) {
    SCOPED_TRACE(shapeless);
    auto world = World::create();
    auto robot = createBox(Eigen::Vector3d::Ones());
    world->addSkeleton(robot);
    auto detached = std::make_shared<SimpleFrame>(nullptr, "detached");
    if (!shapeless) {
      auto frame = std::make_shared<SimpleFrame>(Frame::World(), "detached");
      frame->setParentFrame(nullptr);
      detached = frame;
    }
    world->addSimpleFrame(detached);
    world->compile();
    const auto topology = world->getTopologyRevision();
    const auto parameter = world->getParameterRevision();
    EXPECT_THROW(world->clone(), std::runtime_error);
    {
      auto edit = world->edit();
      EXPECT_THROW(world->clone(), std::runtime_error);
    }
    EXPECT_EQ(detached->getParentFrame(), nullptr);
    EXPECT_EQ(world->getNumSimpleFrames(), 1u);
    EXPECT_GE(world->getTopologyRevision(), topology);
    EXPECT_GE(world->getParameterRevision(), parameter);
    EXPECT_NO_THROW(world->step());
  }
}

//==============================================================================
TEST(World, CompiledCloneRejectsDetachedExternalAncestorBeforeKinematicRead)
{
  // The owned frame's parent is non-null but is itself detached, so reading
  // the owned frame's World pose would walk into a null ancestor.
  auto ancestor = std::make_shared<SimpleFrame>(nullptr, "external_detached");
  auto owned = std::make_shared<SimpleFrame>(ancestor.get(), "owned_child");
  auto world = World::create();
  world->addSimpleFrame(owned);
  world->compile();
  EXPECT_THROW(world->clone(), std::runtime_error);
  {
    auto edit = world->edit();
    EXPECT_THROW(world->clone(), std::runtime_error);
  }
  EXPECT_EQ(owned->getParentFrame(), ancestor.get());
  EXPECT_EQ(ancestor->getParentFrame(), nullptr);
  EXPECT_EQ(world->getNumSimpleFrames(), 1u);
}

//==============================================================================
TEST(World, CloneOfCompiledWorldAttachesFramesToClonedParents)
{
  auto outside = std::make_shared<SimpleFrame>(Frame::World(), "outside");
  auto world = World::create();
  auto robot = Skeleton::create("robot");
  auto* body = robot->createJointAndBodyNodePair<FreeJoint>().second;
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(0.1, -0.2, 0.3);
  offset.linear() = eulerXYZToMatrix(Eigen::Vector3d(0.3, 0.2, 0.1));
  robot->getJoint(0)->setPositions(FreeJoint::convertToPositions(offset));
  body->setName("first_body");
  auto* secondBody = robot->createJointAndBodyNodePair<FreeJoint>().second;
  secondBody->setName("second_body");
  // Node creation order deliberately differs from body order.
  auto* secondTip = secondBody->createEndEffector("second_tip");
  auto* secondShape = secondBody->createShapeNodeWith<CollisionAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  secondShape->setName("second_shape");
  auto* tip = body->createEndEffector("tip");
  auto* firstShape = body->createShapeNodeWith<CollisionAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  firstShape->setName("first_shape");
  tip->setDefaultRelativeTransform(offset, true);
  world->addSkeleton(robot);
  auto onBody = std::make_shared<SimpleFrame>(body, "on_body", offset);
  onBody->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  auto onTip = std::make_shared<SimpleFrame>(tip, "on_tip", offset);
  auto parent = std::make_shared<SimpleFrame>(Frame::World(), "parent", offset);
  auto child = std::make_shared<SimpleFrame>(parent.get(), "child", offset);
  // The child precedes its parent.
  const std::vector<SimpleFramePtr> frames{
      child,
      onBody,
      onTip,
      parent,
      std::make_shared<SimpleFrame>(secondBody, "on_second_body", offset),
      std::make_shared<SimpleFrame>(secondTip, "on_second_tip", offset),
      std::make_shared<SimpleFrame>(firstShape, "on_first_shape", offset),
      std::make_shared<SimpleFrame>(secondShape, "on_second_shape", offset)};
  for (const auto& frame : frames) {
    frame->setRelativeSpatialVelocity(Eigen::Vector6d::LinSpaced(0.1, 0.6));
    frame->setRelativeSpatialAcceleration(Eigen::Vector6d::LinSpaced(0.7, 1.2));
  }
  for (const auto& frame : frames)
    world->addSimpleFrame(frame);
  {
    // Lazy Worlds keep the DART 6.20 clone, on the source Skeleton's frames.
    const auto lazy = world->clone();
    EXPECT_EQ(lazy->getSimpleFrame("on_body")->getParentFrame(), body);
    EXPECT_EQ(lazy->getSimpleFrame("on_tip")->getParentFrame(), tip);
    EXPECT_EQ(
        lazy->getSimpleFrame("child")->getParentFrame(),
        lazy->getSimpleFrame("parent").get());
  }

  world->compile();
  const auto bodyChildren = body->getChildFrames();
  const auto tipChildren = tip->getChildFrames();
  const auto parentChildren = parent->getChildFrames();
  WorldPtr clone;
  ASSERT_NO_THROW(clone = world->clone());
  EXPECT_EQ(body->getChildFrames(), bodyChildren);
  EXPECT_EQ(tip->getChildFrames(), tipChildren);
  EXPECT_EQ(parent->getChildFrames(), parentChildren);
  const auto clonedRobot = clone->getSkeleton(0);
  EXPECT_EQ(
      clone->getSimpleFrame("on_body")->getParentFrame(),
      clonedRobot->getBodyNode(0));
  EXPECT_EQ(
      clone->getSimpleFrame("on_tip")->getParentFrame(),
      clonedRobot->getEndEffector("tip"));
  EXPECT_EQ(
      clone->getSimpleFrame("on_second_body")->getParentFrame(),
      clonedRobot->getBodyNode("second_body"));
  EXPECT_EQ(
      clone->getSimpleFrame("on_second_tip")->getParentFrame(),
      clonedRobot->getEndEffector("second_tip"));
  EXPECT_EQ(
      clone->getSimpleFrame("on_first_shape")->getParentFrame(),
      clonedRobot->getShapeNode("first_shape"));
  EXPECT_EQ(
      clone->getSimpleFrame("on_second_shape")->getParentFrame(),
      clonedRobot->getShapeNode("second_shape"));
  EXPECT_EQ(
      clone->getSimpleFrame("child")->getParentFrame(),
      clone->getSimpleFrame("parent").get());
  EXPECT_TRUE(clone->getSimpleFrame("parent")->getParentFrame()->isWorld());
  EXPECT_NE(clone->getSimpleFrame("on_body")->getShape(), nullptr);
  for (const auto& frame : frames) {
    SCOPED_TRACE(frame->getName());
    const auto copy = clone->getSimpleFrame(frame->getName());
    EXPECT_EQ(
        copy->getParentFrame()->getName(), frame->getParentFrame()->getName());
    EXPECT_TRUE((copy->getRelativeTransform().matrix().array()
                 == frame->getRelativeTransform().matrix().array())
                    .all());
    EXPECT_TRUE((copy->getRelativeSpatialVelocity().array()
                 == frame->getRelativeSpatialVelocity().array())
                    .all());
    EXPECT_TRUE((copy->getRelativeSpatialAcceleration().array()
                 == frame->getRelativeSpatialAcceleration().array())
                    .all());
    EXPECT_TRUE(clone->getSimpleFrame(frame->getName())
                    ->getWorldTransform()
                    .isApprox(frame->getWorldTransform()));
  }
  EXPECT_NO_THROW(world->step());
  EXPECT_NO_THROW(clone->step());

  // A parent the World does not hold has no clone. Sharing it inside edit()
  // would put the clone's frame in this World's model, so clone() throws at
  // any edit depth and leaves the parent alone.
  {
    auto edit = world->edit();
    world->addSimpleFrame(std::make_shared<SimpleFrame>(outside.get()));
  }
  ASSERT_TRUE(world->isCompiled());
  EXPECT_THROW(world->clone(), std::runtime_error);
  WorldPtr edited;
  {
    auto edit = world->edit();
    EXPECT_THROW(edited = world->clone(), std::runtime_error);
  }
  EXPECT_EQ(outside->getChildFrames().size(), 1u);
  // Dropping a clone must not change this World's model.
  edited.reset();
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
TEST(World, LazySharedMembershipAndOwnerLifetimeRemainSupported)
{
  auto robot = Skeleton::create("shared");
  auto* body = robot->createJointAndBodyNodePair<RevoluteJoint>().second;
  auto first = World::create();
  auto second = World::create();
  first->addSkeleton(robot);
  second->addSkeleton(robot);
  // Lazy Worlds have no registry entry, so a lazy sharer goes undetected and
  // the model freezes for it too.
  EXPECT_NO_THROW(first->compile());
  EXPECT_TRUE(first->isCompiled());
  EXPECT_THROW(
      robot->createJointAndBodyNodePair<RevoluteJoint>(body),
      std::runtime_error);
  EXPECT_NO_THROW(first->step());
  EXPECT_NO_THROW(second->step());
  second->removeSkeleton(robot);
  EXPECT_THROW(second->addSkeleton(robot), std::runtime_error);
  EXPECT_FALSE(second->hasSkeleton(robot));
  first.reset();
  EXPECT_NO_THROW(robot->createJointAndBodyNodePair<RevoluteJoint>(body));
  EXPECT_NO_THROW(second->addSkeleton(robot));
}

//==============================================================================
TEST(World, LazyWorldSettersKeepVersionsAndSkipTheRegistry)
{
  if (dart::simulation::detail::getIndexedWorldCount() != 0u)
    GTEST_SKIP() << "an earlier test keeps a compiled World alive";
  const auto locks = dart::simulation::detail::getRegistryLockCount();
  auto world = World::create();
  auto robot = createBox(Eigen::Vector3d::Ones());
  world->addSkeleton(robot);
  world->addSimpleFrame(std::make_shared<SimpleFrame>());
  auto* body = robot->getBodyNode(0);
  auto* joint = robot->getJoint(0);
  auto* shape = body->getShapeNodeWith<CollisionAspect>(0);
  ASSERT_NE(shape->getVisualAspect(), nullptr);
  ASSERT_NE(shape->getDynamicsAspect(), nullptr);
  world->step();

  struct Versions
  {
    std::size_t shape, body, joint, skeleton, structural;
  };
  const auto read = [&] {
    return Versions{
        shape->getVersion(),
        body->getVersion(),
        joint->getVersion(),
        robot->getVersion(),
        SkeletonVersionAccess::getGlobalStructuralVersion()};
  };
  const auto expectDelta = [&](const Versions& before,
                               const Versions& after,
                               std::size_t shapeDelta,
                               std::size_t bodyDelta,
                               std::size_t jointDelta) {
    EXPECT_EQ(after.shape, before.shape + shapeDelta);
    EXPECT_EQ(after.body, before.body + bodyDelta);
    EXPECT_EQ(after.joint, before.joint + jointDelta);
    const std::size_t skeletonDelta
        = std::max({shapeDelta, bodyDelta, jointDelta});
    EXPECT_EQ(after.skeleton, before.skeleton + skeletonDelta);
    EXPECT_EQ(after.structural, before.structural + skeletonDelta);
  };

  // gz-sim's WheelSlip setter runs every step and bumps no version (6.20).
  auto before = read();
  shape->getDynamicsAspect()->setPrimarySlipCompliance(1e-5);
  expectDelta(before, read(), 0u, 0u, 0u);
  // Cosmetic and parameter setters propagate BodyNode -> Skeleton -> global.
  before = read();
  body->setName("renamed");
  expectDelta(before, read(), 0u, 1u, 0u);
  before = read();
  shape->getVisualAspect()->setColor(Eigen::Vector3d(1.0, 0.0, 0.0));
  expectDelta(before, read(), 1u, 1u, 0u);
  before = read();
  body->setMass(2.0 * body->getMass());
  expectDelta(before, read(), 0u, 1u, 0u);
  before = read();
  body->setMomentOfInertia(2.0, 3.0, 4.0);
  expectDelta(before, read(), 0u, 1u, 0u);
  before = read();
  joint->setPositionUpperLimit(0, 1.5);
  expectDelta(before, read(), 0u, 0u, 1u);
  before = read();
  joint->setDampingCoefficient(0, 0.25);
  expectDelta(before, read(), 0u, 0u, 1u);
  // 6.20 moves no version for these: limit enforcement only restamps the
  // automatic constraints, and the joint offsets only the kinematic version.
  before = read();
  joint->setLimitEnforcement(!joint->areLimitsEnforced());
  expectDelta(before, read(), 0u, 0u, 0u);
  before = read();
  joint->setTransformFromParentBodyNode(
      Eigen::Isometry3d(Eigen::Translation3d(0.1, 0.0, 0.0)));
  expectDelta(before, read(), 0u, 0u, 0u);
  before = read();
  joint->setTransformFromChildBodyNode(
      Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.1, 0.0)));
  expectDelta(before, read(), 0u, 0u, 0u);

  // Topology edits, membership, stepping and destruction of lazy Worlds never
  // touch the ownership registry.
  auto* child = robot->createJointAndBodyNodePair<RevoluteJoint>(body).second;
  child->remove();
  world->removeSimpleFrame(world->getSimpleFrame(0));
  world->step();
  EXPECT_EQ(world->getTopologyRevision(), world->getTopologyRevision());
  world.reset();
  EXPECT_EQ(dart::simulation::detail::getRegistryLockCount(), locks);
  EXPECT_EQ(dart::simulation::detail::getIndexedWorldCount(), 0u);
}

//==============================================================================
TEST(World, LazyWorldsOnTwoThreadsNeverTouchTheRegistry)
{
  if (dart::simulation::detail::getIndexedWorldCount() != 0u)
    GTEST_SKIP() << "an earlier test keeps a compiled World alive";
  const auto locks = dart::simulation::detail::getRegistryLockCount();
  // BodyNode construction bumps a process-wide, non-atomic id counter, so the
  // lanes create links one at a time; everything else runs concurrently.
  std::mutex creation;
  std::exception_ptr errors[2];
  std::thread threads[2];
  for (int lane = 0; lane < 2; ++lane) {
    threads[lane] = std::thread([&, lane] {
      try {
        auto world = World::create();
        SkeletonPtr ground;
        SkeletonPtr robot;
        {
          std::lock_guard<std::mutex> lock(creation);
          ground = createBox(Eigen::Vector3d(10.0, 10.0, 0.2));
          robot = createBox(
              Eigen::Vector3d::Ones(), Eigen::Vector3d(0.0, 0.0, 0.6));
        }
        ground->setMobile(false);
        world->addSkeleton(ground);
        world->addSkeleton(robot);
        auto* body = robot->getBodyNode(0);
        auto* dynamics
            = body->getShapeNodeWith<CollisionAspect>(0)->getDynamicsAspect();
        for (int i = 0; i < 200; ++i) {
          dynamics->setPrimarySlipCompliance(1e-6 * ((i + lane) % 3));
          body->setMass(1.0 + 0.1 * (i % 5));
          body->setName(i % 2 ? "even" : "odd");
          auto frame = std::make_shared<SimpleFrame>(body, "attached");
          world->addSimpleFrame(frame);
          BodyNode* child = nullptr;
          {
            std::lock_guard<std::mutex> lock(creation);
            child
                = robot->createJointAndBodyNodePair<RevoluteJoint>(body).second;
          }
          world->step();
          child->remove();
          world->removeSimpleFrame(frame);
        }
      } catch (...) {
        errors[lane] = std::current_exception();
      }
    });
  }
  for (auto& thread : threads)
    thread.join();
  for (auto& error : errors) {
    if (error)
      std::rethrow_exception(error);
  }
  EXPECT_EQ(dart::simulation::detail::getRegistryLockCount(), locks);
}

//==============================================================================
TEST(World, TransferredRootBodyRechainsAttachedFrames)
{
  auto world = World::create();
  auto source = Skeleton::create("source");
  auto* body = source->createJointAndBodyNodePair<FreeJoint>().second;
  auto target = Skeleton::create("target");
  auto frame = std::make_shared<SimpleFrame>(body, "attached");
  world->addSimpleFrame(frame);
  world->compile();
  EXPECT_THROW(
      source->createJointAndBodyNodePair<RevoluteJoint>(body),
      std::runtime_error);
  {
    auto edit = world->edit();
    // Root to root: no parent Frame changes, only the Skeleton does.
    ASSERT_TRUE(body->moveTo(target, nullptr));
    ASSERT_EQ(body->getSkeleton(), target);
    auto parameter = world->getParameterRevision();
    source->setGravity(Eigen::Vector3d(0.0, 0.0, -1.0));
    EXPECT_EQ(world->getParameterRevision(), parameter);
    target->setGravity(Eigen::Vector3d(0.0, 0.0, -2.0));
    EXPECT_GT(world->getParameterRevision(), parameter);
    parameter = world->getParameterRevision();
    frame->setRelativeTranslation(Eigen::Vector3d::UnitX());
    EXPECT_GT(world->getParameterRevision(), parameter);
  }
  EXPECT_NO_THROW(source->createJointAndBodyNodePair<RevoluteJoint>());
  EXPECT_THROW(
      target->createJointAndBodyNodePair<RevoluteJoint>(body),
      std::runtime_error);
}

namespace {

class ThrowingGroupDetector final : public collision::FCLCollisionDetector
{
public:
  ThrowingGroupDetector() = default;

  std::unique_ptr<collision::CollisionGroup> createCollisionGroup() override
  {
    throw std::runtime_error("collision group factory failure");
  }
};

} // namespace

//==============================================================================
TEST(World, ThrowingDetectorGroupFactoryLeavesSolverAndIndexUnchanged)
{
  auto world = World::create();
  world->addSkeleton(createBox(Eigen::Vector3d::Ones()));
  world->compile();
  auto* solver = world->getConstraintSolver();
  const auto detector = solver->getCollisionDetector();
  const auto group = solver->getCollisionGroup();
  auto fcl
      = std::dynamic_pointer_cast<collision::FCLCollisionDetector>(detector);
  ASSERT_NE(fcl, nullptr);
  {
    auto edit = world->edit();
    EXPECT_THROW(
        solver->setCollisionDetector(std::make_shared<ThrowingGroupDetector>()),
        std::runtime_error);
    EXPECT_EQ(solver->getCollisionDetector(), detector);
    EXPECT_EQ(solver->getCollisionGroup(), group);
    // The retained detector is still indexed.
    const auto topology = world->getTopologyRevision();
    fcl->setPrimitiveShapeType(
        fcl->getPrimitiveShapeType() == collision::FCLCollisionDetector::MESH
            ? collision::FCLCollisionDetector::PRIMITIVE
            : collision::FCLCollisionDetector::MESH);
    EXPECT_GT(world->getTopologyRevision(), topology);
  }
  EXPECT_TRUE(world->isCompiled());
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
// Shared handler re-parenting must not read another owner's solver under the
// registry mutex while that owner edits its constraints (run under TSan).
TEST(World, SharedHandlerReparentDoesNotReadOtherOwnersSolver)
{
  using dart::constraint::ContactSurfaceHandler;
  auto first = World::create();
  auto robot = createBox(Eigen::Vector3d::Ones());
  first->addSkeleton(robot);
  first->compile();
  auto second = World::create();
  second->compile();
  auto shared = std::make_shared<ContactSurfaceHandler>();
  auto firstParent = std::make_shared<ContactSurfaceHandler>();
  auto secondParent = std::make_shared<ContactSurfaceHandler>();
  {
    auto firstEdit = first->edit();
    auto secondEdit = second->edit();
    first->getConstraintSolver()->addContactSurfaceHandler(shared);
    second->getConstraintSolver()->addContactSurfaceHandler(shared);
    std::exception_ptr errors[2];
    std::thread editor([&] {
      try {
        auto* solver = first->getConstraintSolver();
        for (int i = 0; i < 300; ++i) {
          auto constraint
              = std::make_shared<dart::constraint::WeldJointConstraint>(
                  robot->getBodyNode(0));
          solver->addConstraint(constraint);
          solver->removeConstraint(constraint);
        }
      } catch (...) {
        errors[0] = std::current_exception();
      }
    });
    std::thread reparent([&] {
      try {
        for (int i = 0; i < 300; ++i) {
          shared->setParent(firstParent);
          shared->setParent(secondParent);
        }
      } catch (...) {
        errors[1] = std::current_exception();
      }
    });
    editor.join();
    reparent.join();
    for (auto& error : errors) {
      if (error)
        std::rethrow_exception(error);
    }
    EXPECT_EQ(shared->getParent(), secondParent);
    // Leave only the spliced parent in the first World's chain.
    EXPECT_TRUE(
        first->getConstraintSolver()->removeContactSurfaceHandler(shared));
    EXPECT_TRUE(
        second->getConstraintSolver()->removeContactSurfaceHandler(shared));
    EXPECT_TRUE(second->getConstraintSolver()->removeContactSurfaceHandler(
        secondParent));
  }
  EXPECT_TRUE(first->isCompiled());
  EXPECT_TRUE(second->isCompiled());
  EXPECT_EQ(
      first->getConstraintSolver()->getLastContactSurfaceHandler(),
      secondParent);
  // The spliced chain froze the current parent, not the stale one.
  EXPECT_THROW(secondParent->setParent(firstParent), std::runtime_error);
  EXPECT_NO_THROW(firstParent->setParent(shared));
}

//==============================================================================
TEST(World, CompiledRevisionsSeparateParametersStateAndAppearance)
{
  auto world = World::create();
  auto robot = createBox(Eigen::Vector3d::Ones());
  world->addSkeleton(robot);
  world->compile();
  auto* body = robot->getBodyNode(0);
  auto* shape = body->getShapeNodeWith<VisualAspect>(0);
  const auto topology = world->getTopologyRevision();
  const auto parameter = world->getParameterRevision();
  const auto structural = SkeletonVersionAccess::getGlobalStructuralVersion();
  body->setMass(body->getMass() * 2.0);
  body->setLocalCOM(Eigen::Vector3d(0.01, 0, 0));
  robot->getJoint(0)->setDampingCoefficient(0, 0.5);
  auto pose = shape->getRelativeTransform();
  pose.translation().x() = 0.05;
  shape->setRelativeTransform(pose);
  auto box = std::dynamic_pointer_cast<BoxShape>(shape->getShape());
  ASSERT_NE(box, nullptr);
  box->setSize(1.05 * box->getSize());
  shape->getCollisionAspect()->setCollidable(false);
  shape->getDynamicsAspect()->setFrictionCoeff(0.7);
  world->setTimeStep(0.002);
  world->setGravity(Eigen::Vector3d(0, 0, -9.8));
  // Versions keep their DART 6.20 propagation; only the revisions classify.
  EXPECT_GT(SkeletonVersionAccess::getGlobalStructuralVersion(), structural);
  EXPECT_GT(world->getParameterRevision(), parameter);
  EXPECT_EQ(world->getTopologyRevision(), topology);
  const auto changedParameters = world->getParameterRevision();
  const auto changedStructural
      = SkeletonVersionAccess::getGlobalStructuralVersion();
  robot->setPosition(0, 0.1);
  robot->setVelocity(0, 0.2);
  robot->setCommand(0, 0.3);
  body->setName("renamed");
  shape->getVisualAspect()->setColor(Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_GT(
      SkeletonVersionAccess::getGlobalStructuralVersion(), changedStructural);
  EXPECT_EQ(world->getParameterRevision(), changedParameters);
  EXPECT_EQ(world->getTopologyRevision(), topology);
  EXPECT_NO_THROW(world->step());
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
TEST(World, SupportGeometryStampsParameterButActivityIsState)
{
  auto world = World::create();
  auto robot = createBox(Eigen::Vector3d::Ones());
  auto* tip = robot->getBodyNode(0)->createEndEffector("tip");
  auto* support = tip->createSupport();
  support->setActive(true);
  support->setGeometry(math::SupportGeometry{Eigen::Vector3d::Zero()});
  world->addSkeleton(robot);
  world->compile();
  const auto topology = world->getTopologyRevision();
  const auto parameter = world->getParameterRevision();

  support->setActive(false);
  support->setActive(true);
  EXPECT_EQ(world->getParameterRevision(), parameter);

  support->setGeometry(math::SupportGeometry{Eigen::Vector3d(0.5, 0.0, 0.0)});
  EXPECT_GT(world->getParameterRevision(), parameter);
  EXPECT_EQ(world->getTopologyRevision(), topology);
  ASSERT_EQ(support->getGeometry().size(), 1u);
  EXPECT_DOUBLE_EQ(support->getGeometry()[0].x(), 0.5);
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
TEST(World, CompiledChartAndPlaneSettersAdvanceParameterRevision)
{
  auto world = World::create();
  auto robot = Skeleton::create("robot");
  auto root = robot->createJointAndBodyNodePair<FreeJoint>();
  auto* freeJoint = root.first;
  auto* planar
      = robot->createJointAndBodyNodePair<PlanarJoint>(root.second).first;
  auto* planar2d = robot
                       ->createJointAndBodyNodePair<TranslationalJoint2D>(
                           planar->getChildBodyNode())
                       .first;
  world->addSkeleton(robot);
  world->compile();
  const auto topology = world->getTopologyRevision();
  // Joint versions keep DART 6.20: a chart change bumps one, a plane none.
  const auto expectStamp = [&](const char* setter,
                               const Joint* joint,
                               std::size_t versionDelta,
                               const std::function<void()>& set) {
    SCOPED_TRACE(setter);
    const auto parameters = world->getParameterRevision();
    const auto version = joint->getVersion();
    set();
    EXPECT_GT(world->getParameterRevision(), parameters);
    EXPECT_EQ(joint->getVersion(), version + versionDelta);
  };
  expectStamp("FreeJoint::setCoordinateChart", freeJoint, 1u, [&] {
    freeJoint->setCoordinateChart(FreeJoint::CoordinateChart::EULER_ZYX);
  });
  const Eigen::Vector3d axis1 = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d axis2(0.0, 1.0, 1.0);
  expectStamp(
      "PlanarJoint::setYZPlane", planar, 0u, [&] { planar->setYZPlane(); });
  expectStamp(
      "PlanarJoint::setZXPlane", planar, 0u, [&] { planar->setZXPlane(); });
  expectStamp("PlanarJoint::setArbitraryPlane", planar, 0u, [&] {
    planar->setArbitraryPlane(axis1, axis2);
  });
  expectStamp(
      "PlanarJoint::setXYPlane", planar, 0u, [&] { planar->setXYPlane(); });
  expectStamp("TranslationalJoint2D::setYZPlane", planar2d, 0u, [&] {
    planar2d->setYZPlane();
  });
  expectStamp("TranslationalJoint2D::setZXPlane", planar2d, 0u, [&] {
    planar2d->setZXPlane();
  });
  expectStamp("TranslationalJoint2D::setArbitraryPlane", planar2d, 0u, [&] {
    planar2d->setArbitraryPlane(axis1, axis2);
  });
  expectStamp("TranslationalJoint2D::setXYPlane", planar2d, 0u, [&] {
    planar2d->setXYPlane();
  });
  EXPECT_EQ(world->getTopologyRevision(), topology);
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
TEST(World, JointParametersPreserveQualifiedVersionIncrement)
{
  struct VersionOverrideJoint : FreeJoint
  {
    explicit VersionOverrideJoint(const Properties& properties)
      : FreeJoint(properties)
    {
    }

    std::size_t incrementVersion() override
    {
      ++calls;
      return FreeJoint::incrementVersion();
    }

    std::size_t calls = 0u;
  };
  for (const bool compiled : {false, true}) {
    auto world = World::create();
    auto robot = Skeleton::create();
    auto* joint
        = robot->createJointAndBodyNodePair<VersionOverrideJoint>().first;
    world->addSkeleton(robot);
    if (compiled)
      world->compile();
    const auto calls = joint->calls;
    const auto version = joint->getVersion();
    const auto parameters = world->getParameterRevision();
    joint->setCoordinateChart(FreeJoint::CoordinateChart::EULER_ZYX);
    EXPECT_EQ(joint->calls, calls);
    EXPECT_EQ(joint->getVersion(), version + 1u);
    EXPECT_EQ(world->getParameterRevision(), parameters + (compiled ? 1u : 0u));
    // GenericJoint parameter setters used the same qualified base call.
    const auto limitVersion = joint->getVersion();
    const auto limitParameters = world->getParameterRevision();
    joint->setPositionLowerLimit(0, -0.5);
    EXPECT_EQ(joint->calls, calls);
    EXPECT_EQ(joint->getVersion(), limitVersion + 1u);
    if (compiled) {
      EXPECT_GT(world->getParameterRevision(), limitParameters);
    } else {
      EXPECT_EQ(world->getParameterRevision(), limitParameters);
    }
  }
}

//==============================================================================
TEST(World, CompileRejectsRawConfigurationChangesUntilEdited)
{
  auto world = createCloneContactWorld();
  world->compile();
  auto& options = world->getConstraintSolver()->getCollisionOption();
  options.maxNumContacts += 1;
  const auto time = world->getTime();
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_THROW(world->compile(), std::runtime_error);
  EXPECT_EQ(world->getTime(), time);
  {
    auto edit = world->edit();
  }
  EXPECT_NO_THROW(world->step());
  auto frame = std::make_shared<SimpleFrame>();
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  world->getConstraintSolver()->getCollisionGroup()->addShapeFrame(frame.get());
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_THROW(world->compile(), std::runtime_error);
  {
    auto edit = world->edit();
  }
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
TEST(World, ExplicitCompileRejectsUnsupportedNativeShapeTransactionally)
{
  auto world = World::create();
  world->setCollisionDetector(CollisionDetectorType::Dart);
  auto robot = createBox(Eigen::Vector3d::Ones());
  auto* shape = robot->getBodyNode(0)->getShapeNodeWith<CollisionAspect>(0);
  shape->setShape(std::make_shared<LineSegmentShape>());
  world->addSkeleton(robot);
  EXPECT_THROW(world->compile(), std::runtime_error);
  EXPECT_FALSE(world->isCompiled());
  EXPECT_NO_THROW(
      shape->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones())));
  EXPECT_NO_THROW(world->compile());
  EXPECT_TRUE(world->isCompiled());
}

//==============================================================================
TEST(World, ExplicitCompileAdmitsSpecializedSoftMeshRoute)
{
  auto world = World::create();
  world->setCollisionDetector(CollisionDetectorType::Dart);
  auto soft = Skeleton::create("soft");
  const auto properties = SoftBodyNodeHelper::makeBoxProperties(
      Eigen::Vector3d::Ones(),
      Eigen::Isometry3d::Identity(),
      Eigen::Vector3i(2, 2, 2),
      1.0,
      500.0,
      10.0,
      1.0);
  soft->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>(
      nullptr,
      FreeJoint::Properties(),
      SoftBodyNode::Properties(BodyNode::Properties(), properties));
  world->addSkeleton(soft);
  EXPECT_NO_THROW(world->compile());
  EXPECT_TRUE(world->isCompiled());
  auto* body = soft->getSoftBodyNode(0);
  const auto points = body->getNumPointMasses();
  const auto faces = body->getNumFaces();
  auto* point = body->getPointMass(0);
  const auto connections = point->getNumConnectedPointMasses();
  EXPECT_THROW(body->removeAllPointMasses(), std::runtime_error);
  EXPECT_THROW(body->addPointMass(PointMass::Properties()), std::runtime_error);
  EXPECT_THROW(body->connectPointMasses(0, 1), std::runtime_error);
  EXPECT_THROW(
      point->addConnectedPointMass(body->getPointMass(1)), std::runtime_error);
  EXPECT_THROW(body->addFace(Eigen::Vector3i(0, 1, 2)), std::runtime_error);
  auto changed = body->getAspectProperties();
  changed.mPointProps.push_back(PointMass::Properties());
  EXPECT_THROW(body->setAspectProperties(changed), std::runtime_error);
  EXPECT_EQ(body->getNumPointMasses(), points);
  EXPECT_EQ(body->getNumFaces(), faces);
  EXPECT_EQ(point->getNumConnectedPointMasses(), connections);
  const auto topology = world->getTopologyRevision();
  const auto parameter = world->getParameterRevision();
  const auto structural = SkeletonVersionAccess::getGlobalStructuralVersion();
  const auto previousPsi = point->getImplicitPsi();
  body->setVertexSpringStiffness(1000.0);
  body->setDampingCoefficient(2.0);
  EXPECT_NE(point->getImplicitPsi(), previousPsi);
  EXPECT_GT(SkeletonVersionAccess::getGlobalStructuralVersion(), structural);
  EXPECT_EQ(world->getTopologyRevision(), topology);
  EXPECT_GT(world->getParameterRevision(), parameter);
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
TEST(World, CompiledWorldDetectsEmptySubscriptionsAndOwnsTheirModels)
{
  auto world = World::create();
  auto empty = Skeleton::create("empty");
  world->compile();
  const auto group = world->getConstraintSolver()->getCollisionGroup();
  group->subscribeTo(empty);
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_THROW(world->compile(), std::runtime_error);
  {
    auto edit = world->edit();
  }
  EXPECT_THROW(
      empty->createJointAndBodyNodePair<RevoluteJoint>(), std::runtime_error);
  EXPECT_EQ(empty->getNumBodyNodes(), 0u);
  group->unsubscribeFrom(empty.get());
  EXPECT_THROW(world->step(), std::runtime_error);

  auto clearingWorld = World::create();
  auto clearingSource = Skeleton::create("cleared_empty");
  const auto clearingGroup
      = clearingWorld->getConstraintSolver()->getCollisionGroup();
  clearingGroup->subscribeTo(clearingSource);
  clearingWorld->compile();
  clearingGroup->removeAllShapeFrames();
  EXPECT_THROW(clearingWorld->step(), std::runtime_error);
  EXPECT_THROW(clearingWorld->compile(), std::runtime_error);
}

//==============================================================================
TEST(World, CompiledWorldDetectsNestedBuiltInFilterChanges)
{
  auto world = createCloneContactWorld();
  auto root = std::make_shared<collision::CompositeCollisionFilter>();
  auto nested = std::make_shared<collision::CompositeCollisionFilter>();
  auto child = std::make_shared<collision::BodyNodeCollisionFilter>();
  root->addCollisionFilter(nested.get());
  world->getConstraintSolver()->getCollisionOption().collisionFilter = root;
  world->compile();
  nested->addCollisionFilter(child.get());
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_THROW(world->compile(), std::runtime_error);
  {
    auto edit = world->edit();
  }
  child->addBodyNodePairToBlackList(
      world->getSkeleton(0)->getBodyNode(0),
      world->getSkeleton(1)->getBodyNode(0));
  EXPECT_THROW(world->step(), std::runtime_error);
}

//==============================================================================
TEST(World, CompiledWorldProtectsStandaloneFrameDescendants)
{
  auto world = World::create();
  auto root = std::make_shared<SimpleFrame>();
  auto child = root->spawnChildSimpleFrame("child");
  world->addSimpleFrame(root);
  world->compile();
  EXPECT_THROW(child->setParentFrame(Frame::World()), std::runtime_error);
  EXPECT_EQ(child->getParentFrame(), root.get());
  EXPECT_THROW(root->spawnChildSimpleFrame("forbidden"), std::runtime_error);
  const auto children = root->getChildFrames();
  EXPECT_THROW(
      static_cast<void>(std::make_shared<SimpleFrame>(root.get())),
      std::runtime_error);
  EXPECT_EQ(root->getChildFrames(), children);
  auto parameters = world->getParameterRevision();
  EXPECT_THROW(
      static_cast<void>(std::make_shared<NamedFixedFrame>(child.get())),
      std::runtime_error);
  EXPECT_EQ(world->getParameterRevision(), parameters);
  child.reset();
  EXPECT_THROW(world->step(), std::runtime_error);
}

//==============================================================================
TEST(World, FrozenJointPropertiesRejectBeforeAnyEarlierFieldChanges)
{
  auto world = World::create();
  auto robot = Skeleton::create("mixed");
  FreeJoint::Properties initial;
  initial.mActuatorTypes = {{0u, Joint::MIMIC}};
  Joint* joint
      = robot->createJointAndBodyNodePair<FreeJoint>(nullptr, initial).first;
  world->addSkeleton(robot);
  world->compile();
  const auto unchanged = joint->getJointProperties();
  EXPECT_NO_THROW(joint->setProperties(unchanged));
  auto changed = unchanged;
  changed.mName = "forbidden_name";
  changed.mT_ParentBodyToJoint.translation().x() = 0.5;
  changed.mActuatorType = Joint::PASSIVE;
  changed.mActuatorTypes.clear();
  EXPECT_THROW(joint->setProperties(changed), std::runtime_error);
  EXPECT_EQ(joint->getName(), unchanged.mName);
  EXPECT_EQ(
      joint->getTransformFromParentBodyNode().matrix(),
      unchanged.mT_ParentBodyToJoint.matrix());
  EXPECT_EQ(
      joint->getJointProperties().mActuatorTypes, unchanged.mActuatorTypes);
}

//==============================================================================
TEST(World, FrozenNodeAndSkeletonBatchesPreflightEveryEntry)
{
  auto world = World::create();
  auto robot = createBox(Eigen::Vector3d::Ones());
  auto* body = robot->getBodyNode(0);
  auto* first = body->getShapeNodeWith<CollisionAspect>(0);
  auto* second = body->createShapeNodeWith<CollisionAspect>(first->getShape());
  world->addSkeleton(robot);
  world->compile();
  auto changes = body->getAllNodeProperties();
  auto& values = changes.getMap().at(typeid(ShapeNode))->getVector();
  ASSERT_EQ(values.size(), 2u);
  auto* firstProperties
      = dynamic_cast<common::Composite::Properties*>(values[0].get());
  auto* secondProperties
      = dynamic_cast<common::Composite::Properties*>(values[1].get());
  ASSERT_NE(firstProperties, nullptr);
  ASSERT_NE(secondProperties, nullptr);
  ASSERT_NE(firstProperties->get<FixedFrame>(), nullptr);
  ASSERT_NE(secondProperties->get<ShapeFrame>(), nullptr);
  firstProperties->get<FixedFrame>()->mRelativeTf.translation().x() = 0.75;
  secondProperties->get<ShapeFrame>()->mShape
      = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  const auto originalTransform = first->getRelativeTransform();
  const auto originalShape = second->getShape();
  EXPECT_THROW(body->setAllNodeProperties(changes), std::runtime_error);
  EXPECT_EQ(first->getRelativeTransform().matrix(), originalTransform.matrix());
  EXPECT_EQ(second->getShape(), originalShape);

  auto properties = robot->getProperties();
  auto* bodyProxy
      = properties.get<dynamics::detail::BodyNodeVectorProxyAspect>();
  ASSERT_NE(bodyProxy, nullptr);
  auto bodies = bodyProxy->get();
  auto* nodeProxy = bodies[0].get<dynamics::detail::NodeVectorProxyAspect>();
  ASSERT_NE(nodeProxy, nullptr);
  nodeProxy->set(changes);
  bodyProxy->set(bodies);
  properties.get<Skeleton>()->mName = "forbidden_skeleton_name";
  const auto originalName = robot->getName();
  EXPECT_THROW(robot->setProperties(properties), std::runtime_error);
  EXPECT_EQ(robot->getName(), originalName);
  EXPECT_EQ(first->getRelativeTransform().matrix(), originalTransform.matrix());
  EXPECT_EQ(second->getShape(), originalShape);
}

//==============================================================================
TEST(World, FrozenSimpleFrameCopyAndWeldMergeRejectBeforeEffects)
{
  auto world = World::create();
  auto frame = std::make_shared<SimpleFrame>();
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frame->createDynamicsAspect();
  world->addSimpleFrame(frame);
  auto robot = Skeleton::create("welded");
  auto* parent = robot->createJointAndBodyNodePair<FreeJoint>().second;
  auto weld = robot->createJointAndBodyNodePair<WeldJoint>(parent);
  world->addSkeleton(robot);
  world->compile();
  const auto parameters = world->getParameterRevision();
  frame->getDynamicsAspect()->setFrictionCoeff(0.8);
  EXPECT_GT(world->getParameterRevision(), parameters);
  SimpleFrame source;
  source.setShape(std::make_shared<BoxShape>(2.0 * Eigen::Vector3d::Ones()));
  source.setRelativeTranslation(Eigen::Vector3d::UnitX());
  const auto oldShape = frame->getShape();
  const auto oldTransform = frame->getRelativeTransform();
  EXPECT_THROW(frame->copy(source, Frame::World()), std::runtime_error);
  EXPECT_EQ(frame->getShape(), oldShape);
  EXPECT_EQ(frame->getRelativeTransform().matrix(), oldTransform.matrix());
  const auto oldInertia = parent->getInertia();
  const auto oldForce = parent->getExternalForceLocal();
  EXPECT_THROW(weld.first->merge(), std::runtime_error);
  EXPECT_EQ(parent->getInertia(), oldInertia);
  EXPECT_EQ(parent->getExternalForceLocal(), oldForce);
  EXPECT_EQ(robot->getNumBodyNodes(), 2u);
  EXPECT_EQ(weld.second->getParentBodyNode(), parent);
}

//==============================================================================
TEST(World, CompiledWorldDetectsRawSubscribedViewChanges)
{
  auto world = World::create();
  auto view = Group::create("subscribed_view");
  world->getConstraintSolver()->getCollisionGroup()->subscribeTo(view);
  world->compile();
  auto loose = Skeleton::create();
  auto* body = loose->createJointAndBodyNodePair<RevoluteJoint>().second;
  view->addBodyNode(body);
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_THROW(world->compile(), std::runtime_error);
}

//==============================================================================
TEST(World, FrozenLinkageReassembleChecksEveryMoveBeforeEffects)
{
  auto original = Skeleton::create("original");
  auto* root = original->createJointAndBodyNodePair<RevoluteJoint>().second;
  auto* middle
      = original->createJointAndBodyNodePair<RevoluteJoint>(root).second;
  auto* leaf
      = original->createJointAndBodyNodePair<RevoluteJoint>(middle).second;
  auto linkage = Linkage::create(Linkage::Criteria(root, leaf));
  auto loose = Skeleton::create("loose");
  auto frozen = Skeleton::create("frozen");
  ASSERT_TRUE(middle->moveTo(loose, nullptr));
  ASSERT_TRUE(leaf->moveTo(frozen, nullptr));
  auto world = World::create();
  world->addSkeleton(frozen);
  world->compile();
  EXPECT_THROW(linkage->reassemble(), std::runtime_error);
  EXPECT_EQ(middle->getSkeleton(), loose);
  EXPECT_EQ(middle->getParentBodyNode(), nullptr);
  EXPECT_EQ(leaf->getSkeleton(), frozen);
  EXPECT_EQ(leaf->getParentBodyNode(), nullptr);
  EXPECT_EQ(original->getNumBodyNodes(), 1u);
}

//==============================================================================
TEST(World, VisualShapeVersionSignalsPreserveNestedGeometryClassification)
{
  auto world = World::create();
  auto robot = createBox(Eigen::Vector3d::Ones());
  auto cloud = std::make_shared<PointCloudShape>();
  cloud->addPoint(Eigen::Vector3d::Zero());
  robot->getBodyNode(0)->createShapeNodeWith<VisualAspect>(cloud);
  world->addSkeleton(robot);
  world->compile();
  const auto topology = world->getTopologyRevision();
  const auto parameter = world->getParameterRevision();
  const auto structural = SkeletonVersionAccess::getGlobalStructuralVersion();
  cloud->notifyColorUpdated(Eigen::Vector4d::Ones());
  cloud->setVisualSize(0.05);
  cloud->setPointShapeType(PointCloudShape::BILLBOARD_CIRCLE);
  EXPECT_EQ(world->getParameterRevision(), parameter);
  EXPECT_EQ(world->getTopologyRevision(), topology);
  bool nested = false;
  auto connection = cloud->onVersionChanged.connect([&](Shape*, std::size_t) {
    if (!nested) {
      nested = true;
      cloud->addPoint(Eigen::Vector3d::UnitX());
    }
  });
  cloud->notifyColorUpdated(Eigen::Vector4d::Ones());
  EXPECT_TRUE(nested);
  EXPECT_GT(world->getParameterRevision(), parameter);
  EXPECT_EQ(world->getTopologyRevision(), topology);
  EXPECT_GT(SkeletonVersionAccess::getGlobalStructuralVersion(), structural);
}

//==============================================================================
TEST(World, CompiledGroupFrameProtectsStandaloneAncestorsAndTheirLifetime)
{
  auto world = World::create();
  auto root = std::make_shared<SimpleFrame>();
  auto child = root->spawnChildSimpleFrame("group_child");
  child->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  child->createDynamicsAspect();
  world->getConstraintSolver()->getCollisionGroup()->addShapeFrame(child.get());
  world->compile();
  const auto parameters = world->getParameterRevision();
  root->setRelativeTranslation(Eigen::Vector3d::UnitX());
  child->getDynamicsAspect()->setFrictionCoeff(0.9);
  EXPECT_GT(world->getParameterRevision(), parameters);
  EXPECT_NO_THROW(world->step());
  SimpleFrame other;
  EXPECT_THROW(root->setParentFrame(&other), std::runtime_error);
  EXPECT_EQ(root->getParentFrame(), Frame::World());
  root.reset();
  EXPECT_EQ(child->getParentFrame(), Frame::World());
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_THROW(world->compile(), std::runtime_error);
}

//==============================================================================
TEST(World, FrozenSolverBulkTransfersPreflightEverySource)
{
  auto sourceWorld = World::create();
  auto frozen = createBox(Eigen::Vector3d::Ones());
  sourceWorld->addSkeleton(frozen);
  sourceWorld->compile();
  auto destination = World::create();
  auto retained = createBox(Eigen::Vector3d::Ones());
  destination->addSkeleton(retained);
  auto* solver = destination->getConstraintSolver();
  auto constraint = std::make_shared<dart::constraint::WeldJointConstraint>(
      retained->getBodyNode(0));
  solver->addConstraint(constraint);
  const auto groupSize = solver->getCollisionGroup()->getNumShapeFrames();
  const auto handler = solver->getLastContactSurfaceHandler();
  auto loose = createBox(Eigen::Vector3d::Ones());
  loose->getBodyNode(0)->setConstraintImpulse(Eigen::Vector6d::Ones());
  EXPECT_THROW(solver->addSkeletons({loose, frozen}), std::runtime_error);
  EXPECT_EQ(solver->getSkeletons(), std::vector<SkeletonPtr>{retained});
  EXPECT_EQ(solver->getCollisionGroup()->getNumShapeFrames(), groupSize);
  EXPECT_EQ(
      loose->getBodyNode(0)->getConstraintImpulse(), Eigen::Vector6d::Ones());
  EXPECT_THROW(
      solver->setFromOtherConstraintSolver(*sourceWorld->getConstraintSolver()),
      std::runtime_error);
  EXPECT_EQ(solver->getSkeletons(), std::vector<SkeletonPtr>{retained});
  EXPECT_EQ(solver->getCollisionGroup()->getNumShapeFrames(), groupSize);
  ASSERT_EQ(solver->getNumConstraints(), 1u);
  EXPECT_EQ(solver->getConstraint(0), constraint);
  EXPECT_EQ(solver->getLastContactSurfaceHandler(), handler);
  EXPECT_EQ(sourceWorld->getSkeleton(0), frozen);
}

//==============================================================================
TEST(World, FrozenContactHandlersPreserveBothChains)
{
  using dart::constraint::ContactSurfaceHandler;
  auto world = World::create();
  auto* solver = world->getConstraintSolver();
  auto handler = std::make_shared<ContactSurfaceHandler>();
  solver->addContactSurfaceHandler(handler);
  const auto parent = handler->getParent();
  world->compile();
  auto candidateParent = std::make_shared<ContactSurfaceHandler>();
  auto candidate = std::make_shared<ContactSurfaceHandler>(candidateParent);
  EXPECT_THROW(solver->addContactSurfaceHandler(candidate), std::runtime_error);
  EXPECT_EQ(candidate->getParent(), candidateParent);
  EXPECT_EQ(solver->getLastContactSurfaceHandler(), handler);
  EXPECT_THROW(
      solver->removeContactSurfaceHandler(handler), std::runtime_error);
  EXPECT_THROW(handler->setParent(nullptr), std::runtime_error);
  EXPECT_EQ(handler->getParent(), parent);
  auto unrelated = World::create();
  const auto unrelatedHandler
      = unrelated->getConstraintSolver()->getLastContactSurfaceHandler();
  EXPECT_THROW(
      unrelated->getConstraintSolver()->addContactSurfaceHandler(handler),
      std::runtime_error);
  EXPECT_EQ(
      unrelated->getConstraintSolver()->getLastContactSurfaceHandler(),
      unrelatedHandler);
  EXPECT_EQ(handler->getParent(), parent);
  const auto revision = world->getTopologyRevision();
  {
    auto edit = world->edit();
    EXPECT_TRUE(solver->removeContactSurfaceHandler(handler));
    solver->addContactSurfaceHandler(candidate);
  }
  EXPECT_GT(world->getTopologyRevision(), revision);
  EXPECT_EQ(solver->getLastContactSurfaceHandler(), candidate);
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
TEST(World, SolverAndDeactivationParametersAdvanceOnlyParameterRevision)
{
  auto world = World::create();
  world->compile();
  const auto topology = world->getTopologyRevision();
  auto previous = world->getParameterRevision();
  auto deactivation = world->getDeactivationOptions();
  deactivation.mLinearSpeedThreshold *= 2;
  world->setDeactivationOptions(deactivation);
  EXPECT_GT(world->getParameterRevision(), previous);
  previous = world->getParameterRevision();
  world->setDeactivationOptions(deactivation);
  EXPECT_EQ(world->getParameterRevision(), previous);
  auto* solver = world->getConstraintSolver();
  solver->setTimeStep(solver->getTimeStep() * 2);
  EXPECT_GT(world->getParameterRevision(), previous);
  previous = world->getParameterRevision();
  solver->setTimeStep(solver->getTimeStep());
  EXPECT_EQ(world->getParameterRevision(), previous);
  solver->setSplitImpulseEnabled(!solver->isSplitImpulseEnabled());
  EXPECT_GT(world->getParameterRevision(), previous);
  previous = world->getParameterRevision();
  solver->setSplitImpulseEnabled(solver->isSplitImpulseEnabled());
  EXPECT_EQ(world->getParameterRevision(), previous);
  auto* boxed
      = dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(solver);
  ASSERT_NE(boxed, nullptr);
  auto options = boxed->getMatrixFreeContactSolverOptions();
  options.mMaxIterations += 1;
  boxed->setMatrixFreeContactSolverOptions(options);
  EXPECT_GT(world->getParameterRevision(), previous);
  previous = world->getParameterRevision();
  boxed->setMatrixFreeContactSolverOptions(options);
  EXPECT_EQ(world->getParameterRevision(), previous);
  EXPECT_NO_THROW(world->step());
  EXPECT_EQ(world->getParameterRevision(), previous);
  EXPECT_EQ(world->getTopologyRevision(), topology);
}

//==============================================================================
TEST(World, BorrowedGroupFrameDestructionIsDetectedWithoutDanglingOwnership)
{
  auto world = World::create();
  auto frame = std::make_shared<SimpleFrame>();
  auto child = frame->spawnChildSimpleFrame("borrowed_child");
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  auto group = world->getConstraintSolver()->getCollisionGroup();
  group->addShapeFrame(frame.get());
  world->compile();
  frame.reset();
  EXPECT_EQ(child->getParentFrame(), Frame::World());
  EXPECT_EQ(group->getNumShapeFrames(), 0u);
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_THROW(world->compile(), std::runtime_error);
}

//==============================================================================
TEST(World, ThreadStorageChangesRequireEditBeforeAnyPoolChanges)
{
  auto world = World::create();
  world->compile();
  auto* solver = world->getConstraintSolver();
  const auto previous = world->getNumSimulationThreads();
  EXPECT_THROW(
      world->setNumSimulationThreads(previous + 1u), std::runtime_error);
  EXPECT_EQ(world->getNumSimulationThreads(), previous);
  EXPECT_EQ(solver->getNumSimulationThreads(), previous);
  EXPECT_THROW(
      solver->setNumSimulationThreads(previous + 1u), std::runtime_error);
  EXPECT_EQ(solver->getNumSimulationThreads(), previous);
  {
    auto edit = world->edit();
    world->setNumSimulationThreads(previous + 1u);
  }
  EXPECT_EQ(world->getNumSimulationThreads(), previous + 1u);
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
TEST(World, NestedSolverAndManualConstraintParametersStampTheirOwners)
{
  using namespace dart::constraint;
  auto world = World::create();
  auto unrelated = World::create();
  auto robot = createBox(Eigen::Vector3d::Ones());
  world->addSkeleton(robot);
  auto* boxed
      = dynamic_cast<BoxedLcpConstraintSolver*>(world->getConstraintSolver());
  ASSERT_NE(boxed, nullptr);
  auto fbf = std::make_shared<FbfFrictionSolver>();
  auto pgs = std::make_shared<PgsBoxedLcpSolver>();
  boxed->setBoxedLcpSolver(fbf);
  boxed->setSecondaryBoxedLcpSolver(pgs);
  auto constraint
      = std::make_shared<WeldJointConstraint>(robot->getBodyNode(0));
  boxed->addConstraint(constraint);
  world->compile();
  const auto topology = world->getTopologyRevision();
  const auto unrelatedParameters = unrelated->getParameterRevision();
  auto previous = world->getParameterRevision();
  auto fbfOptions = fbf->getOptions();
  fbfOptions.maxOuterIterations += 1;
  fbf->setOptions(fbfOptions);
  EXPECT_GT(world->getParameterRevision(), previous);
  previous = world->getParameterRevision();
  fbf->setOptions(fbfOptions);
  EXPECT_EQ(world->getParameterRevision(), previous);
  auto pgsOptions = pgs->getOption();
  pgsOptions.mMaxIteration += 1;
  pgs->setOption(pgsOptions);
  EXPECT_GT(world->getParameterRevision(), previous);
  previous = world->getParameterRevision();
  pgs->setOption(pgsOptions);
  EXPECT_EQ(world->getParameterRevision(), previous);
  auto transform = constraint->getRelativeTransform();
  transform.translation().x() += 0.1;
  constraint->setRelativeTransform(transform);
  EXPECT_GT(world->getParameterRevision(), previous);
  previous = world->getParameterRevision();
  constraint->setRelativeTransform(transform);
  EXPECT_EQ(world->getParameterRevision(), previous);
  EXPECT_EQ(world->getTopologyRevision(), topology);
  auto nsgs = std::make_shared<NsgsFrictionSolver>();
  {
    auto edit = world->edit();
    boxed->setBoxedLcpSolver(nsgs);
  }
  previous = world->getParameterRevision();
  auto nsgsOptions = nsgs->getOptions();
  nsgsOptions.maxSweeps += 1;
  nsgs->setOptions(nsgsOptions);
  EXPECT_GT(world->getParameterRevision(), previous);
  previous = world->getParameterRevision();
  nsgs->setOptions(nsgsOptions);
  EXPECT_EQ(world->getParameterRevision(), previous);
  EXPECT_EQ(unrelated->getParameterRevision(), unrelatedParameters);
}

//==============================================================================
TEST(World, GlobalConstraintParametersStampAllAffectedWorlds)
{
  using namespace dart::constraint;
  struct Parameter
  {
    double (*get)();
    void (*set)(double);
  };
  const Parameter parameters[]
      = {{ContactConstraint::getConstraintForceMixing,
          ContactConstraint::setConstraintForceMixing},
         {SoftContactConstraint::getConstraintForceMixing,
          SoftContactConstraint::setConstraintForceMixing},
         {JointConstraint::getConstraintForceMixing,
          JointConstraint::setConstraintForceMixing},
         {DynamicJointConstraint::getConstraintForceMixing,
          DynamicJointConstraint::setConstraintForceMixing},
         {JointLimitConstraint::getConstraintForceMixing,
          JointLimitConstraint::setConstraintForceMixing},
         {JointCoulombFrictionConstraint::getConstraintForceMixing,
          JointCoulombFrictionConstraint::setConstraintForceMixing},
         {ServoMotorConstraint::getConstraintForceMixing,
          ServoMotorConstraint::setConstraintForceMixing},
         {MimicMotorConstraint::getConstraintForceMixing,
          MimicMotorConstraint::setConstraintForceMixing},
         {CouplerConstraint::getConstraintForceMixing,
          CouplerConstraint::setConstraintForceMixing}};
  auto first = World::create();
  auto second = World::create();
  first->compile();
  second->compile();
  const auto topology = first->getTopologyRevision();
  const auto structural = SkeletonVersionAccess::getGlobalStructuralVersion();
  for (const auto& parameter : parameters) {
    const double original = parameter.get();
    const double changed = original * 0.5 + 1e-5;
    auto firstRevision = first->getParameterRevision();
    auto secondRevision = second->getParameterRevision();
    parameter.set(changed);
    EXPECT_GT(first->getParameterRevision(), firstRevision);
    EXPECT_GT(second->getParameterRevision(), secondRevision);
    firstRevision = first->getParameterRevision();
    secondRevision = second->getParameterRevision();
    parameter.set(changed);
    EXPECT_EQ(first->getParameterRevision(), firstRevision);
    EXPECT_EQ(second->getParameterRevision(), secondRevision);
    parameter.set(original);
  }
  EXPECT_EQ(first->getTopologyRevision(), topology);
  EXPECT_EQ(SkeletonVersionAccess::getGlobalStructuralVersion(), structural);
}

//==============================================================================
TEST(World, SharedDetectorConfigurationChecksEveryOwnerBeforeThreadEffects)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto frozen = World::create();
  auto lazy = World::create();
  frozen->getConstraintSolver()->setCollisionDetector(detector);
  lazy->getConstraintSolver()->setCollisionDetector(detector);
  frozen->compile();
  const auto threads = detector->getNumCollisionThreads();
  EXPECT_THROW(lazy->setNumSimulationThreads(threads + 1u), std::runtime_error);
  EXPECT_EQ(lazy->getNumSimulationThreads(), threads);
  EXPECT_EQ(lazy->getConstraintSolver()->getNumSimulationThreads(), threads);
  EXPECT_THROW(
      lazy->getConstraintSolver()->setNumSimulationThreads(threads + 1u),
      std::runtime_error);
  EXPECT_EQ(lazy->getConstraintSolver()->getNumSimulationThreads(), threads);
  EXPECT_THROW(
      detector->setNumCollisionThreads(threads + 1u), std::runtime_error);
  EXPECT_EQ(detector->getNumCollisionThreads(), threads);
  auto candidateWorld = World::create();
  candidateWorld->setNumSimulationThreads(threads + 1u);
  const auto oldDetector
      = candidateWorld->getConstraintSolver()->getCollisionDetector();
  const auto oldGroup
      = candidateWorld->getConstraintSolver()->getCollisionGroup();
  EXPECT_THROW(
      candidateWorld->getConstraintSolver()->setCollisionDetector(detector),
      std::runtime_error);
  EXPECT_EQ(
      candidateWorld->getConstraintSolver()->getCollisionDetector(),
      oldDetector);
  EXPECT_EQ(
      candidateWorld->getConstraintSolver()->getCollisionGroup(), oldGroup);
  auto retained = createBox(Eigen::Vector3d::Ones());
  lazy->addSkeleton(retained);
  const auto lazyGroup = lazy->getConstraintSolver()->getCollisionGroup();
  const auto lazyGroupSize = lazyGroup->getNumShapeFrames();
  const auto lazyHandler
      = lazy->getConstraintSolver()->getLastContactSurfaceHandler();
  EXPECT_THROW(
      lazy->getConstraintSolver()->setFromOtherConstraintSolver(
          *candidateWorld->getConstraintSolver()),
      std::runtime_error);
  EXPECT_EQ(
      lazy->getConstraintSolver()->getSkeletons(),
      std::vector<SkeletonPtr>{retained});
  EXPECT_EQ(lazy->getConstraintSolver()->getCollisionGroup(), lazyGroup);
  EXPECT_EQ(lazyGroup->getNumShapeFrames(), lazyGroupSize);
  EXPECT_EQ(
      lazy->getConstraintSolver()->getLastContactSurfaceHandler(), lazyHandler);
  EXPECT_EQ(lazy->getConstraintSolver()->getNumSimulationThreads(), threads);
  const auto frozenParameters = frozen->getParameterRevision();
  const auto lazyParameters = lazy->getParameterRevision();
  detector->setSoftFaceInteriorContactsEnabled(
      !detector->getSoftFaceInteriorContactsEnabled());
  EXPECT_GT(frozen->getParameterRevision(), frozenParameters);
  // Lazy Worlds track no revisions through shared objects.
  EXPECT_EQ(lazy->getParameterRevision(), lazyParameters);
  EXPECT_NO_THROW(frozen->step());
  auto fclWorld = World::create();
  auto fcl = std::dynamic_pointer_cast<dart::collision::FCLCollisionDetector>(
      fclWorld->getConstraintSolver()->getCollisionDetector());
  ASSERT_NE(fcl, nullptr);
  fclWorld->compile();
  const auto primitive = fcl->getPrimitiveShapeType();
  const auto replacement
      = primitive == dart::collision::FCLCollisionDetector::MESH
            ? dart::collision::FCLCollisionDetector::PRIMITIVE
            : dart::collision::FCLCollisionDetector::MESH;
  EXPECT_THROW(fcl->setPrimitiveShapeType(replacement), std::runtime_error);
  EXPECT_EQ(fcl->getPrimitiveShapeType(), primitive);
  const auto parameters = fclWorld->getParameterRevision();
  const auto method = fcl->getContactPointComputationMethod();
  fcl->setContactPointComputationMethod(
      method == dart::collision::FCLCollisionDetector::DART
          ? dart::collision::FCLCollisionDetector::FCL
          : dart::collision::FCLCollisionDetector::DART);
  EXPECT_GT(fclWorld->getParameterRevision(), parameters);
}

//==============================================================================
TEST(World, EmptyBodySubscriptionExpirationRejectsBeforeTimeAdvance)
{
  auto world = World::create();
  auto loose = Skeleton::create();
  auto* body = loose->createJointAndBodyNodePair<FreeJoint>().second;
  auto group = world->getConstraintSolver()->getCollisionGroup();
  group->subscribeTo(ConstBodyNodePtr(body));
  ASSERT_EQ(group->getNumShapeFrames(), 0u);
  world->compile();
  loose.reset();
  const auto time = world->getTime();
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_EQ(world->getTime(), time);
  EXPECT_THROW(world->compile(), std::runtime_error);
}

//==============================================================================
TEST(World, SolverReplacementPreflightsFinalWorldThreadsBeforeModelStateChanges)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto frozen = World::create();
  frozen->getConstraintSolver()->setCollisionDetector(detector);
  frozen->compile();
  auto world = World::create();
  world->setNumSimulationThreads(2u);
  auto* oldSolver = world->getConstraintSolver();
  oldSolver->setNumSimulationThreads(1u);
  auto robot = createBox(Eigen::Vector3d::Ones());
  world->addSkeleton(robot);
  auto* body = robot->getBodyNode(0);
  body->setConstraintImpulse(Eigen::Vector6d::Ones());
  DART_SUPPRESS_DEPRECATED_BEGIN
  body->setColliding(true);
  DART_SUPPRESS_DEPRECATED_END
  const auto oldGroup = oldSolver->getCollisionGroup();
  const auto oldFrames = oldGroup->getNumShapeFrames();
  auto candidate
      = std::make_unique<dart::constraint::BoxedLcpConstraintSolver>();
  candidate->setCollisionDetector(detector);
  EXPECT_THROW(
      world->setConstraintSolver(std::move(candidate)), std::runtime_error);
  EXPECT_EQ(world->getConstraintSolver(), oldSolver);
  EXPECT_EQ(oldSolver->getCollisionGroup(), oldGroup);
  EXPECT_EQ(oldGroup->getNumShapeFrames(), oldFrames);
  EXPECT_EQ(oldSolver->getSkeletons(), std::vector<SkeletonPtr>{robot});
  EXPECT_EQ(body->getConstraintImpulse(), Eigen::Vector6d::Ones());
  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_TRUE(body->isColliding());
  DART_SUPPRESS_DEPRECATED_END
}

namespace {

// A custom Node may manage a property and change its associated body without
// attaching itself to a BodyNode's NodeMap. The ownership readers must observe
// its current association on every World visit.
// Quiet Frames have no registered child-set entries. A custom reader can
// change this protected association without recursively taking the World lock.
class OwnershipQueryFrameArm : public Frame
{
public:
  explicit OwnershipQueryFrameArm(
      bool* phase = nullptr, Frame* parent = Frame::World())
    : Entity(ConstructFrame), Frame(parent), mPhase(phase)
  {
    if (mPhase) {
      *mPhase = dynamic_cast<const common::Composite*>(this) != nullptr;
      common::detail::notifyFrameTopologyMutation(this);
    }
  }

  ~OwnershipQueryFrameArm() override
  {
    if (mPhase) {
      *mPhase = dynamic_cast<const common::Composite*>(this) != nullptr;
      common::detail::notifyFrameTopologyMutation(this);
    }
  }

  const std::string& setName(const std::string& name) override
  {
    mName = name;
    return mName;
  }
  const std::string& getName() const override
  {
    return mName;
  }
  const Eigen::Isometry3d& getRelativeTransform() const override
  {
    return mTransform;
  }
  const Eigen::Vector6d& getRelativeSpatialVelocity() const override
  {
    return mZero;
  }
  const Eigen::Vector6d& getRelativeSpatialAcceleration() const override
  {
    return mZero;
  }
  const Eigen::Vector6d& getPrimaryRelativeAcceleration() const override
  {
    return mZero;
  }
  const Eigen::Vector6d& getPartialAcceleration() const override
  {
    return mZero;
  }

  void associate(Frame* parent)
  {
    mParentFrame = parent;
  }
  void announce()
  {
    common::detail::notifyFrameTopologyMutation(this);
  }

private:
  bool* mPhase;
  std::string mName;
  Eigen::Isometry3d mTransform = Eigen::Isometry3d::Identity();
  Eigen::Vector6d mZero = Eigen::Vector6d::Zero();
};

class OwnershipQueryLifecycle final : public OwnershipQueryFrameArm,
                                      public common::Composite
{
public:
  OwnershipQueryLifecycle(bool* phase, Frame* parent)
    : Entity(Frame::World(), true), OwnershipQueryFrameArm(phase, parent)
  {
  }
};

} // namespace

TEST(World, OwnershipQueryResolvesEachLifecyclePhaseSeparately)
{
  auto world = World::create();
  auto model = createBox(Eigen::Vector3d::Ones());
  world->addSkeleton(model);
  world->compile();
  bool compositeInBasePhase = true;
  std::unique_ptr<OwnershipQueryLifecycle> query;
  {
    auto edit = world->edit();
    const auto beforeConstruction = world->getTopologyRevision();
    query = std::make_unique<OwnershipQueryLifecycle>(
        &compositeInBasePhase, model->getBodyNode(0));
    EXPECT_FALSE(compositeInBasePhase);
    EXPECT_GT(world->getTopologyRevision(), beforeConstruction);
  }
  {
    auto edit = world->edit();
    const auto topology = world->getTopologyRevision();
    query->announce();
    EXPECT_GT(world->getTopologyRevision(), topology);
  }
  {
    auto edit = world->edit();
    const auto beforeDestruction = world->getTopologyRevision();
    compositeInBasePhase = true;
    query.reset();
    EXPECT_FALSE(compositeInBasePhase);
    EXPECT_GT(world->getTopologyRevision(), beforeDestruction);
  }
}

//==============================================================================
TEST(World, LateGlobalWorldDestructionKeepsOwnerRegistryAvailable)
{
  gLateDestroyedWorld = World::create();
  gLateDestroyedWorld->compile();
  EXPECT_TRUE(gLateDestroyedWorld->isCompiled());
}

namespace {

class OwnershipCallbackShape final : public BoxShape
{
public:
  OwnershipCallbackShape() : BoxShape(Eigen::Vector3d::Ones()) {}

  const std::string& getType() const override
  {
    if (mCallback) {
      auto callback = std::move(mCallback);
      mCallback = nullptr;
      callback();
    }
    return BoxShape::getType();
  }

  mutable std::function<void()> mCallback;
};

class OwnershipBody final : public BodyNode
{
public:
  OwnershipBody(BodyNode* parent, Joint* joint, const Properties& properties)
    : Entity(Frame::World(), false),
      Frame(Frame::World()),
      BodyNode(parent, joint, properties)
  {
  }

  void reparent(Frame* parent)
  {
    Frame::changeParentFrame(parent);
  }
};

class OwnershipReaderFrame final : public SimpleFrame
{
public:
  OwnershipReaderFrame()
    : Entity(ConstructFrame), Frame(Frame::World()), SimpleFrame()
  {
  }

  const ShapeNode* asShapeNode() const override
  {
    ++mReads;
    if (mArmed)
      throw std::runtime_error("unexpected custom Frame reader");
    return nullptr;
  }

  bool mArmed = false;
  mutable std::size_t mReads = 0u;
};

class OwnershipFrame final : public SimpleFrame
{
public:
  explicit OwnershipFrame(Frame* parent)
    : Entity(ConstructFrame), Frame(parent), SimpleFrame(parent)
  {
  }

  std::function<void(Entity*)> mOnAdd;
  std::function<void(Entity*)> mOnRemove;

protected:
  void processNewEntity(Entity* child) override
  {
    if (mOnAdd)
      mOnAdd(child);
  }

  void processRemovedEntity(Entity* child) override
  {
    if (mOnRemove)
      mOnRemove(child);
  }
};

class NativeDetectorReleaseCallback
{
public:
  NativeDetectorReleaseCallback(
      constraint::ConstraintSolver* solver, std::function<void()> callback)
    : mCallback(std::make_shared<std::function<void()>>(std::move(callback)))
  {
    auto detector = collision::FCLCollisionDetector::create();
    solver->setCollisionDetector(std::shared_ptr<collision::CollisionDetector>(
        detector.get(),
        [detector, callback = mCallback](collision::CollisionDetector*) {
          if (*callback)
            (*callback)();
        }));
  }

  ~NativeDetectorReleaseCallback()
  {
    *mCallback = nullptr;
  }

private:
  std::shared_ptr<std::function<void()>> mCallback;
};

class DetectorInitializationOwnershipProbe
{
public:
  explicit DetectorInitializationOwnershipProbe(const WorldPtr& world)
    : mWorld(world),
      mRobot(createBox(Eigen::Vector3d::Ones())),
      mBody(mRobot->getBodyNode(0)),
      mParent(std::make_shared<BareFrame>(mBody))
  {
  }

  // The old group keeps resolving ownership until the solver publishes its
  // replacement.
  void checkLiveBeforeGroupNotification(
      const collision::CollisionGroupPtr& group)
  {
    group->addShapeFrame(mBody->getShapeNodeWith<CollisionAspect>(0));
    const auto parameters = mWorld->getParameterRevision();
    mBody->setCollidable(false);
    EXPECT_GT(mWorld->getParameterRevision(), parameters);
    group->removeAllShapeFrames();
    const auto cleared = mWorld->getParameterRevision();
    mBody->setCollidable(true);
    EXPECT_EQ(mWorld->getParameterRevision(), cleared);
    EXPECT_EQ(group->getNumShapeFrames(), 0u);
  }

  void checkBorrowedViewAndFrozenMutation()
  {
    auto view = Group::create("detector_initialization_view");
    auto group = mWorld->getConstraintSolver()->getCollisionGroup();
    {
      auto edit = mWorld->edit();
      group->subscribeTo(view);
      ASSERT_TRUE(view->addBodyNode(mBody));
      // Views resolve ownership when the collision group synchronizes.
      group->update();
      auto parameters = mWorld->getParameterRevision();
      mBody->setCollidable(false);
      EXPECT_GT(mWorld->getParameterRevision(), parameters);
      ASSERT_TRUE(view->removeBodyNode(mBody));
      group->update();
      parameters = mWorld->getParameterRevision();
      mBody->setCollidable(true);
      EXPECT_EQ(mWorld->getParameterRevision(), parameters);
      ASSERT_TRUE(view->addBodyNode(mBody));
    }
    EXPECT_NO_THROW(mWorld->compile());
    const auto children = mParent->getChildFrames();
    EXPECT_THROW(
        static_cast<void>(std::make_shared<BareFrame>(mParent.get())),
        std::runtime_error);
    EXPECT_EQ(mParent->getChildFrames(), children);
    EXPECT_THROW(
        mBody->createShapeNodeWith<CollisionAspect>(
            std::make_shared<BoxShape>(Eigen::Vector3d::Ones())),
        std::runtime_error);
  }

private:
  WorldPtr mWorld;
  SkeletonPtr mRobot;
  BodyNode* mBody;
  std::shared_ptr<BareFrame> mParent;
};

} // namespace

TEST(World, DetectorInitializationReleaseKeepsBorrowedOwnershipLive)
{
  auto world = World::create();
  world->compile();
  auto* solver = world->getConstraintSolver();
  DetectorInitializationOwnershipProbe probe(world);
  collision::CollisionGroupPtr oldGroup;
  auto incoming = collision::FCLCollisionDetector::create();
  bool called = false;
  auto install = std::make_unique<World::EditScope>(*world);
  NativeDetectorReleaseCallback release(solver, [&] {
    called = true;
    EXPECT_EQ(solver->getCollisionDetector(), incoming);
    EXPECT_EQ(solver->getCollisionGroup(), oldGroup);
    probe.checkLiveBeforeGroupNotification(oldGroup);
  });
  install.reset();
  {
    auto edit = world->edit();
    oldGroup = solver->getCollisionGroup();
    solver->setCollisionDetector(incoming);
    ASSERT_TRUE(called);
    EXPECT_NE(solver->getCollisionGroup(), oldGroup);
  }
  probe.checkBorrowedViewAndFrozenMutation();
}

TEST(World, OwnershipUsesRawIdentityAndReleasesRemovedAliases)
{
  auto world = World::create();
  auto robot = createBox(Eigen::Vector3d::Ones());
  auto token = std::make_shared<int>(0);
  SkeletonPtr alias(token, robot.get());
  ASSERT_TRUE(alias.owner_before(robot) || robot.owner_before(alias));
  world->addSkeleton(alias);
  world->addSkeleton(robot);
  ASSERT_EQ(world->getNumSkeletons(), 1u);
  world->compile();
  EXPECT_THROW(
      robot->createJointAndBodyNodePair<FreeJoint>(), std::runtime_error);
  {
    auto edit = world->edit();
    world->removeSkeleton(alias);
  }
  EXPECT_EQ(world->getNumSkeletons(), 0u);
  EXPECT_NO_THROW(robot->createJointAndBodyNodePair<FreeJoint>());
}

// A subscribed view's bodies belong to the World once the collision group
// synchronizes with the view (update(), compile() or step()).
TEST(World, OwnershipFollowsBorrowedViewMembershipAtSynchronization)
{
  auto world = World::create();
  world->addSkeleton(createBox(Eigen::Vector3d::Ones()));
  auto view = Group::create("view");
  const auto group = world->getConstraintSolver()->getCollisionGroup();
  group->subscribeTo(view);
  world->compile();
  auto loose = Skeleton::create();
  auto* first = loose->createJointAndBodyNodePair<FreeJoint>().second;
  auto* second = loose->createJointAndBodyNodePair<FreeJoint>().second;
  {
    auto edit = world->edit();
    ASSERT_TRUE(view->addBodyNode(first));
    auto parameter = world->getParameterRevision();
    first->setCollidable(false);
    EXPECT_EQ(world->getParameterRevision(), parameter);
    group->update();
    first->setCollidable(true);
    EXPECT_GT(world->getParameterRevision(), parameter);
    ASSERT_TRUE(view->removeBodyNode(first));
    group->update();
    parameter = world->getParameterRevision();
    first->setCollidable(false);
    EXPECT_EQ(world->getParameterRevision(), parameter);
    ASSERT_TRUE(view->addBodyNode(second));
  }
  // The recompile at scope exit synchronized the group.
  auto parameter = world->getParameterRevision();
  second->setCollidable(false);
  EXPECT_GT(world->getParameterRevision(), parameter);
  EXPECT_NO_THROW(world->step());
  {
    auto edit = world->edit();
    view.reset();
    group->update();
    parameter = world->getParameterRevision();
    second->setCollidable(true);
    EXPECT_EQ(world->getParameterRevision(), parameter);
  }
}

TEST(World, OwnershipDoesNotCallCustomFrameReaderUnderRegistryLock)
{
  auto world = World::create();
  world->addSkeleton(createBox(Eigen::Vector3d::Ones()));
  auto frame = std::make_shared<OwnershipReaderFrame>();
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  world->getConstraintSolver()->getCollisionGroup()->addShapeFrame(frame.get());
  world->compile();
  frame->mReads = 0u;
  frame->mArmed = true;
  auto loose = Skeleton::create("unrelated");
  EXPECT_NO_THROW(loose->createJointAndBodyNodePair<FreeJoint>());
  EXPECT_EQ(frame->mReads, 0u);
  // Disarm before teardown so a failing oracle still exits normally.
  frame->mArmed = false;
}

TEST(World, OwnershipTracksTransferredBodiesThroughoutOneEdit)
{
  auto world = World::create();
  auto source = createBox(Eigen::Vector3d::Ones());
  auto target = Skeleton::create("target");
  world->addSkeleton(source);
  world->addSkeleton(target);
  world->compile();
  auto* body = source->getBodyNode(0);
  const auto topology = world->getTopologyRevision();
  {
    auto edit = world->edit();
    ASSERT_TRUE(body->moveTo(target, nullptr));
    ASSERT_EQ(body->getSkeleton(), target);
    auto parameter = world->getParameterRevision();
    body->setCollidable(false);
    EXPECT_GT(world->getParameterRevision(), parameter);
    auto* child = target->createJointAndBodyNodePair<FreeJoint>(body).second;
    parameter = world->getParameterRevision();
    child->setCollidable(false);
    EXPECT_GT(world->getParameterRevision(), parameter);
    ASSERT_TRUE(child->moveTo(source, nullptr));
    parameter = world->getParameterRevision();
    child->setCollidable(true);
    EXPECT_GT(world->getParameterRevision(), parameter);
    EXPECT_EQ(world->getTopologyRevision(), topology + 1u);
  }
  EXPECT_GT(world->getTopologyRevision(), topology);
  EXPECT_THROW(
      target->createJointAndBodyNodePair<FreeJoint>(), std::runtime_error);
  EXPECT_THROW(
      source->createJointAndBodyNodePair<FreeJoint>(), std::runtime_error);
}

TEST(World, OwnershipKeepsSharedBorrowedMembershipLive)
{
  auto first = World::create();
  auto second = World::create();
  auto robot = createBox(Eigen::Vector3d::Ones());
  first->addSkeleton(robot);
  second->getConstraintSolver()->getCollisionGroup()->subscribeTo(robot);
  first->compile();
  auto firstParameter = first->getParameterRevision();
  auto secondParameter = second->getParameterRevision();
  robot->getBodyNode(0)->setCollidable(false);
  EXPECT_GT(first->getParameterRevision(), firstParameter);
  // A lazy borrower has no registry entry and tracks no revisions.
  EXPECT_EQ(second->getParameterRevision(), secondParameter);
  const auto indexed = dart::simulation::detail::getIndexedWorldCount();
  EXPECT_THROW(second->compile(), std::runtime_error);
  EXPECT_FALSE(second->isCompiled());
  // A failed first compile leaves the registry again.
  EXPECT_EQ(dart::simulation::detail::getIndexedWorldCount(), indexed);
  second->getConstraintSolver()->getCollisionGroup()->unsubscribeFrom(
      robot.get());
  EXPECT_NO_THROW(second->compile());
  EXPECT_EQ(dart::simulation::detail::getIndexedWorldCount(), indexed + 1u);
  EXPECT_THROW(
      robot->createJointAndBodyNodePair<FreeJoint>(), std::runtime_error);
  first.reset();
  EXPECT_NO_THROW(robot->createJointAndBodyNodePair<FreeJoint>());
}

// A view subscribed from inside a model addition is owned once synchronized.
TEST(World, OwnershipTracksViewSubscribedDuringModelAddition)
{
  auto world = World::create();
  auto robot = createBox(Eigen::Vector3d::Ones());
  auto shape = std::make_shared<OwnershipCallbackShape>();
  robot->getBodyNode(0)->getShapeNodeWith<CollisionAspect>(0)->setShape(shape);
  auto view = Group::create("callback_view");
  auto loose = Skeleton::create();
  auto* body = loose->createJointAndBodyNodePair<FreeJoint>().second;
  bool called = false;
  shape->mCallback = [&] {
    called = true;
    world->getConstraintSolver()->getCollisionGroup()->subscribeTo(view);
    ASSERT_TRUE(view->addBodyNode(body));
  };
  world->addSkeleton(robot);
  ASSERT_TRUE(called);
  world->addSkeleton(createBox(Eigen::Vector3d::Ones()));
  world->compile();
  const auto parameter = world->getParameterRevision();
  body->setCollidable(false);
  EXPECT_GT(world->getParameterRevision(), parameter);
  EXPECT_THROW(
      body->createShapeNodeWith<CollisionAspect>(
          std::make_shared<BoxShape>(Eigen::Vector3d::Ones())),
      std::runtime_error);
}

TEST(World, OwnershipNameCallbackNestedAdditionCannotRecertifyRawView)
{
  auto world = World::create();
  auto first = createBox(Eigen::Vector3d::Ones());
  first->setName("duplicate");
  world->addSkeleton(first);
  auto robot = createBox(Eigen::Vector3d::Ones());
  robot->setName("duplicate");
  auto view = Group::create("name_callback_view");
  bool called = false;
  auto connection = robot->onNameChanged.connect(
      [&](ConstMetaSkeletonPtr, const std::string&, const std::string&) {
        if (called)
          return;
        called = true;
        world->getConstraintSolver()->getCollisionGroup()->subscribeTo(view);
        world->addSkeleton(Skeleton::create("nested"));
      });
  world->addSkeleton(robot);
  ASSERT_TRUE(called);
  auto loose = Skeleton::create();
  auto* body = loose->createJointAndBodyNodePair<FreeJoint>().second;
  ASSERT_TRUE(view->addBodyNode(body));
  world->compile();
  EXPECT_THROW(
      body->createShapeNodeWith<CollisionAspect>(
          std::make_shared<BoxShape>(Eigen::Vector3d::Ones())),
      std::runtime_error);
  connection.disconnect();
}

TEST(World, OwnershipThrowingAdditionTracksActualPartialMembership)
{
  auto world = World::create();
  auto robot = createBox(Eigen::Vector3d::Ones());
  auto shape = std::make_shared<OwnershipCallbackShape>();
  robot->getBodyNode(0)->getShapeNodeWith<CollisionAspect>(0)->setShape(shape);
  shape->mCallback = [] {
    throw std::runtime_error("shape reader failure");
  };
  world->compile();
  auto edit = world->edit();
  EXPECT_THROW(world->addSkeleton(robot), std::runtime_error);
  ASSERT_EQ(world->getNumSkeletons(), 1u);
  ASSERT_EQ(world->getSkeleton(0).get(), robot.get());
  const auto parameter = world->getParameterRevision();
  robot->getBodyNode(0)->setCollidable(false);
  EXPECT_GT(world->getParameterRevision(), parameter);
  EXPECT_NO_THROW(world->removeSkeleton(robot));
  ASSERT_EQ(world->getNumSkeletons(), 0u);
  const auto removedParameter = world->getParameterRevision();
  robot->getBodyNode(0)->setCollidable(true);
  EXPECT_EQ(world->getParameterRevision(), removedParameter);
}

//==============================================================================
// A Shape override that throws while a Skeleton joins leaves no collision
// object, subscription or detector entry behind, so the model's shapes are
// freed once it is dropped.
TEST(World, ThrowingShapeAdditionLeavesNoCollisionState)
{
  std::vector<collision::CollisionDetectorPtr> detectors{
      collision::FCLCollisionDetector::create()};
#if HAVE_BULLET
  detectors.push_back(collision::BulletCollisionDetector::create());
#endif
  for (const auto& detector : detectors) {
    SCOPED_TRACE(detector->getType());
    auto world = World::create();
    world->setCollisionDetector(detector);
    auto robot = createBox(Eigen::Vector3d::Ones());
    // Joins after the box, whose collision object the failure must release.
    auto shape = std::make_shared<OwnershipCallbackShape>();
    robot->getBodyNode(0)->createShapeNodeWith<CollisionAspect>(shape);
    shape->mCallback = [] {
      throw std::runtime_error("shape reader failure");
    };
    const auto group = world->getConstraintSolver()->getCollisionGroup();
    EXPECT_THROW(world->addSkeleton(robot), std::runtime_error);
    EXPECT_EQ(group->getNumShapeFrames(), 0u);
    EXPECT_FALSE(group->isSubscribedTo(robot.get()));
    world->removeSkeleton(robot);
    const std::weak_ptr<Shape> released = shape;
    shape.reset();
    robot.reset();
    EXPECT_TRUE(released.expired());
  }
}

TEST(World, OwnershipFollowsForeignBodyFrameAncestryBeforeNativeAddition)
{
  auto world = World::create();
  auto owned = createBox(Eigen::Vector3d::Ones());
  auto foreign = Skeleton::create("foreign");
  auto* body
      = foreign->createJointAndBodyNodePair<FreeJoint, OwnershipBody>().second;
  body->reparent(owned->getBodyNode(0));
  ASSERT_EQ(body->getSkeleton(), foreign);
  world->addSkeleton(owned);
  world->compile();
  const auto parameter = world->getParameterRevision();
  body->setCollidable(false);
  EXPECT_GT(world->getParameterRevision(), parameter);
  EXPECT_THROW(
      body->createShapeNodeWith<CollisionAspect>(
          std::make_shared<BoxShape>(Eigen::Vector3d::Ones())),
      std::runtime_error);
  {
    auto edit = world->edit();
    body->reparent(Frame::World());
  }
}

TEST(World, OwnershipFrameCallbacksSeeCurrentLiveAncestry)
{
  auto world = World::create();
  auto owned = createBox(Eigen::Vector3d::Ones());
  auto parent = std::make_shared<OwnershipFrame>(owned->getBodyNode(0));
  auto child = std::make_shared<SimpleFrame>();
  child->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  child->createDynamicsAspect();
  world->addSkeleton(owned);
  world->compile();
  bool added = false;
  bool removed = false;
  parent->mOnAdd = [&](Entity* entity) {
    EXPECT_EQ(entity->getParentFrame(), parent.get());
    const auto parameter = world->getParameterRevision();
    child->getDynamicsAspect()->setFrictionCoeff(0.7);
    EXPECT_GT(world->getParameterRevision(), parameter);
    added = true;
  };
  parent->mOnRemove = [&](Entity* entity) {
    EXPECT_EQ(entity->getParentFrame(), parent.get());
    const auto parameter = world->getParameterRevision();
    child->getDynamicsAspect()->setFrictionCoeff(0.8);
    EXPECT_GT(world->getParameterRevision(), parameter);
    removed = true;
  };
  {
    auto edit = world->edit();
    child->setParentFrame(parent.get());
  }
  EXPECT_TRUE(added);
  {
    auto edit = world->edit();
    child->setParentFrame(Frame::World());
  }
  EXPECT_TRUE(removed);
  const auto parameter = world->getParameterRevision();
  child->getDynamicsAspect()->setFrictionCoeff(0.9);
  EXPECT_EQ(world->getParameterRevision(), parameter);
  parent->mOnAdd = nullptr;
  parent->mOnRemove = nullptr;
}

//==============================================================================
TEST(World, FixedFrameDescendantLifetimeIsDetectedBeforeFollowingOldPointers)
{
  auto world = World::create();
  auto root = std::make_shared<SimpleFrame>();
  auto middle = std::make_shared<NamedFixedFrame>(root.get());
  auto leaf = std::make_shared<NamedFixedFrame>(middle.get());
  ASSERT_EQ(root->getChildFrames().size(), 1u);
  ASSERT_EQ(middle->getChildFrames().size(), 1u);
  world->addSimpleFrame(root);
  world->compile();
  const auto topology = world->getTopologyRevision();
  const auto parameters = world->getParameterRevision();
  auto pose = Eigen::Isometry3d::Identity();
  pose.translation().x() = 0.2;
  middle->setRelativeTransform(pose);
  EXPECT_GT(world->getParameterRevision(), parameters);
  EXPECT_EQ(world->getTopologyRevision(), topology);
  EXPECT_NO_THROW(world->step());
  EXPECT_THROW(
      static_cast<void>(std::make_shared<SimpleFrame>(middle.get())),
      std::runtime_error);
  leaf.reset();
  const auto time = world->getTime();
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_EQ(world->getTime(), time);
  EXPECT_THROW(world->compile(), std::runtime_error);
}

//==============================================================================
TEST(World, ShapelessModelsOwnFrameDescendantsThroughEveryMembershipRoute)
{
  for (int ownership = 0; ownership < 4; ++ownership) {
    SCOPED_TRACE(ownership);
    auto world = World::create();
    auto skeleton = Skeleton::create();
    auto* body = skeleton->createJointAndBodyNodePair<FreeJoint>().second;
    auto middle = std::make_shared<NamedFixedFrame>(body);
    auto leaf = std::make_shared<SimpleFrame>(middle.get());
    ASSERT_EQ(body->getChildFrames().size(), 1u);
    ASSERT_EQ(middle->getChildFrames().size(), 1u);
    auto* solver = world->getConstraintSolver();
    switch (ownership) {
      case 0:
        world->addSkeleton(skeleton);
        break;
      case 1:
        solver->addSkeleton(skeleton);
        break;
      case 2:
        solver->getCollisionGroup()->subscribeTo(skeleton);
        break;
      case 3:
        solver->getCollisionGroup()->subscribeTo(ConstBodyNodePtr(body));
        break;
    }
    world->compile();
    const auto parameters = world->getParameterRevision();
    auto pose = Eigen::Isometry3d::Identity();
    pose.translation().x() = 0.2;
    middle->setRelativeTransform(pose);
    EXPECT_GT(world->getParameterRevision(), parameters);
    EXPECT_NO_THROW(world->step());
    const auto children = middle->getChildFrames();
    EXPECT_THROW(
        static_cast<void>(std::make_shared<SimpleFrame>(middle.get())),
        std::runtime_error);
    EXPECT_EQ(middle->getChildFrames(), children);
    EXPECT_THROW(
        body->createShapeNodeWith<VisualAspect>(
            std::make_shared<BoxShape>(Eigen::Vector3d::Ones())),
        std::runtime_error);
    EXPECT_EQ(body->getNumShapeNodes(), 0u);
    leaf.reset();
    const auto time = world->getTime();
    EXPECT_THROW(world->step(), std::runtime_error);
    EXPECT_EQ(world->getTime(), time);
    EXPECT_THROW(world->compile(), std::runtime_error);
  }
}

//==============================================================================
TEST(World, NonCompositeFramePreflightsConstructionAndReparenting)
{
  auto world = World::create();
  auto parent = std::make_shared<BareFrame>(Frame::World());
  auto member = std::make_shared<SimpleFrame>(parent.get());
  world->addSimpleFrame(member);
  world->compile();
  const auto children = parent->getChildFrames();
  ASSERT_EQ(children.size(), 1u);
  ASSERT_EQ(children.count(member.get()), 1u);
  const auto topology = world->getTopologyRevision();
  EXPECT_THROW(
      static_cast<void>(std::make_shared<SimpleFrame>(parent.get())),
      std::runtime_error);
  EXPECT_EQ(parent->getChildFrames(), children);
  EXPECT_EQ(world->getTopologyRevision(), topology);

  auto sourceWorld = World::create();
  auto sourceParent = std::make_shared<SimpleFrame>();
  auto source = std::make_shared<SimpleFrame>(sourceParent.get());
  sourceWorld->addSimpleFrame(source);
  const auto sourceChildren = sourceParent->getChildFrames();
  const auto sourceTopology = sourceWorld->getTopologyRevision();
  EXPECT_THROW(source->setParentFrame(parent.get()), std::runtime_error);
  EXPECT_EQ(source->getParentFrame(), sourceParent.get());
  EXPECT_EQ(sourceParent->getChildFrames(), sourceChildren);
  EXPECT_EQ(parent->getChildFrames(), children);
  EXPECT_EQ(sourceWorld->getTopologyRevision(), sourceTopology);
  EXPECT_EQ(world->getTopologyRevision(), topology);
  auto destination = std::make_shared<SimpleFrame>();
  const auto destinationChildren = destination->getChildFrames();
  EXPECT_THROW(parent->setParentFrame(destination.get()), std::runtime_error);
  EXPECT_EQ(parent->getParentFrame(), Frame::World());
  EXPECT_EQ(parent->getChildFrames(), children);
  EXPECT_EQ(destination->getChildFrames(), destinationChildren);
  EXPECT_EQ(world->getTopologyRevision(), topology);
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
TEST(World, NonCompositeFrameChangesStampEditedTopology)
{
  auto world = World::create();
  auto parent = std::make_shared<BareFrame>(Frame::World());
  auto member = std::make_shared<SimpleFrame>(parent.get());
  world->addSimpleFrame(member);
  world->compile();
  const auto parameters = world->getParameterRevision();
  std::shared_ptr<SimpleFrame> sibling;
  {
    auto edit = world->edit();
    const auto topology = world->getTopologyRevision();
    sibling = std::make_shared<SimpleFrame>(parent.get());
    EXPECT_GT(world->getTopologyRevision(), topology);
  }
  {
    auto edit = world->edit();
    const auto topology = world->getTopologyRevision();
    sibling->setParentFrame(Frame::World());
    EXPECT_GT(world->getTopologyRevision(), topology);
  }
  EXPECT_EQ(world->getParameterRevision(), parameters);
  const auto& children = parent->getChildFrames();
  EXPECT_EQ(children.size(), 1u);
  EXPECT_EQ(children.count(member.get()), 1u);
  auto destination = std::make_shared<SimpleFrame>();
  {
    auto edit = world->edit();
    const auto topology = world->getTopologyRevision();
    parent->setParentFrame(destination.get());
    EXPECT_GT(world->getTopologyRevision(), topology);
  }
  EXPECT_EQ(world->getParameterRevision(), parameters);
}

//==============================================================================
TEST(World, RuntimeSkeletonParametersPreserveSubscriptionVersions)
{
  auto world = World::create();
  auto unrelated = World::create();
  auto robot = createBox(Eigen::Vector3d::Ones());
  world->addSkeleton(robot);
  world->compile();
  const auto topology = world->getTopologyRevision();
  const auto version = robot->getVersion();
  const auto unrelatedParameters = unrelated->getParameterRevision();
  const auto checkChange = [&](const auto& setter) {
    const auto parameters = world->getParameterRevision();
    setter();
    EXPECT_GT(world->getParameterRevision(), parameters);
    const auto changedParameters = world->getParameterRevision();
    setter();
    EXPECT_EQ(world->getParameterRevision(), changedParameters);
    EXPECT_EQ(robot->getVersion(), version);
    EXPECT_EQ(world->getTopologyRevision(), topology);
    EXPECT_EQ(unrelated->getParameterRevision(), unrelatedParameters);
  };
  checkChange([&] { robot->setMobile(false); });
  checkChange([&] { robot->setSelfCollisionCheck(true); });
  checkChange([&] { robot->setAdjacentBodyCheck(true); });
  checkChange([&] { robot->setTimeStep(0.002); });
  checkChange([&] { robot->setGravity(Eigen::Vector3d(1.0, 0.0, -9.81)); });
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
namespace {

class StepCallbackHandler final : public constraint::ContactSurfaceHandler
{
public:
  constraint::ContactSurfaceParams createParams(
      const collision::Contact& contact,
      std::size_t numContactsOnCollisionObject) const override
  {
    if (mCallback)
      mCallback();
    return ContactSurfaceHandler::createParams(
        contact, numContactsOnCollisionObject);
  }

  std::function<void()> mCallback;
};

// Runs a callback from the collision filter, which the solver's preparation
// pass and every collide() query reach.
class CallbackCollisionFilter final : public collision::BodyNodeCollisionFilter
{
public:
  bool ignoresCollision(
      const collision::CollisionObject* object1,
      const collision::CollisionObject* object2) const override
  {
    if (mCallback)
      mCallback();
    return BodyNodeCollisionFilter::ignoresCollision(object1, object2);
  }

  std::function<void()> mCallback;
};

class RevisionCallbackCollisionFilter final
  : public collision::BodyNodeCollisionFilter,
    public collision::detail::CollisionFilterSnapshotTracker
{
public:
  std::size_t getCollisionFilterSnapshotRevision() const override
  {
    if (mCallback)
      mCallback();
    return getBodyNodePairBlackListRevision();
  }

  std::function<void()> mCallback;
};

class CountingFclDetector final : public collision::FCLCollisionDetector
{
public:
  explicit CountingFclDetector(int& destroyed) : mDestroyed(destroyed) {}

  ~CountingFclDetector() override
  {
    ++mDestroyed;
  }

private:
  int& mDestroyed;
};

template <class Body>
void runCaptured(Body body, std::exception_ptr& error)
{
  try {
    body();
  } catch (...) {
    error = std::current_exception();
  }
}

} // namespace

//==============================================================================
// Independent lazy Worlds on separate threads never read each other's model
// containers; lazy Worlds have no registry entries at all.
TEST(World, IndependentLazyWorldsMutateConcurrently)
{
  auto worldA = World::create();
  auto groundA = createBox(Eigen::Vector3d(10.0, 10.0, 0.2));
  groundA->setMobile(false);
  auto boxA
      = createBox(Eigen::Vector3d::Ones(), Eigen::Vector3d(0.0, 0.0, 0.6));
  worldA->addSkeleton(groundA);
  worldA->addSkeleton(boxA);
  auto looseA = Skeleton::create("looseA");
  auto* looseBodyA = looseA->createJointAndBodyNodePair<FreeJoint>().second;
  looseBodyA->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  looseA->setPositions(
      (Eigen::Vector6d() << 0.0, 0.0, 0.0, 5.0, 5.0, 5.0).finished());
  auto* shapeNodeA = boxA->getBodyNode(0)->getShapeNodeWith<CollisionAspect>(0);
  const auto groupA = worldA->getConstraintSolver()->getCollisionGroup();

  auto worldB = World::create();
  auto robotB = createBox(Eigen::Vector3d::Ones());
  worldB->addSkeleton(robotB);
  auto* bodyB = robotB->getBodyNode(0);
  auto* dynamicsB
      = bodyB->getShapeNodeWith<CollisionAspect>(0)->getDynamicsAspect();
  auto* jointB = robotB->getJoint(0);

  std::exception_ptr errorA;
  std::exception_ptr errorB;
  std::thread threadA([&] {
    runCaptured(
        [&] {
          for (int i = 0; i < 300; ++i) {
            auto frame = std::make_shared<SimpleFrame>(Frame::World(), "fA");
            worldA->addSimpleFrame(frame);
            groupA->subscribeTo(looseA);
            shapeNodeA->setShape(std::make_shared<BoxShape>(
                Eigen::Vector3d::Constant(1.0 + 0.01 * (i % 2))));
            worldA->step();
            groupA->unsubscribeFrom(looseA.get());
            worldA->removeSimpleFrame(frame);
          }
        },
        errorA);
  });
  std::thread threadB([&] {
    runCaptured(
        [&] {
          for (int i = 0; i < 3000; ++i) {
            bodyB->setMass(1.0 + 0.1 * (i % 7));
            dynamicsB->setFrictionCoeff(0.3 + 0.01 * (i % 5));
            dynamicsB->setPrimarySlipCompliance(1e-6 * (i % 3));
            jointB->setPositionUpperLimit(0, 1.0 + 0.001 * (i % 11));
            auto* child
                = robotB->createJointAndBodyNodePair<RevoluteJoint>(bodyB)
                      .second;
            child->remove("detached");
            if (i % 100 == 0)
              worldB->step();
          }
        },
        errorB);
  });
  threadA.join();
  threadB.join();
  if (errorA)
    std::rethrow_exception(errorA);
  if (errorB)
    std::rethrow_exception(errorB);
  EXPECT_EQ(worldA->getNumSimpleFrames(), 0u);
  EXPECT_EQ(robotB->getNumBodyNodes(), 1u);
}

//==============================================================================
TEST(World, EditOnLazyWorldLeavesItLazy)
{
  auto first = World::create();
  auto second = World::create();
  auto robot = createBox(Eigen::Vector3d::Ones());
  first->addSkeleton(robot);
  second->addSkeleton(robot);
  {
    auto edit = first->edit();
    first->addSimpleFrame(std::make_shared<SimpleFrame>());
  }
  EXPECT_FALSE(first->isCompiled());
  EXPECT_NO_THROW(
      robot->createJointAndBodyNodePair<RevoluteJoint>(robot->getBodyNode(0)));
  EXPECT_NO_THROW(first->step());
  EXPECT_NO_THROW(second->step());
  first->removeSkeleton(robot);
  second->removeSkeleton(robot);
  {
    auto edit = first->edit();
  }
  EXPECT_FALSE(first->isCompiled());
  EXPECT_NO_THROW(first->addSkeleton(robot));
}

//==============================================================================
TEST(World, FailedRecompileAtScopeExitSurfacesAtNextStep)
{
  auto world = World::create();
  world->setCollisionDetector(CollisionDetectorType::Dart);
  auto robot = createBox(Eigen::Vector3d::Ones());
  auto* shape = robot->getBodyNode(0)->getShapeNodeWith<CollisionAspect>(0);
  const auto box = shape->getShape();
  world->addSkeleton(robot);
  world->compile();
  auto scope = std::make_unique<World::EditScope>(*world);
  shape->setShape(std::make_shared<LineSegmentShape>());
  EXPECT_NO_THROW(scope.reset());
  EXPECT_FALSE(world->isCompiled());
  const auto time = world->getTime();
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_EQ(world->getTime(), time);
  // Repairing the model does not silence the deferred error: the next
  // compile() reports it once and the one after that compiles.
  EXPECT_NO_THROW(shape->setShape(box));
  EXPECT_THROW(world->compile(), std::runtime_error);
  EXPECT_NO_THROW(world->compile());
  EXPECT_TRUE(world->isCompiled());
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
TEST(World, RepairEditConsumesDeferredRecompileError)
{
  auto world = World::create();
  world->setCollisionDetector(CollisionDetectorType::Dart);
  auto robot = createBox(Eigen::Vector3d::Ones());
  auto* shape = robot->getBodyNode(0)->getShapeNodeWith<CollisionAspect>(0);
  const auto box = shape->getShape();
  world->addSkeleton(robot);
  world->compile();
  {
    auto edit = world->edit();
    shape->setShape(std::make_shared<LineSegmentShape>());
  }
  ASSERT_FALSE(world->isCompiled());
  {
    // Still invalid: the scope exit stores a fresh failure.
    auto edit = world->edit();
  }
  EXPECT_FALSE(world->isCompiled());
  EXPECT_THROW(world->step(), std::runtime_error);
  {
    auto edit = world->edit();
    shape->setShape(box);
  }
  EXPECT_TRUE(world->isCompiled());
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
TEST(World, ReentrantStepAndTopologyEditDuringStepThrow)
{
  auto world = World::create();
  auto ground = createBox(Eigen::Vector3d(10.0, 10.0, 1.0));
  ground->setMobile(false);
  auto box
      = createBox(Eigen::Vector3d::Ones(), Eigen::Vector3d(0.0, 0.0, 0.95));
  world->addSkeleton(ground);
  world->addSkeleton(box);
  auto handler = std::make_shared<StepCallbackHandler>();
  world->getConstraintSolver()->addContactSurfaceHandler(handler);
  int calls = 0;
  handler->mCallback = [&] {
    ++calls;
  };
  world->step();
  ASSERT_GT(calls, 0);

  const auto time = world->getTime();
  const auto bodies = box->getNumBodyNodes();
  handler->mCallback = [&] {
    world->step();
  };
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_EQ(world->getTime(), time);
  handler->mCallback = [&] {
    box->createJointAndBodyNodePair<RevoluteJoint>(box->getBodyNode(0));
  };
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_EQ(world->getTime(), time);
  EXPECT_EQ(box->getNumBodyNodes(), bodies);
  handler->mCallback = nullptr;
  EXPECT_NO_THROW(world->step());
  EXPECT_GT(world->getTime(), time);
}

//==============================================================================
TEST(World, TopologyEditFromPreparationFilterDuringStepThrows)
{
  auto world = World::create();
  auto ground = createBox(Eigen::Vector3d(10.0, 10.0, 1.0));
  ground->setMobile(false);
  auto box
      = createBox(Eigen::Vector3d::Ones(), Eigen::Vector3d(0.0, 0.0, 0.95));
  world->addSkeleton(ground);
  world->addSkeleton(box);
  auto filter = std::make_shared<CallbackCollisionFilter>();
  world->getConstraintSolver()->getCollisionOption().collisionFilter = filter;
  const auto bodies = box->getNumBodyNodes();
  int calls = 0;
  filter->mCallback = [&] {
    if (calls++ == 0)
      box->createJointAndBodyNodePair<RevoluteJoint>(box->getBodyNode(0));
  };
  // The first step prepares the solver, which queries the filter before any
  // solve stage runs.
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(box->getNumBodyNodes(), bodies);
  filter->mCallback = nullptr;
  EXPECT_NO_THROW(world->step());
  EXPECT_GT(world->getTime(), 0.0);
}

//==============================================================================
TEST(World, TopologyEditFromReplayedFilterDuringLazyStepThrows)
{
  auto world = World::create();
  auto ground = createBox(Eigen::Vector3d(10.0, 10.0, 1.0));
  ground->setMobile(false);
  auto box = createBox(
      Eigen::Vector3d::Ones(), Eigen::Vector3d(0.0, 0.0, 1.0 - 5e-7));
  world->addSkeleton(ground);
  world->addSkeleton(box);
  auto filter = std::make_shared<CallbackCollisionFilter>();
  world->getConstraintSolver()->getCollisionOption().collisionFilter = filter;
  for (int i = 0; i < 2000 && !box->isResting(); ++i)
    world->step();
  ASSERT_TRUE(box->isResting());
  ASSERT_TRUE(world->isInSimulationMode());
  ASSERT_FALSE(world->isCompiled());

  const auto time = world->getTime();
  const auto bodies = box->getNumBodyNodes();
  int calls = 0;
  filter->mCallback = [&] {
    if (calls++ == 0)
      box->createJointAndBodyNodePair<RevoluteJoint>(box->getBodyNode(0));
  };
  // Preparation is current: the first callback replays the previous solve's
  // filter decisions in the wake check, before either stepping path.
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(world->getTime(), time);
  EXPECT_EQ(box->getNumBodyNodes(), bodies);
  filter->mCallback = nullptr;
  EXPECT_NO_THROW(world->step());
  EXPECT_GT(world->getTime(), time);
  EXPECT_NO_THROW(
      box->createJointAndBodyNodePair<RevoluteJoint>(box->getBodyNode(0)));
}

//==============================================================================
namespace {
class CommandHookJoint : public FreeJoint
{
public:
  explicit CommandHookJoint(const Properties& properties)
    : FreeJoint(properties)
  {
  }

  double getCommand(std::size_t index) const override
  {
    if (mHook)
      mHook();
    return FreeJoint::getCommand(index);
  }

  std::function<void()> mHook;
};
} // namespace

//==============================================================================
TEST(World, TopologyEditFromJointOverrideOnCachedRestingStepThrows)
{
  auto world = World::create();
  auto ground = createBox(Eigen::Vector3d(10.0, 10.0, 1.0));
  ground->setMobile(false);
  world->addSkeleton(ground);
  auto box = Skeleton::create("hooked box");
  auto pair = box->createJointAndBodyNodePair<CommandHookJoint>();
  pair.second
      ->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
          std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0.0, 0.0, 1.0 - 5e-7);
  pair.first->setPositions(FreeJoint::convertToPositions(pose));
  world->addSkeleton(box);
  for (int i = 0; i < 2000 && !box->isResting(); ++i)
    world->step(false);
  ASSERT_TRUE(box->isResting());
  // The first contact-free step captures the snapshot but leaves it unready.
  for (int i = 0;
       i < 1000 && world->getLastCollisionResult().getNumContacts() > 0;
       ++i)
    world->step(false);
  ASSERT_EQ(world->getLastCollisionResult().getNumContacts(), 0u);
  ASSERT_TRUE(box->isResting());
  // Validate it without resetting, so step(true) takes the cached reset branch.
  world->step(false);
  ASSERT_TRUE(world->isInSimulationMode());
  ASSERT_FALSE(world->isCompiled());

  const auto time = world->getTime();
  int calls = 0;
  pair.first->mHook = [&] {
    if (calls++ == 0)
      box->createJointAndBodyNodePair<RevoluteJoint>(box->getBodyNode(0));
  };
  EXPECT_THROW(world->step(true), std::runtime_error);
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(world->getTime(), time);
  EXPECT_EQ(box->getNumBodyNodes(), 1u);
  pair.first->mHook = nullptr;
  EXPECT_NO_THROW(world->step(true));
  EXPECT_GT(world->getTime(), time);
}

//==============================================================================
TEST(World, TopologyEditFromFilterRevisionDuringLazyStepThrows)
{
  auto world = World::create();
  auto box = createBox(Eigen::Vector3d::Ones());
  world->addSkeleton(box);
  auto filter = std::make_shared<RevisionCallbackCollisionFilter>();
  world->getConstraintSolver()->getCollisionOption().collisionFilter = filter;
  world->step();
  ASSERT_TRUE(world->isInSimulationMode());
  ASSERT_FALSE(world->isCompiled());
  const auto bodies = box->getNumBodyNodes();
  const auto time = world->getTime();
  filter->mCallback = [&] {
    box->createJointAndBodyNodePair<RevoluteJoint>(box->getBodyNode(0));
  };
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_EQ(box->getNumBodyNodes(), bodies);
  EXPECT_EQ(world->getTime(), time);
  filter->mCallback = nullptr;
  EXPECT_NO_THROW(world->step());
  EXPECT_NO_THROW(
      box->createJointAndBodyNodePair<RevoluteJoint>(box->getBodyNode(0)));
}

//==============================================================================
TEST(World, LazyStepGuardsAncestorsAndSkeletonsOfOwnedFrames)
{
  auto world = World::create();
  DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);
  auto ground = createBox(Eigen::Vector3d(10.0, 10.0, 1.0));
  ground->setMobile(false);
  auto box
      = createBox(Eigen::Vector3d::Ones(), Eigen::Vector3d(0.0, 0.0, 0.95));
  world->addSkeleton(ground);
  world->addSkeleton(box);
  auto handler = std::make_shared<StepCallbackHandler>();
  world->getConstraintSolver()->addContactSurfaceHandler(handler);
  // An owned frame below an ancestor the World never added.
  auto parent = std::make_shared<SimpleFrame>(Frame::World(), "parent");
  auto child = std::make_shared<SimpleFrame>(parent.get(), "child");
  world->addSimpleFrame(child);
  auto other = std::make_shared<SimpleFrame>(Frame::World(), "other");
  // A Skeleton reached only through a collision frame added to the group.
  auto foreign
      = createBox(Eigen::Vector3d::Ones(), Eigen::Vector3d(5.0, 0.0, 0.0));
  world->getConstraintSolver()->getCollisionGroup()->addShapeFrame(
      foreign->getBodyNode(0)->getShapeNodeWith<CollisionAspect>(0));
  int calls = 0;
  handler->mCallback = [&] {
    ++calls;
  };
  world->step();
  ASSERT_GT(calls, 0);

  handler->mCallback = [&] {
    parent->setParentFrame(other.get());
  };
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_EQ(parent->getParentFrame(), Frame::World());
  handler->mCallback = [&] {
    other->setParentFrame(parent.get());
  };
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_EQ(other->getParentFrame(), Frame::World());
  const auto bodies = foreign->getNumBodyNodes();
  handler->mCallback = [&] {
    foreign->createJointAndBodyNodePair<RevoluteJoint>(foreign->getBodyNode(0));
  };
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_EQ(foreign->getNumBodyNodes(), bodies);
  // Unrelated models stay editable from the callback.
  auto unrelated = createBox(Eigen::Vector3d::Ones());
  handler->mCallback = [&] {
    if (unrelated->getNumBodyNodes() == 1u) {
      unrelated->createJointAndBodyNodePair<RevoluteJoint>(
          unrelated->getBodyNode(0));
    }
  };
  EXPECT_NO_THROW(world->step());
  EXPECT_EQ(unrelated->getNumBodyNodes(), 2u);
  handler->mCallback = nullptr;
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
namespace {

// Steps with `edit` run from `callback`: the first call must throw an error
// starting with `prefix` out of step() before time advances.
void expectStepRejectsEdit(
    World& world,
    std::function<void()>& callback,
    const std::function<void()>& edit,
    const std::string& prefix = "WorldBusyError: ")
{
  const auto time = world.getTime();
  int calls = 0;
  callback = [&] {
    ++calls;
    edit();
  };
  std::string error;
  try {
    world.step();
  } catch (const std::runtime_error& thrown) {
    error = thrown.what();
  }
  callback = nullptr;
  EXPECT_EQ(error.rfind(prefix, 0), 0u) << "error: '" << error << "'";
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(world.getTime(), time);
}

} // namespace

//==============================================================================
// Group edits from the stepping World's own filter or contact handler would
// free collision objects that the step still uses. Each is rejected before it
// changes the group, so the World keeps stepping (a compiled one without
// edit()).
TEST(World, GroupEditsFromStepCallbacksThrowBeforeChangingTheGroup)
{
  for (const bool compiled : {false, true}) {
    for (const bool dartDetector : {false, true}) {
      SCOPED_TRACE(compiled ? "compiled" : "lazy");
      SCOPED_TRACE(dartDetector ? "dart" : "fcl");
      auto world = World::create();
      if (dartDetector)
        world->setCollisionDetector(CollisionDetectorType::Dart);
      DeactivationOptions deactivation;
      deactivation.mEnabled = false;
      world->setDeactivationOptions(deactivation);
      auto ground = createBox(Eigen::Vector3d(10.0, 10.0, 1.0));
      ground->setMobile(false);
      auto box
          = createBox(Eigen::Vector3d::Ones(), Eigen::Vector3d(0.0, 0.0, 0.95));
      world->addSkeleton(ground);
      world->addSkeleton(box);
      auto* solver = world->getConstraintSolver();
      auto filter = std::make_shared<CallbackCollisionFilter>();
      solver->getCollisionOption().collisionFilter = filter;
      auto handler = std::make_shared<StepCallbackHandler>();
      solver->addContactSurfaceHandler(handler);
      const auto group = solver->getCollisionGroup();
      auto subscribed
          = createBox(Eigen::Vector3d::Ones(), Eigen::Vector3d(20.0, 0.0, 0.0));
      auto* subscribedBody = subscribed->getBodyNode(0);
      group->subscribeTo(ConstBodyNodePtr(subscribedBody));
      auto loose = createBox(
          Eigen::Vector3d::Ones(), Eigen::Vector3d(-20.0, 0.0, 0.0));
      auto* looseBody = loose->getBodyNode(0);
      auto frame = std::make_shared<SimpleFrame>(Frame::World(), "extra");
      frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
      auto* shape = box->getBodyNode(0)->getShapeNodeWith<CollisionAspect>(0);
      if (compiled)
        world->compile();
      ASSERT_NO_THROW(world->step());

      const auto frames = group->getNumShapeFrames();
      const auto version = group->getContentVersion();
      const bool checkLocks
          = !compiled && dart::simulation::detail::getIndexedWorldCount() == 0u;
      const auto locks = dart::simulation::detail::getRegistryLockCount();
      const std::vector<std::function<void()>> edits{
          [&] { group->addShapeFrame(frame.get()); },
          [&] { group->removeShapeFrame(shape); },
          [&] { group->removeAllShapeFrames(); },
          [&] { group->subscribeTo(loose); },
          [&] { group->subscribeTo(ConstBodyNodePtr(looseBody)); },
          [&] { group->unsubscribeFrom(box.get()); },
          [&] { group->unsubscribeFrom(subscribedBody); },
      };
      for (std::size_t i = 0u; i < edits.size(); ++i) {
        for (auto* callback : {&filter->mCallback, &handler->mCallback}) {
          SCOPED_TRACE(i);
          SCOPED_TRACE(callback == &filter->mCallback ? "filter" : "handler");
          const auto time = world->getTime();
          expectStepRejectsEdit(*world, *callback, edits[i]);
          EXPECT_EQ(group->getNumShapeFrames(), frames);
          EXPECT_EQ(group->getContentVersion(), version);
          EXPECT_TRUE(group->hasShapeFrame(shape));
          EXPECT_FALSE(group->hasShapeFrame(frame.get()));
          EXPECT_TRUE(group->isSubscribedTo(box.get()));
          EXPECT_TRUE(group->isSubscribedTo(subscribedBody));
          EXPECT_FALSE(group->isSubscribedTo(loose.get()));
          EXPECT_FALSE(group->isSubscribedTo(looseBody));
          EXPECT_NO_THROW(world->step());
          EXPECT_GT(world->getTime(), time);
        }
      }

      if (checkLocks) {
        EXPECT_EQ(dart::simulation::detail::getRegistryLockCount(), locks);
      }

      // Groups the stepping World does not use stay editable from callbacks.
      auto unrelated = world->getCollisionDetector()->createCollisionGroup();
      handler->mCallback = [&] {
        if (!unrelated->hasShapeFrame(frame.get()))
          unrelated->addShapeFrame(frame.get());
      };
      EXPECT_NO_THROW(world->step());
      handler->mCallback = nullptr;
      EXPECT_TRUE(unrelated->hasShapeFrame(frame.get()));
      EXPECT_EQ(world->isCompiled(), compiled);
    }
  }
}

//==============================================================================
namespace {

// Exposes the children of a composite filter.
class InspectableCompositeFilter final
  : public collision::CompositeCollisionFilter
{
public:
  bool hasChild(const collision::CollisionFilter* filter) const
  {
    return mFilters.count(filter) != 0u;
  }

  std::size_t getNumChildren() const
  {
    return mFilters.size();
  }

  void reserve(std::size_t count)
  {
    mFilters.reserve(count);
  }

  // Whether one more child rehashes the set, invalidating its iterators.
  bool nextInsertRehashes() const
  {
    return mFilters.size() + 1u
           > mFilters.max_load_factor() * mFilters.bucket_count();
  }
};

// Counts the queries that reach it and runs a callback from each.
class CallbackChildFilter final : public collision::CollisionFilter
{
public:
  bool ignoresCollision(
      const collision::CollisionObject*,
      const collision::CollisionObject*) const override
  {
    ++mQueries;
    if (mCallback)
      mCallback();
    return false;
  }

  std::function<void()> mCallback;
  mutable int mQueries = 0;
};

} // namespace

//==============================================================================
// A child edit that would invalidate a composite filter's running query, which
// DART 6 left undefined, throws before the children change. The composite
// keeps filtering in later steps.
TEST(World, CompositeFilterEditsThatInvalidateItsQueryThrowBeforeChangingIt)
{
  for (const bool compiled : {false, true}) {
    SCOPED_TRACE(compiled ? "compiled" : "lazy");
    auto world = World::create();
    DeactivationOptions deactivation;
    deactivation.mEnabled = false;
    world->setDeactivationOptions(deactivation);
    auto ground = createBox(Eigen::Vector3d(10.0, 10.0, 1.0));
    ground->setMobile(false);
    auto box
        = createBox(Eigen::Vector3d::Ones(), Eigen::Vector3d(0.0, 0.0, 0.95));
    world->addSkeleton(ground);
    world->addSkeleton(box);
    auto root = std::make_shared<InspectableCompositeFilter>();
    auto nested = std::make_shared<InspectableCompositeFilter>();
    auto child = std::make_shared<CallbackChildFilter>();
    auto other = std::make_shared<CallbackChildFilter>();
    nested->addCollisionFilter(child.get());
    root->addCollisionFilter(nested.get());
    // Fill the root up to its rehash point.
    std::vector<std::unique_ptr<CallbackChildFilter>> fillers;
    while (!root->nextInsertRehashes()) {
      fillers.push_back(std::make_unique<CallbackChildFilter>());
      root->addCollisionFilter(fillers.back().get());
    }
    const auto rootChildren = root->getNumChildren();
    world->getConstraintSolver()->getCollisionOption().collisionFilter = root;
    if (compiled)
      world->compile();
    ASSERT_NO_THROW(world->step());
    ASSERT_GT(child->mQueries, 0);

    const std::vector<std::function<void()>> edits{
        [&] { nested->removeCollisionFilter(child.get()); },
        [&] { nested->removeAllCollisionFilters(); },
        [&] { root->removeCollisionFilter(nested.get()); },
        [&] { root->addCollisionFilter(other.get()); },
        [&] { root->removeAllCollisionFilters(); },
    };
    for (std::size_t i = 0u; i < edits.size(); ++i) {
      SCOPED_TRACE(i);
      const auto time = world->getTime();
      expectStepRejectsEdit(
          *world, child->mCallback, edits[i], "CompositeCollisionFilter: ");
      EXPECT_EQ(root->getNumChildren(), rootChildren);
      EXPECT_TRUE(root->hasChild(nested.get()));
      EXPECT_EQ(nested->getNumChildren(), 1u);
      EXPECT_TRUE(nested->hasChild(child.get()));
      const auto queries = child->mQueries;
      EXPECT_NO_THROW(world->step());
      EXPECT_GT(world->getTime(), time);
      EXPECT_GT(child->mQueries, queries);
      EXPECT_EQ(other->mQueries, 0);
    }

    // Composites that are not iterating stay editable from a child.
    auto spare = std::make_shared<InspectableCompositeFilter>();
    child->mCallback = [&] {
      spare->addCollisionFilter(other.get());
    };
    EXPECT_NO_THROW(world->step());
    child->mCallback = nullptr;
    EXPECT_TRUE(spare->hasChild(other.get()));
    EXPECT_EQ(world->isCompiled(), compiled);
  }
}

//==============================================================================
// Child edits that keep a composite filter's running query valid stay allowed
// as in DART 6, from a direct collide() or a never-compiled World's step:
// removing another child, and inserting one without a rehash.
TEST(World, CompositeFilterAllowsChildEditsThatKeepItsQueryValid)
{
  for (const bool step : {false, true}) {
    SCOPED_TRACE(step ? "lazy World step" : "collide() without a World");
    auto world = World::create();
    DeactivationOptions deactivation;
    deactivation.mEnabled = false;
    world->setDeactivationOptions(deactivation);
    auto ground = createBox(Eigen::Vector3d(10.0, 10.0, 1.0));
    ground->setMobile(false);
    auto box
        = createBox(Eigen::Vector3d::Ones(), Eigen::Vector3d(0.0, 0.0, 0.95));
    auto composite = std::make_shared<InspectableCompositeFilter>();
    collision::CollisionOption option;
    option.collisionFilter = composite;
    auto group = world->getCollisionDetector()->createCollisionGroup(
        ground.get(), box.get());
    if (step) {
      world->addSkeleton(ground);
      world->addSkeleton(box);
      world->getConstraintSolver()->getCollisionOption().collisionFilter
          = composite;
    }
    const auto query = [&] {
      if (step)
        world->step();
      else
        group->collide(option);
    };

    composite->reserve(3u);
    CallbackChildFilter first;
    CallbackChildFilter second;
    CallbackChildFilter added;
    composite->addCollisionFilter(&first);
    composite->addCollisionFilter(&second);
    ASSERT_FALSE(composite->nextInsertRehashes());
    // Whichever child runs first removes the other and inserts a third.
    bool edited = false;
    const auto editOnce = [&](const CallbackChildFilter* other) {
      if (std::exchange(edited, true))
        return;
      composite->removeCollisionFilter(other);
      composite->addCollisionFilter(&added);
    };
    first.mCallback = [&] {
      editOnce(&second);
    };
    second.mCallback = [&] {
      editOnce(&first);
    };
    EXPECT_NO_THROW(query());
    EXPECT_TRUE(edited);
    EXPECT_EQ(composite->getNumChildren(), 2u);
    EXPECT_TRUE(composite->hasChild(&added));
    EXPECT_NE(composite->hasChild(&first), composite->hasChild(&second));

    // Removing the child being queried is rejected either way.
    added.mCallback = [&] {
      composite->removeCollisionFilter(&added);
    };
    EXPECT_THROW(query(), std::runtime_error);
    EXPECT_TRUE(composite->hasChild(&added));
  }
}

//==============================================================================
TEST(CollisionFilter, StandaloneCompositeEditsPreserveValidIterators)
{
  InspectableCompositeFilter composite;
  CallbackChildFilter first;
  CallbackChildFilter second;
  CallbackChildFilter added;
  composite.reserve(3u);
  composite.addCollisionFilter(&first);
  composite.addCollisionFilter(&second);
  bool edited = false;
  const auto editOnce = [&](const CallbackChildFilter* other) {
    if (std::exchange(edited, true))
      return;
    composite.removeCollisionFilter(other);
    composite.addCollisionFilter(&added);
  };
  first.mCallback = [&] {
    editOnce(&second);
  };
  second.mCallback = [&] {
    editOnce(&first);
  };
  EXPECT_NO_THROW(composite.ignoresCollision(nullptr, nullptr));
  EXPECT_TRUE(edited);
  EXPECT_EQ(composite.getNumChildren(), 2u);
  EXPECT_TRUE(composite.hasChild(&added));

  std::vector<std::unique_ptr<CallbackChildFilter>> fillers;
  while (!composite.nextInsertRehashes()) {
    fillers.push_back(std::make_unique<CallbackChildFilter>());
    composite.addCollisionFilter(fillers.back().get());
  }
  CallbackChildFilter rehash;
  const std::vector<std::function<void()>> edits{
      [&] { composite.removeCollisionFilter(&added); },
      [&] { composite.removeAllCollisionFilters(); },
      [&] {
        composite.addCollisionFilter(&rehash);
      }};
  const auto size = composite.getNumChildren();
  for (const auto& edit : edits) {
    added.mCallback = edit;
    try {
      composite.ignoresCollision(nullptr, nullptr);
      FAIL() << "invalidating edit accepted";
    } catch (const std::runtime_error& error) {
      EXPECT_EQ(
          std::string(error.what()).rfind("CompositeCollisionFilter: ", 0), 0u);
    }
    EXPECT_EQ(composite.getNumChildren(), size);
    EXPECT_TRUE(composite.hasChild(&added));
  }
  added.mCallback = nullptr;
  EXPECT_NO_THROW(composite.ignoresCollision(nullptr, nullptr));
}

//==============================================================================
TEST(World, ClearingEmptySubscriptionsPreservesContentVersion)
{
  auto world = World::create();
  auto robot = Skeleton::create();
  robot->createJointAndBodyNodePair<FreeJoint>();
  auto group = world->getConstraintSolver()->getCollisionGroup();
  group->subscribeTo(robot);
  const auto version = group->getContentVersion();
  group->removeAllShapeFrames();
  EXPECT_EQ(group->getContentVersion(), version);
  EXPECT_FALSE(group->isSubscribedTo(robot.get()));
  group->subscribeTo(robot);
  world->compile();
  group->removeAllShapeFrames();
  EXPECT_EQ(group->getContentVersion(), version);
  EXPECT_THROW(world->step(), std::runtime_error);
}

//==============================================================================
TEST(World, IndexAllocationFailureLeavesNoPartialMembership)
{
  using dart::constraint::ContactConstraint;
  using dart::simulation::detail::failIndexAllocation;
  using dart::simulation::detail::getIndexedWorldCount;
  struct ResetAllocationFailure
  {
    ~ResetAllocationFailure()
    {
      failIndexAllocation(0u);
    }
  } reset;
  auto first = World::create();
  auto robot = createBox(Eigen::Vector3d::Ones());
  first->addSkeleton(robot);
  auto ground = createBox(
      Eigen::Vector3d(10.0, 10.0, 1.0), Eigen::Vector3d(0.0, 0.0, -0.95));
  ground->setMobile(false);
  first->addSkeleton(ground);
  auto handler = std::make_shared<StepCallbackHandler>();
  first->getConstraintSolver()->addContactSurfaceHandler(handler);
  first->compile();
  const auto indexed = getIndexedWorldCount();
  auto second = World::create();
  auto box = createBox(Eigen::Vector3d::Ones());
  second->addSkeleton(box);

  // Failing before any membership is published.
  failIndexAllocation(1u);
  EXPECT_THROW(second->compile(), std::bad_alloc);
  EXPECT_FALSE(second->isCompiled());
  EXPECT_EQ(getIndexedWorldCount(), indexed);
  // Failing while the entries are registered, after the World joined.
  failIndexAllocation(3u);
  EXPECT_THROW(second->compile(), std::bad_alloc);
  EXPECT_FALSE(second->isCompiled());
  EXPECT_EQ(getIndexedWorldCount(), indexed);
  // The World stayed lazy: its model is neither frozen nor stamped.
  const auto revision = second->getTopologyRevision();
  EXPECT_NO_THROW(
      box->createJointAndBodyNodePair<RevoluteJoint>(box->getBodyNode(0)));
  EXPECT_EQ(second->getTopologyRevision(), revision);
  second.reset();
  // Global notifications visit every registered World, so a destroyed one
  // must have left the registry.
  const auto allowance = ContactConstraint::getErrorAllowance();
  ContactConstraint::setErrorAllowance(allowance + 0.1);
  ContactConstraint::setErrorAllowance(allowance);

  // A failed rebuild returns to lazy semantics, including callback guards.
  failIndexAllocation(2u);
  EXPECT_THROW(first->compile(), std::bad_alloc);
  EXPECT_FALSE(first->isCompiled());
  EXPECT_EQ(getIndexedWorldCount(), indexed - 1u);
  const auto bodies = robot->getNumBodyNodes();
  int calls = 0;
  handler->mCallback = [&] {
    if (calls++ == 0)
      robot->createJointAndBodyNodePair<RevoluteJoint>(robot->getBodyNode(0));
  };
  EXPECT_THROW(first->step(), std::runtime_error);
  EXPECT_GT(calls, 0);
  EXPECT_EQ(robot->getNumBodyNodes(), bodies);
  handler->mCallback = nullptr;
  EXPECT_NO_THROW(first->compile());
  EXPECT_TRUE(first->isCompiled());
  EXPECT_THROW(
      robot->createJointAndBodyNodePair<RevoluteJoint>(robot->getBodyNode(0)),
      std::runtime_error);
}

//==============================================================================
// Membership is indexed before it is published, so a failed index allocation
// leaves no half-added model for the scope-exit recompile to accept.
TEST(World, IndexAllocationFailureLeavesAdditionsOut)
{
  using dart::simulation::detail::failIndexAllocation;
  struct ResetAllocationFailure
  {
    ~ResetAllocationFailure()
    {
      failIndexAllocation(0u);
    }
  } reset;
  auto world = World::create();
  auto parent = std::make_shared<SimpleFrame>();
  auto child = std::make_shared<SimpleFrame>(parent.get());
  world->addSimpleFrame(parent);
  world->compile();
  auto robot = createBox(Eigen::Vector3d::Ones());
  auto pose = Eigen::Isometry3d::Identity();
  pose.translation().x() = 0.2;
  {
    auto edit = world->edit();
    failIndexAllocation(1u);
    EXPECT_THROW(world->addSkeleton(robot), std::bad_alloc);
    EXPECT_EQ(world->getNumSkeletons(), 0u);
    // The child's second key, its parent, fails after its first was indexed.
    failIndexAllocation(2u);
    EXPECT_THROW(world->addSimpleFrame(child), std::bad_alloc);
    EXPECT_EQ(world->getNumSimpleFrames(), 1u);
    // Detached, the child no longer belongs to the World in any way.
    child->setParentFrame(Frame::World());
    const auto parameters = world->getParameterRevision();
    child->setRelativeTransform(pose);
    EXPECT_EQ(world->getParameterRevision(), parameters);
  }
  EXPECT_TRUE(world->isCompiled());
  {
    auto edit = world->edit();
    world->addSkeleton(robot);
    world->addSimpleFrame(child);
  }
  EXPECT_TRUE(world->isCompiled());
  EXPECT_EQ(world->getNumSkeletons(), 1u);
  EXPECT_EQ(world->getConstraintSolver()->getSkeletons().size(), 1u);
  EXPECT_EQ(world->getNumSimpleFrames(), 2u);
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
// compile() runs user extension points, so their callbacks are held to the
// rules of a step's callbacks instead of reentering the compiling World.
TEST(World, CompileCallbacksCannotReenterTheWorld)
{
  auto world = World::create();
  auto ground = createBox(
      Eigen::Vector3d(10.0, 10.0, 1.0), Eigen::Vector3d(0.0, 0.0, -0.5));
  ground->setMobile(false);
  auto robot
      = createBox(Eigen::Vector3d::Ones(), Eigen::Vector3d(0.0, 0.0, 0.45));
  world->addSkeleton(ground);
  world->addSkeleton(robot);
  auto filter = std::make_shared<CallbackCollisionFilter>();
  world->getConstraintSolver()->getCollisionOption().collisionFilter = filter;
  std::vector<std::string> errors;
  const auto attempt = [&](const auto& call) {
    try {
      call();
      errors.emplace_back("accepted");
    } catch (const std::runtime_error& error) {
      errors.emplace_back(error.what());
    }
  };
  bool ran = false;
  filter->mCallback = [&] {
    if (std::exchange(ran, true))
      return;
    attempt([&] { world->compile(); });
    attempt([&] { auto edit = world->edit(); });
    attempt([&] { world->step(); });
    attempt([&] {
      robot->createJointAndBodyNodePair<RevoluteJoint>(robot->getBodyNode(0));
    });
  };
  EXPECT_NO_THROW(world->compile());
  EXPECT_TRUE(ran);
  ASSERT_EQ(errors.size(), 4u);
  for (const auto& error : errors)
    EXPECT_EQ(error.rfind("WorldBusyError: ", 0), 0u) << error;
  EXPECT_EQ(robot->getNumBodyNodes(), 1u);
  EXPECT_TRUE(world->isCompiled());
  filter->mCallback = nullptr;
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
// A recompile rebuilds its validation snapshots in place, so a failure there
// leaves the World uncompiled instead of frozen on partial snapshots.
TEST(World, FailedSnapshotRebuildLeavesWorldUncompiled)
{
  auto world = World::create();
  world->addSkeleton(createBox(Eigen::Vector3d::Ones()));
  auto filter = std::make_shared<RevisionCallbackCollisionFilter>();
  world->getConstraintSolver()->getCollisionOption().collisionFilter = filter;
  world->compile();
  // A recompile's last revision read records the rebuilt snapshot.
  int reads = 0;
  filter->mCallback = [&] {
    ++reads;
  };
  world->compile();
  const int last = reads;
  ASSERT_GT(last, 0);
  reads = 0;
  filter->mCallback = [&] {
    if (++reads == last)
      throw std::bad_alloc();
  };
  EXPECT_THROW(world->compile(), std::bad_alloc);
  EXPECT_EQ(reads, last);
  EXPECT_FALSE(world->isCompiled());
  filter->mCallback = nullptr;
  EXPECT_NO_THROW(world->compile());
  EXPECT_TRUE(world->isCompiled());
}

//==============================================================================
TEST(World, FailedSubscriptionPreservesExistingOwnership)
{
  auto world = World::create();
  auto robot = Skeleton::create();
  auto* first = robot->createJointAndBodyNodePair<FreeJoint>().second;
  auto* second = robot->createJointAndBodyNodePair<FreeJoint>().second;
  const auto group = world->getConstraintSolver()->getCollisionGroup();
  group->subscribeTo(ConstBodyNodePtr(first));
  auto view = Group::create();
  view->addBodyNode(first);
  view->addBodyNode(second);
  world->compile();
  {
    auto edit = world->edit();
    // Fail after registering the first body of the borrowed view.
    dart::simulation::detail::failIndexAllocation(2u);
    EXPECT_THROW(group->subscribeTo(view), std::bad_alloc);
    dart::simulation::detail::failIndexAllocation(0u);
    EXPECT_FALSE(group->isSubscribedTo(view.get()));
    // The failed view must not remove the pre-existing BodyNode claim.
    const auto revision = world->getParameterRevision();
    first->setCollidable(false);
    EXPECT_GT(world->getParameterRevision(), revision);
  }
  EXPECT_TRUE(world->isCompiled());
  EXPECT_THROW(
      robot->createJointAndBodyNodePair<FreeJoint>(), std::runtime_error);
  EXPECT_NO_THROW(world->step());
}

//==============================================================================
TEST(World, RechainSkipsRejoinedWorldAtSameAddress)
{
  using dart::simulation::detail::failIndexAllocation;
  using dart::simulation::detail::setRechainUnlockedHook;
  struct ResetHooks
  {
    ~ResetHooks()
    {
      failIndexAllocation(0u);
      setRechainUnlockedHook({});
    }
  } reset;
  auto world = World::create();
  auto ancestor = std::make_shared<SimpleFrame>();
  auto member = std::make_shared<SimpleFrame>(ancestor.get());
  auto staleParent = std::make_shared<SimpleFrame>();
  auto finalParent = std::make_shared<SimpleFrame>();
  world->addSimpleFrame(member);
  world->compile();
  // An unwinding edit leaves an indexed, editable World without recompiling.
  try {
    auto edit = world->edit();
    throw std::runtime_error("leave editable");
  } catch (const std::runtime_error&) {
  }
  setRechainUnlockedHook([&] {
    failIndexAllocation(2u);
    EXPECT_THROW(world->compile(), std::bad_alloc);
    ancestor->setParentFrame(finalParent.get());
    world->compile();
  });
  ancestor->setParentFrame(staleParent.get());
  ASSERT_TRUE(world->isCompiled());
  ASSERT_EQ(ancestor->getParentFrame(), finalParent.get());
  auto parameter = world->getParameterRevision();
  staleParent->setRelativeTranslation(Eigen::Vector3d::UnitX());
  EXPECT_EQ(world->getParameterRevision(), parameter);
  finalParent->setRelativeTranslation(Eigen::Vector3d::UnitY());
  EXPECT_GT(world->getParameterRevision(), parameter);
}

//==============================================================================
TEST(WorldDeathTest, LazySolverCopyLeavesSharedHandlerParentsUnread)
{
  // With no World compiled, copying a solver reads only what DART 6 read, so
  // another thread may reparent a handler in the copied chain (checked by
  // TSan builds). Other tests keep compiled Worlds alive, so this runs in a
  // re-executed process where none exists.
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
      {
        using dart::constraint::ContactSurfaceHandler;
        const bool lazyOnly
            = !dart::constraint::detail::contactSurfaceChecksActive();
        auto source = World::create();
        auto target = World::create();
        auto shared = std::make_shared<ContactSurfaceHandler>();
        auto firstParent = std::make_shared<ContactSurfaceHandler>();
        auto secondParent = std::make_shared<ContactSurfaceHandler>();
        source->getConstraintSolver()->addContactSurfaceHandler(shared);
        std::atomic<bool> start{false};
        std::thread reparent([&] {
          while (!start.load(std::memory_order_acquire)) {
          }
          for (int i = 0; i < 200; ++i) {
            shared->setParent(firstParent);
            shared->setParent(secondParent);
          }
        });
        start.store(true, std::memory_order_release);
        for (int i = 0; i < 200; ++i) {
          target->getConstraintSolver()->setFromOtherConstraintSolver(
              *source->getConstraintSolver());
        }
        reparent.join();
        // std::exit runs ThreadSanitizer's exit hook, which turns a reported
        // race into a failing exit code; std::_Exit would skip it.
        std::exit(
            lazyOnly && !target->isCompiled()
                    && shared->getParent() == secondParent
                ? 0
                : 1);
      },
      ::testing::ExitedWithCode(0),
      "");
}

//==============================================================================
TEST(World, SolverCopyRefreshesKeysWithOneTopologyStamp)
{
  auto world = World::create();
  world->compile();
  try {
    auto edit = world->edit();
    throw std::runtime_error("leave editable");
  } catch (const std::runtime_error&) {
  }
  auto source = World::create();
  const auto revision = world->getTopologyRevision();
  world->getConstraintSolver()->setFromOtherConstraintSolver(
      *source->getConstraintSolver());
  EXPECT_EQ(world->getTopologyRevision(), revision + 1u);
  const auto handler
      = source->getConstraintSolver()->getLastContactSurfaceHandler();
  const auto parameter = world->getTopologyRevision();
  handler->setParent(std::make_shared<constraint::ContactSurfaceHandler>());
  EXPECT_EQ(world->getTopologyRevision(), parameter + 1u);
}

//==============================================================================
TEST(World, OwnerLookupCoversMoreIndexedWorldsThanTheInlineScratch)
{
  // Indexed Worlds cannot share a Skeleton, but they can share a detector,
  // which lists every one of them under a single key.
  constexpr std::size_t kWorlds = 10u;
  auto detector = collision::DARTCollisionDetector::create();
  std::vector<WorldPtr> worlds;
  for (std::size_t i = 0u; i < kWorlds; ++i) {
    auto world = World::create();
    world->getConstraintSolver()->setCollisionDetector(detector);
    world->addSkeleton(
        createBox(Eigen::Vector3d::Ones(), Eigen::Vector3d(3.0 * i, 0.0, 0.0)));
    world->compile();
    worlds.push_back(world);
  }
  std::vector<std::size_t> parameters;
  for (const auto& world : worlds)
    parameters.push_back(world->getParameterRevision());
  detector->setSoftFaceInteriorContactsEnabled(
      !detector->getSoftFaceInteriorContactsEnabled());
  for (std::size_t i = 0u; i < kWorlds; ++i) {
    EXPECT_EQ(worlds[i]->getParameterRevision(), parameters[i] + 1u) << i;
    EXPECT_TRUE(worlds[i]->isCompiled()) << i;
  }
}

//==============================================================================
TEST(World, SlipSetterCostDoesNotScaleWithIdleWorlds)
{
  auto world = World::create();
  std::vector<DynamicsAspect*> wheels;
  for (int i = 0; i < 4; ++i) {
    auto wheel = createBox(
        Eigen::Vector3d::Ones(), Eigen::Vector3d(3.0 * i, 0.0, 0.0));
    world->addSkeleton(wheel);
    wheels.push_back(wheel->getBodyNode(0)
                         ->getShapeNodeWith<CollisionAspect>(0)
                         ->getDynamicsAspect());
  }
  const auto measure = [&] {
    double best = std::numeric_limits<double>::infinity();
    for (int repeat = 0; repeat < 5; ++repeat) {
      const auto start = std::chrono::steady_clock::now();
      for (int i = 0; i < 2000; ++i) {
        for (auto* wheel : wheels)
          wheel->setPrimarySlipCompliance(1e-6 * ((i & 1) + 1));
      }
      best = std::min(
          best,
          std::chrono::duration<double, std::nano>(
              std::chrono::steady_clock::now() - start)
              .count());
    }
    return best;
  };
  const double alone = measure();
  std::vector<WorldPtr> idle;
  for (int i = 0; i < 256; ++i) {
    idle.push_back(World::create());
    idle.back()->addSkeleton(createBox(Eigen::Vector3d::Ones()));
  }
  const double crowded = measure();
  EXPECT_LT(crowded, 8.0 * alone + 1.0e6)
      << "alone " << alone << " ns, with 256 idle Worlds " << crowded << " ns";
}

//==============================================================================
TEST(World, RawDetectorRejectionDoesNotLeak)
{
  auto world = World::create();
  world->addSkeleton(createBox(Eigen::Vector3d::Ones()));
  world->compile();
  int destroyed = 0;
  auto* detector = new CountingFclDetector(destroyed);
  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_THROW(
      world->getConstraintSolver()->setCollisionDetector(detector),
      std::runtime_error);
  DART_SUPPRESS_DEPRECATED_END
  EXPECT_EQ(destroyed, 1);
}

//==============================================================================
TEST(World, FailedSourceSyncRetriesWithoutAnotherMutation)
{
  auto world = World::create();
  auto view = Group::create();
  auto robot = createBox(Eigen::Vector3d::Ones());
  auto other = createBox(Eigen::Vector3d::Ones());
  world->addSkeleton(robot);
  world->addSkeleton(other);
  auto group = world->getConstraintSolver()->getCollisionGroup();
  // Keep the source edit responsible for introducing the robot's shape.
  group->removeAllShapeFrames();
  group->subscribeTo(view);
  group->addShapeFramesOf(other.get());
  world->compile();
  ASSERT_EQ(group->getNumShapeFrames(), 1u);
  ASSERT_FALSE(group->hasShapeFrame(robot->getBodyNode(0)->getShapeNode(0)));
  {
    auto edit = world->edit();
    struct ResetFailure
    {
      ~ResetFailure()
      {
        dart::simulation::detail::failIndexAllocation(0u);
      }
    } reset;
    view->addBodyNode(robot->getBodyNode(0));
    dart::simulation::detail::failIndexAllocation(1u);
    EXPECT_THROW(group->update(), std::bad_alloc);
    dart::simulation::detail::failIndexAllocation(0u);
    ASSERT_NO_THROW(group->update());
    EXPECT_EQ(group->getNumShapeFrames(), 2u);
    EXPECT_TRUE(group->hasShapeFrame(robot->getBodyNode(0)->getShapeNode(0)));
    EXPECT_TRUE(group->collide());
  }
  EXPECT_NO_THROW(world->step());
}

namespace {
class SubscriptionReentryDetector final : public collision::FCLCollisionDetector
{
public:
  std::function<void()> mCallback;
  std::unique_ptr<collision::CollisionObject> createCollisionObject(
      const ShapeFrame* frame) override
  {
    if (mCallback)
      mCallback();
    return FCLCollisionDetector::createCollisionObject(frame);
  }
};
} // namespace

//==============================================================================
TEST(World, NestedGroupUpdateFromDetectorIsRejected)
{
  for (const bool compiled : {false, true}) {
    for (int query = 0; query < 4; ++query) {
      SCOPED_TRACE(compiled);
      SCOPED_TRACE(query);
      auto world = World::create();
      auto detector = std::make_shared<SubscriptionReentryDetector>();
      world->setCollisionDetector(detector);
      auto group = world->getConstraintSolver()->getCollisionGroup();
      auto view = Group::create();
      group->subscribeTo(view);
      auto robot = createBox(Eigen::Vector3d::Ones());
      view->addBodyNode(robot->getBodyNode(0));
      bool entered = false;
      bool rejected = false;
      detector->mCallback = [&] {
        if (std::exchange(entered, true))
          return;
        try {
          if (query == 0)
            group->update();
          else if (query == 1)
            group->collide();
          else if (query == 2)
            group->distance();
          else
            group->raycast(Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());
        } catch (const std::runtime_error& error) {
          rejected = std::string(error.what()).find("WorldBusyError") == 0u;
        }
      };
      if (compiled)
        EXPECT_NO_THROW(world->compile());
      else
        EXPECT_NO_THROW(world->step());
      detector->mCallback = nullptr;
      EXPECT_TRUE(entered);
      EXPECT_TRUE(rejected);
      EXPECT_NO_THROW(group->update());
      EXPECT_EQ(group->getNumShapeFrames(), 1u);
      EXPECT_NO_THROW(world->step());
    }
  }
}

//==============================================================================
TEST(World, DifferentGroupUpdatesNestButRejectCycles)
{
  auto firstDetector = std::make_shared<SubscriptionReentryDetector>();
  auto secondDetector = std::make_shared<SubscriptionReentryDetector>();
  auto first = firstDetector->createCollisionGroup();
  auto second = secondDetector->createCollisionGroup();
  auto firstView = Group::create();
  auto secondView = Group::create();
  first->subscribeTo(firstView);
  second->subscribeTo(secondView);
  auto firstBody = createBox(Eigen::Vector3d::Ones());
  auto secondBody = createBox(Eigen::Vector3d::Ones());
  firstView->addBodyNode(firstBody->getBodyNode(0));
  secondView->addBodyNode(secondBody->getBodyNode(0));
  bool nested = false;
  secondDetector->mCallback = [&] {
    nested = true;
    EXPECT_THROW(first->update(), std::runtime_error);
  };
  firstDetector->mCallback = [&] {
    EXPECT_NO_THROW(second->update());
  };
  EXPECT_NO_THROW(first->update());
  EXPECT_TRUE(nested);
  firstDetector->mCallback = nullptr;
  secondDetector->mCallback = nullptr;
  EXPECT_EQ(first->getNumShapeFrames(), 1u);
  EXPECT_EQ(second->getNumShapeFrames(), 1u);
  EXPECT_NO_THROW(first->update());
  EXPECT_NO_THROW(second->update());
}

//==============================================================================
TEST(World, SourceSyncFailureSweep)
{
  for (int sourceKind = 0; sourceKind < 3; ++sourceKind) {
    bool completed = false;
    for (std::size_t nth = 1; nth < 64; ++nth) {
      SCOPED_TRACE(sourceKind);
      SCOPED_TRACE(nth);
      auto world = World::create();
      auto robot = Skeleton::create();
      auto* body = robot->createJointAndBodyNodePair<FreeJoint>().second;
      auto view = Group::create();
      auto group = world->getConstraintSolver()->getCollisionGroup();
      if (sourceKind == 0)
        group->subscribeTo(view);
      else if (sourceKind == 1)
        group->subscribeTo(robot);
      else
        group->subscribeTo(ConstBodyNodePtr(body));
      world->compile();
      bool failed = false;
      {
        auto edit = world->edit();
        struct ResetFailure
        {
          ~ResetFailure()
          {
            dart::simulation::detail::failIndexAllocation(0u);
          }
        } reset;
        body->createShapeNodeWith<CollisionAspect>(
            std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
        body->createShapeNodeWith<CollisionAspect>(
            std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
        if (sourceKind == 0)
          view->addBodyNode(body);
        dart::simulation::detail::failIndexAllocation(nth);
        try {
          group->update();
        } catch (const std::bad_alloc&) {
          failed = true;
        }
        dart::simulation::detail::failIndexAllocation(0u);
        ASSERT_NO_THROW(group->update());
        ASSERT_EQ(group->getNumShapeFrames(), 2u);
        EXPECT_TRUE(group->hasShapeFrame(body->getShapeNode(0)));
        EXPECT_TRUE(group->hasShapeFrame(body->getShapeNode(1)));
        EXPECT_NO_THROW(group->collide());
      }
      EXPECT_NO_THROW(world->step());
      EXPECT_THROW(
          robot->createJointAndBodyNodePair<FreeJoint>(), std::runtime_error);
      {
        auto edit = world->edit();
        group->removeAllShapeFrames();
      }
      EXPECT_NO_THROW(robot->createJointAndBodyNodePair<FreeJoint>());
      if (!failed) {
        completed = true;
        break;
      }
    }
    EXPECT_TRUE(completed);
  }
}

//==============================================================================
TEST(World, SubscribedViewChangesDuringStepAreRejected)
{
  for (const bool compiled : {false, true}) {
    for (const bool native : {false, true}) {
      auto world = World::create();
      if (native)
        world->setCollisionDetector(CollisionDetectorType::Dart);
      DeactivationOptions options;
      options.mEnabled = false;
      world->setDeactivationOptions(options);
      auto ground = createBox(Eigen::Vector3d(10, 10, 1));
      ground->setMobile(false);
      world->addSkeleton(ground);
      world->addSkeleton(
          createBox(Eigen::Vector3d::Ones(), Eigen::Vector3d(0, 0, 0.95)));
      auto loose
          = createBox(Eigen::Vector3d::Ones(), Eigen::Vector3d(20, 0, 0));
      auto shapeless = Skeleton::create();
      auto* emptyBody
          = shapeless->createJointAndBodyNodePair<FreeJoint>().second;
      auto view = Group::create();
      auto group = world->getConstraintSolver()->getCollisionGroup();
      group->subscribeTo(view);
      auto filter = std::make_shared<CallbackCollisionFilter>();
      world->getConstraintSolver()->getCollisionOption().collisionFilter
          = filter;
      auto handler = std::make_shared<StepCallbackHandler>();
      world->getConstraintSolver()->addContactSurfaceHandler(handler);
      if (compiled)
        world->compile();
      ASSERT_NO_THROW(world->step());
      for (auto* body : {loose->getBodyNode(0), emptyBody}) {
        for (auto* callback : {&filter->mCallback, &handler->mCallback}) {
          const auto count = group->getNumShapeFrames();
          const auto version = group->getContentVersion();
          expectStepRejectsEdit(*world, *callback, [&] {
            view->addBodyNode(body);
            group->update();
          });
          EXPECT_EQ(group->getNumShapeFrames(), count);
          EXPECT_EQ(group->getContentVersion(), version);
          view->removeBodyNode(body);
          EXPECT_NO_THROW(world->step());
          {
            auto edit = world->edit();
            view->addBodyNode(body);
            group->update();
          }
          const auto addedCount = group->getNumShapeFrames();
          const auto addedVersion = group->getContentVersion();
          expectStepRejectsEdit(*world, *callback, [&] {
            view->removeBodyNode(body);
            group->update();
          });
          EXPECT_EQ(group->getNumShapeFrames(), addedCount);
          EXPECT_EQ(group->getContentVersion(), addedVersion);
          view->addBodyNode(body);
          EXPECT_NO_THROW(world->step());
          {
            auto edit = world->edit();
            view->removeBodyNode(body);
            group->update();
          }
        }
      }
      handler->mCallback = [&] {
        group->update();
      };
      EXPECT_NO_THROW(world->step());
      handler->mCallback = nullptr;
    }
  }
}

namespace {
class SharedSubscriptionSolver final
  : public dart::constraint::BoxedLcpConstraintSolver
{
public:
  explicit SharedSubscriptionSolver(const collision::CollisionGroupPtr& group)
  {
    mCollisionGroup = group;
    mCollisionDetector = group->getCollisionDetector();
  }
  void setFromOtherConstraintSolver(
      const dart::constraint::ConstraintSolver&) override
  {
  }
};
} // namespace

//==============================================================================
TEST(World, SourceBodyAddedRollbackAcrossOwners)
{
  for (const bool preexisting : {false, true}) {
    bool completed = false;
    for (std::size_t nth = 1; nth < 64; ++nth) {
      SCOPED_TRACE(preexisting);
      SCOPED_TRACE(nth);
      auto first = World::create();
      auto second = World::create();
      auto group = first->getConstraintSolver()->getCollisionGroup();
      second->setConstraintSolver(
          std::make_unique<SharedSubscriptionSolver>(group));
      ASSERT_EQ(second->getConstraintSolver()->getCollisionGroup(), group);
      auto view = Group::create();
      group->subscribeTo(view);
      first->compile();
      second->compile();
      auto robot = createBox(Eigen::Vector3d::Ones());
      auto* body = robot->getBodyNode(0);
      auto* prior = robot->createJointAndBodyNodePair<FreeJoint>().second;
      bool failed = false;
      {
        auto firstEdit = first->edit();
        auto secondEdit = second->edit();
        struct ResetFailure
        {
          ~ResetFailure()
          {
            dart::simulation::detail::failIndexAllocation(0u);
          }
        } reset;
        group->subscribeTo(ConstBodyNodePtr(prior));
        view->addBodyNode(body);
        if (preexisting)
          first->setConstraintSolver(
              std::make_unique<SharedSubscriptionSolver>(group));
        dart::simulation::detail::failIndexAllocation(nth);
        try {
          group->update();
        } catch (const std::bad_alloc&) {
          failed = true;
        }
        dart::simulation::detail::failIndexAllocation(0u);
        const auto firstVersion = first->getParameterRevision();
        const auto secondVersion = second->getParameterRevision();
        prior->setCollidable(false);
        EXPECT_GT(first->getParameterRevision(), firstVersion);
        EXPECT_GT(second->getParameterRevision(), secondVersion);
        EXPECT_NO_THROW(group->update());
        EXPECT_EQ(group->getNumShapeFrames(), 1u);
        view->removeBodyNode(body);
        EXPECT_NO_THROW(group->update());
        EXPECT_EQ(group->getNumShapeFrames(), 0u);
        // Compilation rejects shared model ownership. Detach the second owner
        // after checking both claims so the first can freeze its retained one.
        second->setConstraintSolver(
            std::make_unique<dart::constraint::BoxedLcpConstraintSolver>());
      }
      EXPECT_TRUE(first->isCompiled());
      EXPECT_TRUE(second->isCompiled());
      EXPECT_THROW(
          robot->createJointAndBodyNodePair<FreeJoint>(), std::runtime_error);
      {
        auto firstEdit = first->edit();
        auto secondEdit = second->edit();
        group->unsubscribeFrom(prior);
      }
      EXPECT_NO_THROW(robot->createJointAndBodyNodePair<FreeJoint>());
      EXPECT_NO_THROW(first->step());
      EXPECT_NO_THROW(second->step());
      if (!failed) {
        completed = true;
        break;
      }
    }
    EXPECT_TRUE(completed);
  }
}

namespace {
std::shared_ptr<dart::math::TriMeshd> subscriptionTetrahedron(double size)
{
  auto mesh = std::make_shared<dart::math::TriMeshd>();
  mesh->addVertex(0, 0, 0);
  mesh->addVertex(size, 0, 0);
  mesh->addVertex(0, size, 0);
  mesh->addVertex(0, 0, size);
  mesh->addTriangle(0, 2, 1);
  mesh->addTriangle(0, 1, 3);
  mesh->addTriangle(0, 3, 2);
  mesh->addTriangle(1, 2, 3);
  return mesh;
}
} // namespace

//==============================================================================
TEST(World, FrozenGeometryStorageIsUnchanged)
{
  auto world = World::create();
  auto frame = std::make_shared<SimpleFrame>();
  auto convex = std::make_shared<ConvexMeshShape>(subscriptionTetrahedron(1));
  frame->setShape(convex);
  world->addSimpleFrame(frame);
  world->compile();
  const auto topology = world->getTopologyRevision();
  const auto parameters = world->getParameterRevision();
  const auto version = convex->getVersion();
  const auto frameVersion = frame->getVersion();
  const auto storage = convex->getMesh();
  int calls = 0;
  auto connection
      = convex->onVersionChanged.connect([&](Shape*, std::size_t) { ++calls; });
  EXPECT_THROW(convex->setMesh(subscriptionTetrahedron(2)), std::runtime_error);
  EXPECT_EQ(convex->getMesh(), storage);
  EXPECT_EQ(convex->getVersion(), version);
  EXPECT_EQ(frame->getVersion(), frameVersion);
  EXPECT_EQ(world->getTopologyRevision(), topology);
  EXPECT_EQ(world->getParameterRevision(), parameters);
  EXPECT_EQ(calls, 0);
  {
    auto edit = world->edit();
    convex->setMesh(subscriptionTetrahedron(2));
    EXPECT_GT(world->getTopologyRevision(), topology);
    EXPECT_EQ(world->getParameterRevision(), parameters);
  }
  EXPECT_DOUBLE_EQ(convex->getBoundingBox().getMax()[0], 2);
  EXPECT_TRUE(world->isCompiled());
  EXPECT_NO_THROW(world->step());

  DART_SUPPRESS_DEPRECATED_BEGIN
  auto scene = std::make_shared<aiScene>();
  auto replacement = std::make_shared<aiScene>();
  auto mesh = std::shared_ptr<MeshShape>(new MeshShape(
      Eigen::Vector3d::Ones(), std::shared_ptr<const aiScene>(scene)));
  {
    auto edit = world->edit();
    frame->setShape(mesh);
  }
  const auto meshVersion = mesh->getVersion();
  const auto meshTopology = world->getTopologyRevision();
  const auto meshParameters = world->getParameterRevision();
  for (const auto ownership :
       {MeshShape::MeshOwnership::None,
        MeshShape::MeshOwnership::Imported,
        MeshShape::MeshOwnership::Copied,
        MeshShape::MeshOwnership::Manual,
        MeshShape::MeshOwnership::Custom}) {
    EXPECT_THROW(
        mesh->setMesh(replacement.get(), ownership, dart::common::Uri()),
        std::runtime_error);
    EXPECT_EQ(mesh->getMesh(), scene.get());
    EXPECT_EQ(mesh->getVersion(), meshVersion);
  }
  const std::vector<std::function<void()>> mutations{
      [&] {
        mesh->setMesh(
            replacement.get(),
            MeshShape::MeshOwnership::None,
            dart::common::Uri());
      },
      [&] { mesh->setMesh(std::shared_ptr<const aiScene>(replacement)); },
      [&] { mesh->setMesh(nullptr, std::string()); },
      [&] {
        mesh->setMesh(nullptr, dart::common::Uri());
      }};
  for (const auto& mutate : mutations) {
    EXPECT_THROW(mutate(), std::runtime_error);
    EXPECT_EQ(mesh->getMesh(), scene.get());
    EXPECT_EQ(mesh->getVersion(), meshVersion);
    EXPECT_EQ(world->getTopologyRevision(), meshTopology);
    EXPECT_EQ(world->getParameterRevision(), meshParameters);
  }
  EXPECT_NO_THROW(mesh->setMesh(std::shared_ptr<const aiScene>(scene)));
  EXPECT_EQ(mesh->getVersion(), meshVersion);
  EXPECT_NO_THROW(mesh->setMesh(
      std::shared_ptr<const aiScene>(scene), dart::common::Uri("metadata")));
  EXPECT_EQ(world->getTopologyRevision(), meshTopology);
  EXPECT_GT(world->getParameterRevision(), meshParameters);
  DART_SUPPRESS_DEPRECATED_END
}

#if HAVE_OCTOMAP
//==============================================================================
TEST(World, FrozenVoxelOverloadsAndEditedCollision)
{
  auto world = World::create();
  auto voxels = std::make_shared<VoxelGridShape>(0.1);
  auto frame = std::make_shared<SimpleFrame>();
  frame->setShape(voxels);
  world->addSimpleFrame(frame);
  auto group = world->getConstraintSolver()->getCollisionGroup();
  auto box = std::make_shared<SimpleFrame>();
  box->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.05)));
  box->setRelativeTranslation(Eigen::Vector3d::Constant(0.05));
  voxels->updateOccupancy(Eigen::Vector3d::Constant(3));
  group->addShapeFramesOf(frame.get(), box.get());
  world->compile();
  EXPECT_FALSE(group->collide());
  const auto storage = voxels->getOctree();
  const auto size = storage->size();
  const auto version = voxels->getVersion();
  const auto topology = world->getTopologyRevision();
  const auto parameters = world->getParameterRevision();
  octomap::Pointcloud points;
  points.push_back(0.05f, 0.05f, 0.05f);
  SimpleFrame relative;
  const std::vector<std::function<void()>> mutations{
      [&] { voxels->setOctree(std::make_shared<octomap::OcTree>(0.2)); },
      [&] { voxels->updateOccupancy(Eigen::Vector3d::Constant(0.05), true); },
      [&] {
        voxels->updateOccupancy(
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(0.05));
      },
      [&] {
        voxels->updateOccupancy(
            points, Eigen::Vector3d::Zero(), Frame::World());
      },
      [&] {
        voxels->updateOccupancy(points, Eigen::Vector3d::Zero(), &relative);
      },
      [&] {
        voxels->updateOccupancy(
            points, Eigen::Vector3d::Zero(), Eigen::Isometry3d::Identity());
      }};
  for (const auto& mutate : mutations) {
    EXPECT_THROW(mutate(), std::runtime_error);
    EXPECT_EQ(voxels->getOctree(), storage);
    EXPECT_EQ(storage->size(), size);
    EXPECT_EQ(voxels->getVersion(), version);
    EXPECT_EQ(world->getTopologyRevision(), topology);
    EXPECT_EQ(world->getParameterRevision(), parameters);
  }
  EXPECT_NO_THROW(voxels->setOctree(storage));
  EXPECT_NO_THROW(voxels->setOctree(nullptr));
  EXPECT_EQ(voxels->getVersion(), version);
  {
    auto edit = world->edit();
    voxels->updateOccupancy(Eigen::Vector3d::Constant(0.05));
    EXPECT_EQ(voxels->getVersion(), version + 1u);
    EXPECT_GT(world->getTopologyRevision(), topology);
    EXPECT_EQ(world->getParameterRevision(), parameters);
  }
  EXPECT_TRUE(group->collide());
  EXPECT_NO_THROW(world->step());
}
#endif

//==============================================================================
TEST(World, NestedGroupUpdatesAllowDifferentGroupsButRejectCycles)
{
  auto detector = std::make_shared<SubscriptionReentryDetector>();
  auto first = detector->createCollisionGroup();
  auto second = detector->createCollisionGroup();
  auto firstSource = Group::create();
  auto secondSource = Group::create();
  first->subscribeTo(firstSource);
  second->subscribeTo(secondSource);
  auto firstRobot = createBox(Eigen::Vector3d::Ones());
  auto secondRobot = createBox(Eigen::Vector3d::Ones());
  firstSource->addBodyNode(firstRobot->getBodyNode(0));
  secondSource->addBodyNode(secondRobot->getBodyNode(0));
  int depth = 0;
  detector->mCallback = [&] {
    if (++depth == 1)
      second->update();
    else
      first->update();
  };
  EXPECT_THROW(first->update(), std::runtime_error);
  EXPECT_EQ(first->getNumShapeFrames(), 0u);
  EXPECT_EQ(second->getNumShapeFrames(), 0u);
  depth = 0;
  detector->mCallback = [&] {
    if (++depth == 1)
      second->update();
  };
  EXPECT_NO_THROW(first->update());
  detector->mCallback = nullptr;
  EXPECT_EQ(first->getNumShapeFrames(), 1u);
  EXPECT_EQ(second->getNumShapeFrames(), 1u);
}

//==============================================================================
TEST(World, SharedGeometryUsesCurrentListenerOwners)
{
  auto first = World::create();
  auto second = World::create();
  auto shape = std::make_shared<ConvexMeshShape>(subscriptionTetrahedron(1));
  auto firstFrame = std::make_shared<SimpleFrame>();
  auto secondFrame = std::make_shared<SimpleFrame>();
  firstFrame->setShape(shape);
  secondFrame->setShape(shape);
  first->addSimpleFrame(firstFrame);
  second->addSimpleFrame(secondFrame);
  first->compile();
  second->compile();
  {
    auto firstEdit = first->edit();
    EXPECT_THROW(
        shape->setMesh(subscriptionTetrahedron(2)), std::runtime_error);
    {
      auto secondEdit = second->edit();
      EXPECT_NO_THROW(shape->setMesh(subscriptionTetrahedron(2)));
      secondFrame->setShape(nullptr);
    }
    EXPECT_NO_THROW(shape->setMesh(subscriptionTetrahedron(3)));
    firstFrame->setShape(nullptr);
  }
  EXPECT_NO_THROW(shape->setMesh(subscriptionTetrahedron(4)));
  auto parent = std::make_shared<SimpleFrame>();
  auto child = std::make_shared<SimpleFrame>(parent.get());
  child->setShape(shape);
  {
    auto edit = first->edit();
    first->addSimpleFrame(parent);
  }
  EXPECT_THROW(shape->setMesh(subscriptionTetrahedron(5)), std::runtime_error);
  {
    auto edit = first->edit();
    child->setParentFrame(Frame::World());
  }
  EXPECT_NO_THROW(shape->setMesh(subscriptionTetrahedron(5)));
  child.reset();
  EXPECT_NO_THROW(shape->setMesh(subscriptionTetrahedron(6)));
}

//==============================================================================
TEST(World, NestedTopologyShapeVersionsKeepTheirCategory)
{
  auto world = World::create();
  auto shape = std::make_shared<ConvexMeshShape>(subscriptionTetrahedron(1));
  auto frame = std::make_shared<SimpleFrame>();
  frame->setShape(shape);
  auto other = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  auto otherFrame = std::make_shared<SimpleFrame>();
  otherFrame->setShape(other);
  world->addSimpleFrame(frame);
  world->addSimpleFrame(otherFrame);
  world->compile();
  bool nested = false;
  auto listener = shape->onVersionChanged.connect([&](Shape*, std::size_t) {
    if (std::exchange(nested, true))
      return;
    shape->incrementVersion();
    other->setSize(Eigen::Vector3d::Constant(2));
  });
  const auto topology = world->getTopologyRevision();
  const auto parameters = world->getParameterRevision();
  const auto version = shape->getVersion();
  {
    auto edit = world->edit();
    shape->setMesh(subscriptionTetrahedron(2));
    EXPECT_EQ(world->getTopologyRevision(), topology + 1u);
    EXPECT_EQ(world->getParameterRevision(), parameters + 2u);
    EXPECT_EQ(shape->getVersion(), version + 2u);
  }
  EXPECT_TRUE(nested);
}

//==============================================================================
TEST(World, EditedConvexStorageChangesCollision)
{
  auto world = World::create();
  world->setCollisionDetector(CollisionDetectorType::Dart);
  auto shape = std::make_shared<ConvexMeshShape>(subscriptionTetrahedron(1));
  auto frame = std::make_shared<SimpleFrame>();
  frame->setShape(shape);
  auto box = std::make_shared<SimpleFrame>();
  box->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.1)));
  box->setRelativeTranslation(Eigen::Vector3d::Constant(0.8));
  auto group = world->getConstraintSolver()->getCollisionGroup();
  group->addShapeFramesOf(frame.get(), box.get());
  world->compile();
  EXPECT_FALSE(group->collide());
  const auto topology = world->getTopologyRevision();
  {
    auto edit = world->edit();
    shape->setMesh(subscriptionTetrahedron(3));
    EXPECT_GT(world->getTopologyRevision(), topology);
  }
  EXPECT_TRUE(group->collide());
}

//==============================================================================
TEST(World, BetweenStepBorrowedViewChangesSynchronizeOnlyWhenAutomatic)
{
  for (const bool automatic : {false, true}) {
    auto world = World::create();
    auto view = Group::create();
    auto group = world->getConstraintSolver()->getCollisionGroup();
    group->subscribeTo(view);
    group->setAutomaticUpdate(automatic);
    auto robot = createBox(Eigen::Vector3d::Ones());
    ASSERT_NO_THROW(world->step());
    view->addBodyNode(robot->getBodyNode(0));
    ASSERT_NO_THROW(world->step());
    EXPECT_EQ(group->getNumShapeFrames(), automatic ? 1u : 0u);
    EXPECT_FALSE(world->isCompiled());
    if (automatic) {
      view->removeBodyNode(robot->getBodyNode(0));
      ASSERT_NO_THROW(world->step());
      EXPECT_EQ(group->getNumShapeFrames(), 0u);
    }
  }
}

//==============================================================================
TEST(World, ExpiredEmptySourceCannotSynchronizeInsideCallback)
{
  for (const bool compiled : {false, true}) {
    auto world = World::create();
    DeactivationOptions options;
    options.mEnabled = false;
    world->setDeactivationOptions(options);
    auto ground = createBox(Eigen::Vector3d(10, 10, 1));
    ground->setMobile(false);
    world->addSkeleton(ground);
    world->addSkeleton(
        createBox(Eigen::Vector3d::Ones(), Eigen::Vector3d(0, 0, 0.95)));
    auto view = Group::create();
    auto group = world->getConstraintSolver()->getCollisionGroup();
    group->subscribeTo(view);
    auto handler = std::make_shared<StepCallbackHandler>();
    world->getConstraintSolver()->addContactSurfaceHandler(handler);
    if (compiled)
      world->compile();
    const auto version = group->getContentVersion();
    const auto count = group->getNumShapeFrames();
    expectStepRejectsEdit(*world, handler->mCallback, [&] {
      view.reset();
      group->update();
    });
    EXPECT_EQ(group->getContentVersion(), version);
    EXPECT_EQ(group->getNumShapeFrames(), count);
    {
      auto edit = world->edit();
      EXPECT_NO_THROW(group->update());
    }
    EXPECT_NO_THROW(world->step());
  }
}

//==============================================================================
TEST(World, GeometryPreflightFollowsMovedBodyAndBusyLazyOwner)
{
  auto first = World::create();
  auto second = World::create();
  auto source = createBox(Eigen::Vector3d::Ones());
  auto target = Skeleton::create();
  auto* body = source->getBodyNode(0);
  auto shape = std::make_shared<ConvexMeshShape>(subscriptionTetrahedron(1));
  body->getShapeNode(0)->setShape(shape);
  first->setCollisionDetector(CollisionDetectorType::Dart);
  second->setCollisionDetector(CollisionDetectorType::Dart);
  first->addSkeleton(source);
  second->addSkeleton(target);
  first->compile();
  second->compile();
  {
    auto firstEdit = first->edit();
    auto secondEdit = second->edit();
    ASSERT_TRUE(body->moveTo(target, nullptr));
    first->getConstraintSolver()->getCollisionGroup()->update();
    second->getConstraintSolver()->getCollisionGroup()->update();
  }
  ASSERT_TRUE(first->isCompiled());
  ASSERT_TRUE(second->isCompiled());
  {
    auto firstEdit = first->edit();
    EXPECT_THROW(
        shape->setMesh(subscriptionTetrahedron(2)), std::runtime_error);
  }
  {
    auto secondEdit = second->edit();
    EXPECT_NO_THROW(shape->setMesh(subscriptionTetrahedron(2)));
  }
  auto lazy = World::create();
  auto ground = createBox(Eigen::Vector3d(10, 10, 1));
  ground->setMobile(false);
  lazy->addSkeleton(ground);
  lazy->addSkeleton(
      createBox(Eigen::Vector3d::Ones(), Eigen::Vector3d(0, 0, 0.95)));
  auto frame = std::make_shared<SimpleFrame>();
  auto looseShape
      = std::make_shared<ConvexMeshShape>(subscriptionTetrahedron(1));
  frame->setShape(looseShape);
  lazy->addSimpleFrame(frame);
  auto handler = std::make_shared<StepCallbackHandler>();
  lazy->getConstraintSolver()->addContactSurfaceHandler(handler);
  const auto storage = looseShape->getMesh();
  const auto version = looseShape->getVersion();
  expectStepRejectsEdit(*lazy, handler->mCallback, [&] {
    looseShape->setMesh(subscriptionTetrahedron(2));
  });
  EXPECT_EQ(looseShape->getMesh(), storage);
  EXPECT_EQ(looseShape->getVersion(), version);
  EXPECT_FALSE(lazy->isCompiled());
}

namespace {
std::shared_ptr<const aiScene> subscriptionMeshScene(float size)
{
  auto scene = std::make_shared<aiScene>();
  scene->mNumMeshes = 1;
  scene->mMeshes = new aiMesh* [1] {
    new aiMesh
  };
  auto* mesh = scene->mMeshes[0];
  mesh->mNumVertices = 4;
  mesh->mVertices
      = new aiVector3D[4]{{0, 0, 0}, {size, 0, 0}, {0, size, 0}, {0, 0, size}};
  mesh->mNumFaces = 4;
  mesh->mFaces = new aiFace[4];
  const unsigned int faces[4][3]{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
  for (int i = 0; i < 4; ++i) {
    mesh->mFaces[i].mNumIndices = 3;
    mesh->mFaces[i].mIndices
        = new unsigned int[3]{faces[i][0], faces[i][1], faces[i][2]};
  }
  scene->mRootNode = new aiNode;
  scene->mRootNode->mNumMeshes = 1;
  scene->mRootNode->mMeshes = new unsigned int[1]{0};
  return scene;
}
} // namespace

//==============================================================================
TEST(World, EditedMeshStorageChangesTopologyAndCollision)
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  auto world = World::create();
  auto shape = std::shared_ptr<MeshShape>(
      new MeshShape(Eigen::Vector3d::Ones(), subscriptionMeshScene(1)));
  auto meshBody = createBox(Eigen::Vector3d::Ones());
  meshBody->getBodyNode(0)->getShapeNode(0)->setShape(shape);
  auto box = createBox(
      Eigen::Vector3d::Constant(0.3), Eigen::Vector3d::Constant(0.7));
  world->addSkeleton(meshBody);
  world->addSkeleton(box);
  auto group = world->getConstraintSolver()->getCollisionGroup();
  world->compile();
  EXPECT_FALSE(group->collide());
  const auto topology = world->getTopologyRevision();
  {
    auto edit = world->edit();
    shape->setMesh(subscriptionMeshScene(2));
    EXPECT_GT(world->getTopologyRevision(), topology);
  }
  EXPECT_TRUE(world->isCompiled());
  EXPECT_TRUE(group->collide());
  const auto replacedTopology = world->getTopologyRevision();
  const auto parameters = world->getParameterRevision();
  shape->setScale(1.1);
  EXPECT_EQ(world->getTopologyRevision(), replacedTopology);
  EXPECT_GT(world->getParameterRevision(), parameters);
  DART_SUPPRESS_DEPRECATED_END
}

#if HAVE_OCTOMAP
//==============================================================================
TEST(World, NestedTopologyToVisualVersionDoesNotNotifyParameters)
{
  auto world = World::create();
  auto shape = std::make_shared<VoxelGridShape>(0.1);
  auto frame = std::make_shared<SimpleFrame>();
  frame->setShape(shape);
  world->addSimpleFrame(frame);
  world->compile();
  bool nested = false;
  auto connection = shape->onVersionChanged.connect([&](Shape*, std::size_t) {
    if (!std::exchange(nested, true))
      shape->notifyColorUpdated(Eigen::Vector4d::Ones());
  });
  const auto topology = world->getTopologyRevision();
  const auto parameters = world->getParameterRevision();
  const auto version = shape->getVersion();
  {
    auto edit = world->edit();
    shape->updateOccupancy(Eigen::Vector3d::Zero());
    EXPECT_EQ(world->getTopologyRevision(), topology + 1u);
    EXPECT_EQ(world->getParameterRevision(), parameters);
    EXPECT_EQ(shape->getVersion(), version + 2u);
  }
  EXPECT_TRUE(nested);
}
#endif
