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
#include "dart/common/Factory.hpp"
#include "dart/common/Macros.hpp"
#include "dart/common/Uri.hpp"
#include "dart/common/detail/ModelMutation.hpp"
#include "dart/constraint/detail/NativeDetectorInitialization.h"
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
#include "dart/dynamics/FixedFrame.hpp"
#include "dart/dynamics/Group.hpp"
#include "dart/dynamics/LineSegmentShape.hpp"
#include "dart/dynamics/Linkage.hpp"
#include "dart/dynamics/PointCloudShape.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/math/Random.hpp"
#include "dart/simulation/DeactivationOptions.hpp"
#include "dart/simulation/Recording.hpp"
#include "dart/utils/SkelParser.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <new>
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

namespace {

thread_local std::function<bool()>* gDetectorAllocationCallback = nullptr;

class ScopedDetectorAllocationCallback
{
public:
  explicit ScopedDetectorAllocationCallback(std::function<bool()> callback)
    : mCallback(std::move(callback)), mPrevious(gDetectorAllocationCallback)
  {
    gDetectorAllocationCallback = &mCallback;
  }

  ~ScopedDetectorAllocationCallback()
  {
    gDetectorAllocationCallback = mPrevious;
  }

private:
  std::function<bool()> mCallback;
  std::function<bool()>* mPrevious;
};

} // namespace

void* operator new(std::size_t bytes)
{
  if (auto* callback = std::exchange(gDetectorAllocationCallback, nullptr)) {
    if (!(*callback)())
      gDetectorAllocationCallback = callback;
  }
  if (auto* pointer = std::malloc(bytes == 0u ? 1u : bytes))
    return pointer;
  throw std::bad_alloc();
}

void* operator new[](std::size_t bytes)
{
  return ::operator new(bytes);
}

void* operator new(std::size_t bytes, const std::nothrow_t&) noexcept
{
  try {
    return ::operator new(bytes);
  } catch (const std::bad_alloc&) {
    return nullptr;
  }
}

void* operator new[](std::size_t bytes, const std::nothrow_t&) noexcept
{
  try {
    return ::operator new[](bytes);
  } catch (const std::bad_alloc&) {
    return nullptr;
  }
}

void operator delete(void* pointer) noexcept
{
  std::free(pointer);
}

void operator delete[](void* pointer) noexcept
{
  std::free(pointer);
}

void operator delete(void* pointer, std::size_t) noexcept
{
  std::free(pointer);
}

void operator delete[](void* pointer, std::size_t) noexcept
{
  std::free(pointer);
}

void operator delete(void* pointer, const std::nothrow_t&) noexcept
{
  ::operator delete(pointer);
}

void operator delete[](void* pointer, const std::nothrow_t&) noexcept
{
  ::operator delete[](pointer);
}

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;

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

//==============================================================================
TEST(World, LazySharedMembershipAndOwnerLifetimeRemainSupported)
{
  auto robot = Skeleton::create("shared");
  auto* body = robot->createJointAndBodyNodePair<RevoluteJoint>().second;
  auto first = World::create();
  auto second = World::create();
  first->addSkeleton(robot);
  second->addSkeleton(robot);
  EXPECT_THROW(first->compile(), std::runtime_error);
  EXPECT_FALSE(first->isCompiled());
  EXPECT_NO_THROW(robot->createJointAndBodyNodePair<RevoluteJoint>(body));
  EXPECT_NO_THROW(first->step());
  EXPECT_NO_THROW(second->step());
  second->removeSkeleton(robot);
  first->compile();
  EXPECT_THROW(second->addSkeleton(robot), std::runtime_error);
  EXPECT_FALSE(second->hasSkeleton(robot));
  first.reset();
  EXPECT_NO_THROW(robot->createJointAndBodyNodePair<RevoluteJoint>(body));
  EXPECT_NO_THROW(second->addSkeleton(robot));
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
  EXPECT_EQ(SkeletonVersionAccess::getGlobalStructuralVersion(), structural);
  EXPECT_GT(world->getParameterRevision(), parameter);
  EXPECT_EQ(world->getTopologyRevision(), topology);
  const auto changedParameters = world->getParameterRevision();
  robot->setPosition(0, 0.1);
  robot->setVelocity(0, 0.2);
  robot->setCommand(0, 0.3);
  body->setName("renamed");
  shape->getVisualAspect()->setColor(Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_EQ(SkeletonVersionAccess::getGlobalStructuralVersion(), structural);
  EXPECT_EQ(world->getParameterRevision(), changedParameters);
  EXPECT_EQ(world->getTopologyRevision(), topology);
  EXPECT_NO_THROW(world->step());
  EXPECT_NO_THROW(world->step());
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
  EXPECT_EQ(SkeletonVersionAccess::getGlobalStructuralVersion(), structural);
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
  EXPECT_THROW(std::make_shared<SimpleFrame>(root.get()), std::runtime_error);
  EXPECT_EQ(root->getChildFrames(), children);
  auto parameters = world->getParameterRevision();
  EXPECT_THROW(
      std::make_shared<NamedFixedFrame>(child.get()), std::runtime_error);
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
  EXPECT_EQ(SkeletonVersionAccess::getGlobalStructuralVersion(), structural);
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
  EXPECT_GT(lazy->getParameterRevision(), lazyParameters);
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
class OwnershipQueryNode final : public Node, public common::Composite
{
public:
  explicit OwnershipQueryNode(BodyNode* body) : Node(body) {}

  const std::string& setName(const std::string& name) override
  {
    mName = name;
    return mName;
  }

  const std::string& getName() const override
  {
    return mName;
  }

  void setValue(int value)
  {
    common::detail::notifyParameterMutation(this);
    mValue = value;
  }

  void associate(BodyNode* body)
  {
    mBodyNode = body;
  }

  std::shared_ptr<const Skeleton> getSkeleton() const override
  {
    mTrace->push_back("get");
    if (mThrow)
      throw std::runtime_error("later owner getter");
    const auto* skeleton = mBodyNode->getSkeleton().get();
    return std::shared_ptr<const Skeleton>(skeleton, [this](const Skeleton*) {
      mTrace->push_back("release");
      if (mOnRelease) {
        auto callback = std::move(mOnRelease);
        mOnRelease = nullptr;
        callback();
      }
    });
  }

  std::vector<std::string>* mTrace = nullptr;
  mutable std::function<void()> mOnRelease;
  bool mThrow = false;
  int mValue = 0;

protected:
  Node* cloneNode(BodyNode* body) const override
  {
    return new OwnershipQueryNode(body);
  }

private:
  std::string mName;
};

class OwnershipQuerySource final : public Group
{
public:
  explicit OwnershipQuerySource(BodyNode* body)
    : Group("query_source", std::vector<BodyNode*>{body}, false, false)
  {
  }

  std::size_t getNumBodyNodes() const override
  {
    if (mOnRead) {
      auto callback = std::move(mOnRead);
      mOnRead = nullptr;
      callback();
    }
    return Group::getNumBodyNodes();
  }

  mutable std::function<void()> mOnRead;
};

// Quiet Frames have no registered child-set entries. A custom reader can
// change this protected association without recursively taking the World lock.
class OwnershipQueryFrame final : public SimpleFrame
{
public:
  OwnershipQueryFrame()
    : Entity(Frame::World(), true), Frame(Frame::World()), SimpleFrame()
  {
  }

  void associate(Frame* parent)
  {
    mParentFrame = parent;
  }
};

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

class OwnershipQueryLeft : public OwnershipQueryFrameArm
{
};
class OwnershipQueryRight : public OwnershipQueryFrameArm
{
};
class OwnershipQueryAmbiguous final : public OwnershipQueryLeft,
                                      public OwnershipQueryRight,
                                      public common::Composite
{
public:
  OwnershipQueryAmbiguous() : Entity(Frame::World(), true) {}

  const std::string& setName(const std::string& name) override
  {
    mName = name;
    return mName;
  }
  const std::string& getName() const override
  {
    return mName;
  }

  void dirtyTransform() override
  {
    OwnershipQueryLeft::dirtyTransform();
    OwnershipQueryRight::dirtyTransform();
  }
  void dirtyVelocity() override
  {
    OwnershipQueryLeft::dirtyVelocity();
    OwnershipQueryRight::dirtyVelocity();
  }
  void dirtyAcceleration() override
  {
    OwnershipQueryLeft::dirtyAcceleration();
    OwnershipQueryRight::dirtyAcceleration();
  }

protected:
  void changeParentFrame(Frame* parent) override
  {
    OwnershipQueryLeft::changeParentFrame(parent);
    OwnershipQueryRight::changeParentFrame(parent);
  }

private:
  std::string mName;
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

TEST(World, OwnershipQueryReadsChangedNodeAssociationForEachWorld)
{
  auto first = World::create();
  auto second = World::create();
  auto loose = createBox(Eigen::Vector3d::Ones());
  auto owned = createBox(Eigen::Vector3d::Ones());
  auto source = std::make_shared<OwnershipQuerySource>(loose->getBodyNode(0));
  first->getConstraintSolver()->getCollisionGroup()->subscribeTo(source);
  second->addSkeleton(owned);
  std::vector<std::string> trace;
  OwnershipQueryNode node(loose->getBodyNode(0));
  node.mTrace = &trace;
  source->mOnRead = [&] {
    trace.push_back("source");
    node.associate(owned->getBodyNode(0));
  };
  const auto firstRevision = first->getParameterRevision();
  const auto secondRevision = second->getParameterRevision();
  node.setValue(1);
  EXPECT_EQ(node.mValue, 1);
  EXPECT_EQ(first->getParameterRevision(), firstRevision + 1u);
  EXPECT_EQ(second->getParameterRevision(), secondRevision + 1u);
  ASSERT_GE(trace.size(), 5u);
  EXPECT_EQ(
      (std::vector<std::string>(trace.end() - 5, trace.end())),
      (std::vector<std::string>{"get", "release", "source", "get", "release"}));
}

TEST(World, OwnershipQueryReleasesGetterBeforePositiveMembership)
{
  auto observer = World::create();
  auto first = World::create();
  auto second = World::create();
  auto firstModel = createBox(Eigen::Vector3d::Ones());
  auto secondModel = createBox(Eigen::Vector3d::Ones());
  auto source
      = std::make_shared<OwnershipQuerySource>(firstModel->getBodyNode(0));
  observer->getConstraintSolver()->getCollisionGroup()->subscribeTo(source);
  first->addSkeleton(firstModel);
  second->addSkeleton(secondModel);
  std::vector<std::string> trace;
  OwnershipQueryNode node(firstModel->getBodyNode(0));
  node.mTrace = &trace;
  source->mOnRead = [&] {
    trace.push_back("source");
    node.mOnRelease = [&] {
      node.associate(secondModel->getBodyNode(0));
    };
  };
  const auto observerRevision = observer->getParameterRevision();
  const auto firstRevision = first->getParameterRevision();
  const auto secondRevision = second->getParameterRevision();
  node.setValue(1);
  EXPECT_EQ(observer->getParameterRevision(), observerRevision + 1u);
  EXPECT_EQ(first->getParameterRevision(), firstRevision + 1u);
  EXPECT_EQ(second->getParameterRevision(), secondRevision + 1u);
  ASSERT_GE(trace.size(), 5u);
  EXPECT_EQ(
      (std::vector<std::string>(trace.end() - 5, trace.end())),
      (std::vector<std::string>{"source", "get", "release", "get", "release"}));
}

TEST(World, OwnershipQueryLaterThrowPreservesEarlierOwnerEffects)
{
  auto first = World::create();
  auto second = World::create();
  auto loose = createBox(Eigen::Vector3d::Ones());
  auto source = std::make_shared<OwnershipQuerySource>(loose->getBodyNode(0));
  first->getConstraintSolver()->getCollisionGroup()->subscribeTo(source);
  std::vector<std::string> trace;
  OwnershipQueryNode node(loose->getBodyNode(0));
  node.mTrace = &trace;
  source->mOnRead = [&] {
    trace.push_back("source");
    node.mThrow = true;
  };
  const auto firstRevision = first->getParameterRevision();
  const auto secondRevision = second->getParameterRevision();
  EXPECT_THROW(node.setValue(1), std::runtime_error);
  EXPECT_EQ(node.mValue, 0);
  EXPECT_EQ(first->getParameterRevision(), firstRevision + 1u);
  EXPECT_EQ(second->getParameterRevision(), secondRevision);
  ASSERT_GE(trace.size(), 4u);
  EXPECT_EQ(
      (std::vector<std::string>(trace.end() - 4, trace.end())),
      (std::vector<std::string>{"get", "release", "source", "get"}));
}

TEST(World, OwnershipQueryNullBodySkipsGetterForLaterWorld)
{
  auto first = World::create();
  auto second = World::create();
  auto loose = createBox(Eigen::Vector3d::Ones());
  auto source = std::make_shared<OwnershipQuerySource>(loose->getBodyNode(0));
  first->getConstraintSolver()->getCollisionGroup()->subscribeTo(source);
  std::vector<std::string> trace;
  OwnershipQueryNode node(loose->getBodyNode(0));
  node.mTrace = &trace;
  source->mOnRead = [&] {
    trace.push_back("source");
    node.associate(nullptr);
    node.mThrow = true;
  };
  const auto firstRevision = first->getParameterRevision();
  const auto secondRevision = second->getParameterRevision();
  EXPECT_NO_THROW(node.setValue(1));
  EXPECT_EQ(node.mValue, 1);
  EXPECT_EQ(first->getParameterRevision(), firstRevision + 1u);
  EXPECT_EQ(second->getParameterRevision(), secondRevision);
  ASSERT_GE(trace.size(), 3u);
  EXPECT_EQ(
      (std::vector<std::string>(trace.end() - 3, trace.end())),
      (std::vector<std::string>{"get", "release", "source"}));
}

TEST(World, OwnershipQueryReadsChangedFrameAncestryForEachWorld)
{
  auto first = World::create();
  auto second = World::create();
  auto loose = createBox(Eigen::Vector3d::Ones());
  auto owned = createBox(Eigen::Vector3d::Ones());
  auto source = std::make_shared<OwnershipQuerySource>(loose->getBodyNode(0));
  first->getConstraintSolver()->getCollisionGroup()->subscribeTo(source);
  second->addSkeleton(owned);
  OwnershipQueryFrame frame;
  frame.createDynamicsAspect();
  frame.associate(loose->getBodyNode(0));
  source->mOnRead = [&] {
    frame.associate(owned->getBodyNode(0));
  };
  const auto firstRevision = first->getParameterRevision();
  const auto secondRevision = second->getParameterRevision();
  frame.getDynamicsAspect()->setFrictionCoeff(0.7);
  EXPECT_EQ(first->getParameterRevision(), firstRevision + 1u);
  EXPECT_EQ(second->getParameterRevision(), secondRevision + 1u);
}

TEST(World, OwnershipQueryPreservesAmbiguousFrameRoundTrip)
{
  auto world = World::create();
  auto model = createBox(Eigen::Vector3d::Ones());
  world->addSkeleton(model);
  OwnershipQueryAmbiguous query;
  auto* left = static_cast<OwnershipQueryLeft*>(&query);
  left->associate(model->getBodyNode(0));
  const auto* object = dynamic_cast<const common::Composite*>(left);
  ASSERT_NE(object, nullptr);
  ASSERT_EQ(dynamic_cast<const Frame*>(object), nullptr);
  const auto topology = world->getTopologyRevision();
  left->announce();
  EXPECT_EQ(world->getTopologyRevision(), topology);
}

TEST(World, OwnershipQueryResolvesEachLifecyclePhaseSeparately)
{
  auto world = World::create();
  auto model = createBox(Eigen::Vector3d::Ones());
  world->addSkeleton(model);
  bool compositeInBasePhase = true;
  const auto beforeConstruction = world->getTopologyRevision();
  auto query = std::make_unique<OwnershipQueryLifecycle>(
      &compositeInBasePhase, model->getBodyNode(0));
  EXPECT_FALSE(compositeInBasePhase);
  EXPECT_GT(world->getTopologyRevision(), beforeConstruction);
  const auto topology = world->getTopologyRevision();
  query->announce();
  EXPECT_GT(world->getTopologyRevision(), topology);
  const auto beforeDestruction = world->getTopologyRevision();
  compositeInBasePhase = true;
  query.reset();
  EXPECT_FALSE(compositeInBasePhase);
  EXPECT_GT(world->getTopologyRevision(), beforeDestruction);
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

  void checkLiveBeforeGroupNotification(
      const collision::CollisionGroupPtr& group)
  {
    bool observed = false;
    const auto topology = mWorld->getTopologyRevision();
    {
      ScopedDetectorAllocationCallback allocation([&] {
        if (group->getNumShapeFrames() == 0u)
          return false;
        observed = true;
        EXPECT_EQ(mWorld->getTopologyRevision(), topology);
        const auto parameters = mWorld->getParameterRevision();
        mBody->setCollidable(false);
        EXPECT_GT(mWorld->getParameterRevision(), parameters);
        auto descendant = std::make_shared<BareFrame>(mParent.get());
        EXPECT_GT(mWorld->getTopologyRevision(), topology);
        return true;
      });
      group->addShapeFrame(mBody->getShapeNodeWith<CollisionAspect>(0));
    }
    EXPECT_TRUE(observed);
    group->removeAllShapeFrames();
    const auto parameters = mWorld->getParameterRevision();
    mBody->setCollidable(true);
    EXPECT_EQ(mWorld->getParameterRevision(), parameters);
    EXPECT_EQ(group->getNumShapeFrames(), 0u);
  }

  void checkBorrowedViewAndFrozenMutation()
  {
    auto view = Group::create("detector_initialization_view");
    auto group = mWorld->getConstraintSolver()->getCollisionGroup();
    group->subscribeTo(view);
    ASSERT_TRUE(view->addBodyNode(mBody));
    auto parameters = mWorld->getParameterRevision();
    mBody->setCollidable(false);
    EXPECT_GT(mWorld->getParameterRevision(), parameters);
    ASSERT_TRUE(view->removeBodyNode(mBody));
    parameters = mWorld->getParameterRevision();
    mBody->setCollidable(true);
    EXPECT_EQ(mWorld->getParameterRevision(), parameters);
    ASSERT_TRUE(view->addBodyNode(mBody));
    mWorld->compile();
    const auto children = mParent->getChildFrames();
    EXPECT_THROW(
        std::make_shared<BareFrame>(mParent.get()), std::runtime_error);
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

void expectDetectorCertificateStaysRetired(constraint::ConstraintSolver* solver)
{
  EXPECT_FALSE(constraint::detail::hasNativeOwnershipCertificate(solver));
  solver->setCollisionDetector(collision::FCLCollisionDetector::create());
  EXPECT_FALSE(constraint::detail::hasNativeOwnershipCertificate(solver));
  EXPECT_EQ(solver->getCollisionGroup()->getNumShapeFrames(), 0u);
  EXPECT_EQ(
      solver->getCollisionGroup()->getCollisionDetector(),
      solver->getCollisionDetector());
}

} // namespace

TEST(World, DetectorInitializationAllocationKeepsBorrowedOwnershipLive)
{
  auto world = World::create();
  auto* solver = world->getConstraintSolver();
  auto oldGroup = solver->getCollisionGroup();
  auto incoming = collision::FCLCollisionDetector::create();
  DetectorInitializationOwnershipProbe probe(world);
  bool called = false;
  {
    ScopedDetectorAllocationCallback allocation([&] {
      called = true;
      EXPECT_EQ(solver->getCollisionDetector(), incoming);
      EXPECT_EQ(solver->getCollisionGroup(), oldGroup);
      probe.checkLiveBeforeGroupNotification(oldGroup);
      return true;
    });
    solver->setCollisionDetector(incoming);
  }
  ASSERT_TRUE(called);
  EXPECT_NE(solver->getCollisionGroup(), oldGroup);
  probe.checkBorrowedViewAndFrozenMutation();
}

TEST(World, DetectorInitializationPreservesOnlyPristineNativeCertificate)
{
  auto world = World::create();
  auto* solver = world->getConstraintSolver();
  ASSERT_TRUE(constraint::detail::hasNativeOwnershipCertificate(solver));
  std::vector<collision::CollisionDetectorPtr> detectors{
      collision::FCLCollisionDetector::create(),
      collision::DARTCollisionDetector::create(),
      collision::FCLCollisionDetector::create()};
#if HAVE_BULLET
  detectors.push_back(collision::BulletCollisionDetector::create());
#endif
#if HAVE_ODE
  detectors.push_back(collision::OdeCollisionDetector::create());
#endif
  for (const auto& detector : detectors) {
    const auto topology = world->getTopologyRevision();
    solver->setCollisionDetector(detector);
    EXPECT_EQ(world->getTopologyRevision(), topology + 1u);
    EXPECT_TRUE(constraint::detail::hasNativeOwnershipCertificate(solver));
    EXPECT_EQ(solver->getCollisionGroup()->getCollisionDetector(), detector);
  }
  world->addSkeleton(createBox(Eigen::Vector3d::Ones()));
  EXPECT_TRUE(constraint::detail::hasNativeOwnershipCertificate(solver));
}

TEST(World, DetectorInitializationReleaseKeepsBorrowedOwnershipLive)
{
  auto world = World::create();
  auto* solver = world->getConstraintSolver();
  DetectorInitializationOwnershipProbe probe(world);
  collision::CollisionGroupPtr oldGroup;
  auto incoming = collision::FCLCollisionDetector::create();
  bool called = false;
  NativeDetectorReleaseCallback release(solver, [&] {
    called = true;
    EXPECT_EQ(solver->getCollisionDetector(), incoming);
    EXPECT_EQ(solver->getCollisionGroup(), oldGroup);
    probe.checkLiveBeforeGroupNotification(oldGroup);
  });
  oldGroup = solver->getCollisionGroup();
  solver->setCollisionDetector(incoming);
  ASSERT_TRUE(called);
  EXPECT_NE(solver->getCollisionGroup(), oldGroup);
  probe.checkBorrowedViewAndFrozenMutation();
}

TEST(World, DetectorInitializationNestedReplacementCannotRecertify)
{
  auto world = World::create();
  auto* solver = world->getConstraintSolver();
  auto nested = collision::FCLCollisionDetector::create();
  bool called = false;
  NativeDetectorReleaseCallback release(solver, [&] {
    called = true;
    solver->setCollisionDetector(nested);
    EXPECT_EQ(solver->getCollisionDetector(), nested);
  });
  ASSERT_TRUE(constraint::detail::hasNativeOwnershipCertificate(solver));
  solver->setCollisionDetector(collision::FCLCollisionDetector::create());
  ASSERT_TRUE(called);
  EXPECT_EQ(solver->getCollisionDetector(), nested);
  expectDetectorCertificateStaysRetired(solver);
  DetectorInitializationOwnershipProbe(world)
      .checkBorrowedViewAndFrozenMutation();
}

TEST(World, DetectorInitializationMutationThenEmptyCannotRecertify)
{
  auto world = World::create();
  auto* solver = world->getConstraintSolver();
  auto view = Group::create("temporary_detector_view");
  auto loose = Skeleton::create();
  auto* body = loose->createJointAndBodyNodePair<FreeJoint>().second;
  bool called = false;
  NativeDetectorReleaseCallback release(solver, [&] {
    called = true;
    auto group = solver->getCollisionGroup();
    group->subscribeTo(view);
    ASSERT_TRUE(view->addBodyNode(body));
    const auto parameters = world->getParameterRevision();
    body->setCollidable(false);
    EXPECT_GT(world->getParameterRevision(), parameters);
    ASSERT_TRUE(view->removeBodyNode(body));
    group->unsubscribeFrom(view.get());
    EXPECT_EQ(group->getNumShapeFrames(), 0u);
  });
  ASSERT_TRUE(constraint::detail::hasNativeOwnershipCertificate(solver));
  solver->setCollisionDetector(collision::FCLCollisionDetector::create());
  ASSERT_TRUE(called);
  expectDetectorCertificateStaysRetired(solver);
}

TEST(World, DetectorInitializationTopologyChangeThenRestoreCannotRecertify)
{
  auto world = World::create();
  auto* solver = world->getConstraintSolver();
  auto incoming = collision::DARTCollisionDetector::create();
  ASSERT_EQ(incoming->getNumCollisionThreads(), 1u);
  bool called = false;
  NativeDetectorReleaseCallback release(solver, [&] {
    called = true;
    const auto topology = world->getTopologyRevision();
    incoming->setNumCollisionThreads(2u);
    incoming->setNumCollisionThreads(1u);
    EXPECT_GT(world->getTopologyRevision(), topology);
    EXPECT_EQ(incoming->getNumCollisionThreads(), 1u);
  });
  ASSERT_TRUE(constraint::detail::hasNativeOwnershipCertificate(solver));
  solver->setCollisionDetector(incoming);
  ASSERT_TRUE(called);
  expectDetectorCertificateStaysRetired(solver);
}

TEST(World, DetectorInitializationThreadMismatchCannotRecertify)
{
  auto world = World::create();
  auto* solver = world->getConstraintSolver();
  auto incoming = collision::DARTCollisionDetector::create();
  incoming->setNumCollisionThreads(2u);
  ASSERT_EQ(solver->getNumSimulationThreads(), 1u);
  ASSERT_TRUE(constraint::detail::hasNativeOwnershipCertificate(solver));
  solver->setCollisionDetector(incoming);
  EXPECT_EQ(incoming->getNumCollisionThreads(), 1u);
  expectDetectorCertificateStaysRetired(solver);
}

TEST(World, DetectorInitializationAllocationFailureRecoversWithoutRecertifying)
{
  auto world = World::create();
  auto* solver = world->getConstraintSolver();
  auto oldGroup = solver->getCollisionGroup();
  auto incoming = collision::FCLCollisionDetector::create();
  ASSERT_TRUE(constraint::detail::hasNativeOwnershipCertificate(solver));
  bool called = false;
  bool caught = false;
  {
    ScopedDetectorAllocationCallback allocation([&]() -> bool {
      called = true;
      EXPECT_EQ(solver->getCollisionDetector(), incoming);
      EXPECT_EQ(solver->getCollisionGroup(), oldGroup);
      throw std::bad_alloc();
    });
    try {
      solver->setCollisionDetector(incoming);
    } catch (const std::bad_alloc&) {
      caught = true;
    }
  }
  ASSERT_TRUE(called);
  EXPECT_TRUE(caught);
  expectDetectorCertificateStaysRetired(solver);
  DetectorInitializationOwnershipProbe(world)
      .checkBorrowedViewAndFrozenMutation();
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

TEST(World, OwnershipReadsLiveBorrowedViewMembershipAndExpiry)
{
  auto world = World::create();
  world->addSkeleton(createBox(Eigen::Vector3d::Ones()));
  auto view = Group::create("view");
  world->getConstraintSolver()->getCollisionGroup()->subscribeTo(view);
  auto loose = Skeleton::create();
  auto* first = loose->createJointAndBodyNodePair<FreeJoint>().second;
  auto* second = loose->createJointAndBodyNodePair<FreeJoint>().second;
  ASSERT_TRUE(view->addBodyNode(first));
  auto parameter = world->getParameterRevision();
  first->setCollidable(false);
  EXPECT_GT(world->getParameterRevision(), parameter);
  ASSERT_TRUE(view->removeBodyNode(first));
  parameter = world->getParameterRevision();
  first->setCollidable(true);
  EXPECT_EQ(world->getParameterRevision(), parameter);
  ASSERT_TRUE(view->addBodyNode(second));
  second->setCollidable(false);
  EXPECT_GT(world->getParameterRevision(), parameter);
  view.reset();
  parameter = world->getParameterRevision();
  second->setCollidable(true);
  EXPECT_EQ(world->getParameterRevision(), parameter);
}

TEST(World, OwnershipDoesNotCallCustomFrameReaderUnderRegistryLock)
{
  auto world = World::create();
  world->addSkeleton(createBox(Eigen::Vector3d::Ones()));
  auto frame = std::make_shared<OwnershipReaderFrame>();
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  world->getConstraintSolver()->getCollisionGroup()->addShapeFrame(frame.get());
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
  auto firstParameter = first->getParameterRevision();
  auto secondParameter = second->getParameterRevision();
  robot->getBodyNode(0)->setCollidable(false);
  EXPECT_GT(first->getParameterRevision(), firstParameter);
  EXPECT_GT(second->getParameterRevision(), secondParameter);
  EXPECT_THROW(second->compile(), std::runtime_error);
  second->getConstraintSolver()->getCollisionGroup()->unsubscribeFrom(
      robot.get());
  EXPECT_NO_THROW(second->compile());
  EXPECT_NO_THROW(robot->createJointAndBodyNodePair<FreeJoint>());
  first->compile();
  EXPECT_THROW(
      robot->createJointAndBodyNodePair<FreeJoint>(), std::runtime_error);
}

TEST(World, OwnershipCustomShapeCallbackCannotRecertifyRawView)
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
    const auto parameter = world->getParameterRevision();
    body->setCollidable(false);
    EXPECT_GT(world->getParameterRevision(), parameter);
  };
  world->addSkeleton(robot);
  ASSERT_TRUE(called);
  world->addSkeleton(createBox(Eigen::Vector3d::Ones()));
  world->compile();
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
  child->setParentFrame(parent.get());
  EXPECT_TRUE(added);
  child->setParentFrame(Frame::World());
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
  EXPECT_THROW(std::make_shared<SimpleFrame>(middle.get()), std::runtime_error);
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
        std::make_shared<SimpleFrame>(middle.get()), std::runtime_error);
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
  EXPECT_THROW(std::make_shared<SimpleFrame>(parent.get()), std::runtime_error);
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
TEST(World, NonCompositeFrameChangesStampLazyTopology)
{
  auto world = World::create();
  auto parent = std::make_shared<BareFrame>(Frame::World());
  auto member = std::make_shared<SimpleFrame>(parent.get());
  world->addSimpleFrame(member);
  const auto topology = world->getTopologyRevision();
  const auto parameters = world->getParameterRevision();
  auto sibling = std::make_shared<SimpleFrame>(parent.get());
  EXPECT_GT(world->getTopologyRevision(), topology);
  const auto attachedTopology = world->getTopologyRevision();
  sibling->setParentFrame(Frame::World());
  EXPECT_GT(world->getTopologyRevision(), attachedTopology);
  EXPECT_EQ(world->getParameterRevision(), parameters);
  const auto& children = parent->getChildFrames();
  EXPECT_EQ(children.size(), 1u);
  EXPECT_EQ(children.count(member.get()), 1u);
  const auto detachedTopology = world->getTopologyRevision();
  auto destination = std::make_shared<SimpleFrame>();
  parent->setParentFrame(destination.get());
  EXPECT_GT(world->getTopologyRevision(), detachedTopology);
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
