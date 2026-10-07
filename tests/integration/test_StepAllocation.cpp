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

#include "AllocationCounting.hpp"
#include "dart/collision/CollisionFilter.hpp"
#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/collision/fcl/FCLCollisionDetector.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/BoxedLcpSolver.hpp"
#include "dart/constraint/ContactSurface.hpp"
#include "dart/dynamics/dynamics.hpp"
#include "dart/lcpsolver/dantzig/DantzigLcp.hpp"
#include "dart/simulation/World.hpp"
#include "dart/utils/SkelParser.hpp"

#if HAVE_BULLET
  #include "dart/collision/bullet/bullet.hpp"
#endif
#if HAVE_ODE
  #include "dart/collision/ode/ode.hpp"
#endif

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <cstddef>
#include <cstdlib>

namespace {

constexpr std::size_t kBoxesPerSide = 3u;
constexpr int kWarmupSteps = 50;
constexpr int kSoftSkelContactWarmupSteps = 500;
constexpr int kMeasuredSteps = 100;
constexpr double kBoxEdge = 0.9;
constexpr double kGroundThickness = 0.1;

void* volatile g_allocationSink = nullptr;

class CountingDantzigBoxedLcpSolver final
  : public dart::constraint::BoxedLcpSolver
{
public:
  explicit CountingDantzigBoxedLcpSolver(
      dart::test::CountingMemoryAllocator& allocator)
    : mScratch(allocator)
  {
  }

  const std::string& getType() const override
  {
    static const std::string type = "CountingDantzigBoxedLcpSolver";
    return type;
  }

  bool solve(
      int n,
      double* A,
      double* x,
      double* b,
      int nub,
      double* lo,
      double* hi,
      int* findex,
      bool earlyTermination) override
  {
    return dart::lcpsolver::dantzig::solveLcpWithScratch<double>(
        n, A, x, b, nullptr, nub, lo, hi, findex, mScratch, earlyTermination);
  }

#if DART_BUILD_MODE_DEBUG
  bool canSolve(int, const double*) override
  {
    return true;
  }
#endif

private:
  dart::lcpsolver::dantzig::DantzigLcpScratch<double> mScratch;
};

class PassThroughCollisionFilter final : public dart::collision::CollisionFilter
{
public:
  bool ignoresCollision(
      const dart::collision::CollisionObject*,
      const dart::collision::CollisionObject*) const override
  {
    return false;
  }
};

dart::dynamics::SkeletonPtr createBox(
    std::size_t index,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& color)
{
  auto boxSkel = dart::dynamics::Skeleton::create(
      "allocation_box_" + std::to_string(index));

  auto* boxBody
      = boxSkel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>().second;

  auto boxShape = std::make_shared<dart::dynamics::BoxShape>(size);
  auto* shapeNode = boxBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(boxShape);
  shapeNode->getVisualAspect()->setColor(color);
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.2);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  boxBody->getParentJoint()->setTransformFromParentBodyNode(tf);

  return boxSkel;
}

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create("allocation_ground");
  auto* groundBody
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  auto* groundShapeNode = groundBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(10.0, 10.0, kGroundThickness)));
  groundShapeNode->getVisualAspect()->setColor(dart::Color::LightGray());
  groundShapeNode->getDynamicsAspect()->setRestitutionCoeff(0.2);

  return ground;
}

dart::simulation::WorldPtr createStackedBoxesWorld(
    std::size_t dim,
    const dart::collision::CollisionDetectorPtr& collisionDetector)
{
  auto world = dart::simulation::World::create("step_allocation_boxes");
  world->setNumSimulationThreads(1u);
  world->setTimeStep(0.001);
  world->getConstraintSolver()->setCollisionDetector(collisionDetector);

  std::size_t index = 0u;
  const double horizontalSpacing = kBoxEdge + 0.05;
  const double baseOffset = (static_cast<double>(dim) - 1.0) * 0.5;
  for (std::size_t i = 0u; i < dim; ++i) {
    for (std::size_t j = 0u; j < dim; ++j) {
      for (std::size_t k = 0u; k < dim; ++k) {
        const double x
            = (static_cast<double>(i) - baseOffset) * horizontalSpacing;
        const double y
            = (static_cast<double>(j) - baseOffset) * horizontalSpacing;
        const double z = 0.5 * kGroundThickness + 0.5 * kBoxEdge
                         + static_cast<double>(k) * kBoxEdge;
        const Eigen::Vector3d position(x, y, z);
        const Eigen::Vector3d size(kBoxEdge, kBoxEdge, kBoxEdge);
        const Eigen::Vector3d color(
            static_cast<double>(i + 1u) / static_cast<double>(dim + 1u),
            static_cast<double>(j + 1u) / static_cast<double>(dim + 1u),
            static_cast<double>(k + 1u) / static_cast<double>(dim + 1u));
        world->addSkeleton(createBox(index++, position, size, color));
      }
    }
  }

  world->addSkeleton(createGround());

  return world;
}

dart::simulation::WorldPtr createFallingBoxWorld(const std::string& name)
{
  auto world = dart::simulation::World::create(name);
  world->setNumSimulationThreads(1u);
  world->setTimeStep(0.001);
  world->addSkeleton(createBox(
      0u,
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(0.2, 0.2, 0.2),
      Eigen::Vector3d(0.2, 0.4, 0.8)));
  return world;
}

dart::dynamics::SkeletonPtr createSoftBox(
    const std::string& name,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& size)
{
  auto skel = dart::dynamics::Skeleton::create(name);

  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties jointProps(
      name + "_joint");
  dart::dynamics::BodyNode::Properties bodyProps(
      dart::dynamics::BodyNode::AspectProperties(name + "_body"));
  bodyProps.mInertia.setMass(1.0);

  const auto softProperties
      = dart::dynamics::SoftBodyNodeHelper::makeBoxProperties(
          size,
          Eigen::Isometry3d::Identity(),
          Eigen::Vector3i(3, 3, 3),
          1.0,
          500.0,
          10.0,
          1.0);
  const dart::dynamics::SoftBodyNode::Properties softBodyProperties(
      bodyProps, softProperties);

  auto pair = skel->createJointAndBodyNodePair<
      dart::dynamics::FreeJoint,
      dart::dynamics::SoftBodyNode>(nullptr, jointProps, softBodyProperties);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  pair.first->setPositions(dart::dynamics::FreeJoint::convertToPositions(tf));

  return skel;
}

void expectWorldStateExactlyEqual(
    const dart::simulation::World& lhs, const dart::simulation::World& rhs)
{
  ASSERT_EQ(lhs.getNumSkeletons(), rhs.getNumSkeletons());

  for (std::size_t i = 0u; i < lhs.getNumSkeletons(); ++i) {
    const auto lhsSkeleton = lhs.getSkeleton(i);
    const auto rhsSkeleton = rhs.getSkeleton(i);
    ASSERT_NE(lhsSkeleton, nullptr);
    ASSERT_NE(rhsSkeleton, nullptr);

    const Eigen::VectorXd lhsPositions = lhsSkeleton->getPositions();
    const Eigen::VectorXd rhsPositions = rhsSkeleton->getPositions();
    ASSERT_EQ(lhsPositions.size(), rhsPositions.size());
    for (Eigen::Index j = 0; j < lhsPositions.size(); ++j) {
      EXPECT_EQ(lhsPositions[j], rhsPositions[j])
          << "position mismatch skeleton=" << i << " dof=" << j;
    }

    const Eigen::VectorXd lhsVelocities = lhsSkeleton->getVelocities();
    const Eigen::VectorXd rhsVelocities = rhsSkeleton->getVelocities();
    ASSERT_EQ(lhsVelocities.size(), rhsVelocities.size());
    for (Eigen::Index j = 0; j < lhsVelocities.size(); ++j) {
      EXPECT_EQ(lhsVelocities[j], rhsVelocities[j])
          << "velocity mismatch skeleton=" << i << " dof=" << j;
    }
  }
}

void installCountingDantzigSolver(
    const dart::simulation::WorldPtr& world,
    dart::test::CountingMemoryAllocator& allocator)
{
  auto* boxedSolver = dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(
      world->getConstraintSolver());
  ASSERT_NE(boxedSolver, nullptr);

  boxedSolver->setBoxedLcpSolver(
      std::make_shared<CountingDantzigBoxedLcpSolver>(allocator));
}

struct StepAllocationMeasurement
{
  dart::test::HeapAllocationSnapshot globalHeap;
  dart::test::RawHeapAllocationSnapshot rawHeap;
  dart::test::CountingMemoryAllocatorSnapshot countingAllocator;
  int warmupSteps = 0;
  int measuredSteps = 0;
  std::size_t lastStepContacts = 0u;
  std::size_t lastStepSoftSoftContacts = 0u;
  std::size_t maxOperatorNewPerStep = 0u;
  std::size_t maxRawMallocPerStep = 0u;
};

struct SoftSceneStats
{
  std::size_t softBodies = 0u;
  std::size_t pointMasses = 0u;
};

SoftSceneStats collectSoftSceneStats(const dart::simulation::WorldPtr& world)
{
  SoftSceneStats stats;
  if (!world)
    return stats;

  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (!skeleton)
      continue;

    stats.softBodies += skeleton->getNumSoftBodyNodes();
    for (std::size_t j = 0u; j < skeleton->getNumSoftBodyNodes(); ++j) {
      const auto* softBody = skeleton->getSoftBodyNode(j);
      if (softBody != nullptr)
        stats.pointMasses += softBody->getNumPointMasses();
    }
  }

  return stats;
}

std::size_t countSoftSoftContacts(
    const dart::collision::CollisionResult& collisionResult)
{
  std::size_t count = 0u;
  for (const auto& contact : collisionResult.getContacts()) {
    const auto bodyNode1 = contact.getBodyNodePtr1();
    const auto bodyNode2 = contact.getBodyNodePtr2();
    if (bodyNode1 != nullptr && bodyNode2 != nullptr
        && bodyNode1->asSoftBodyNode() != nullptr
        && bodyNode2->asSoftBodyNode() != nullptr) {
      ++count;
    }
  }

  return count;
}

StepAllocationMeasurement measureWorldStepsNow(
    const dart::simulation::WorldPtr& world,
    dart::test::CountingMemoryAllocator& allocator,
    int measuredSteps,
    int warmupSteps = 0,
    const std::function<void(int)>& beforeStep = {})
{
  // Opt-in allocation-site attribution: set DART_TEST_ALLOCATION_BACKTRACE
  // to dump aggregated backtraces of every measured operator-new call.
  const bool sampleBacktraces
      = std::getenv("DART_TEST_ALLOCATION_BACKTRACE") != nullptr;
  if (sampleBacktraces) {
    dart::test::clearAllocationBacktraces();
    dart::test::setAllocationBacktraceSamplingEnabled(true);
  }

  dart::test::ScopedHeapAllocationCounter globalCounter;
  dart::test::ScopedRawHeapAllocationCounter rawCounter;
  dart::test::ScopedCountingMemoryAllocatorCounter allocatorCounter(allocator);

  std::size_t maxOperatorNewPerStep = 0u;
  std::size_t maxRawMallocPerStep = 0u;
  for (int i = 0; i < measuredSteps; ++i) {
    const auto heapBefore = globalCounter.allocationCount();
    const auto rawBefore = rawCounter.allocationCount();
    if (beforeStep)
      beforeStep(i);
    world->step();
    maxOperatorNewPerStep = std::max(
        maxOperatorNewPerStep, globalCounter.allocationCount() - heapBefore);
    maxRawMallocPerStep = std::max(
        maxRawMallocPerStep, rawCounter.allocationCount() - rawBefore);
  }

  globalCounter.stop();
  rawCounter.stop();
  allocatorCounter.stop();

  if (sampleBacktraces) {
    dart::test::setAllocationBacktraceSamplingEnabled(false);
    dart::test::dumpAllocationBacktraces(std::cout, 25u);
  }

  return {
      globalCounter.snapshot(),
      rawCounter.snapshot(),
      allocatorCounter.snapshot(),
      warmupSteps,
      measuredSteps,
      world->getLastCollisionResult().getNumContacts(),
      countSoftSoftContacts(world->getLastCollisionResult()),
      maxOperatorNewPerStep,
      maxRawMallocPerStep};
}

StepAllocationMeasurement measureWorldStepAllocations(
    const dart::simulation::WorldPtr& world,
    dart::test::CountingMemoryAllocator& allocator)
{
  for (int i = 0; i < kWarmupSteps; ++i) {
    world->step();
  }

  return measureWorldStepsNow(world, allocator, kMeasuredSteps, kWarmupSteps);
}

std::string perStep(std::size_t count, int measuredSteps)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(3)
     << static_cast<double>(count) / static_cast<double>(measuredSteps);
  return os.str();
}

void recordProperty(const std::string& key, std::size_t value)
{
  ::testing::Test::RecordProperty(key, std::to_string(value));
}

void recordProperty(const std::string& key, const std::string& value)
{
  ::testing::Test::RecordProperty(key, value);
}

[[nodiscard]] const char* strictGlobalHeapGateSkipReason()
{
#if defined(DART_CODECOV)
  return "coverage instrumentation can allocate outside DART";
#elif defined(__SANITIZE_ADDRESS__)
  return "AddressSanitizer can allocate outside DART";
#elif defined(__has_feature)
  #if __has_feature(address_sanitizer)
  return "AddressSanitizer can allocate outside DART";
  #elif !defined(__linux__) || !defined(__GLIBC__)
  return "strict operator-new gate is supported on Linux glibc";
  #else
  return "";
  #endif
#elif !defined(__linux__) || !defined(__GLIBC__)
  return "strict operator-new gate is supported on Linux glibc";
#else
  return "";
#endif
}

void reportMeasurement(
    const std::string& label,
    const StepAllocationMeasurement& measurement,
    const std::string& note = "",
    bool requireContacts = true)
{
  const std::string prefix = label + "_";

  recordProperty(prefix + "boxes_per_side", kBoxesPerSide);
  recordProperty(prefix + "warmup_steps", measurement.warmupSteps);
  recordProperty(prefix + "measured_steps", measurement.measuredSteps);
  recordProperty(prefix + "last_step_contacts", measurement.lastStepContacts);
  recordProperty(
      prefix + "last_step_soft_soft_contacts",
      measurement.lastStepSoftSoftContacts);

  // Scene validity, not an allocation assertion: the baseline is only
  // meaningful if the measured window actually exercises contact solving.
  if (requireContacts) {
    EXPECT_GT(measurement.lastStepContacts, 0u)
        << label << " scene produced no contacts in the measured window";
  }
  recordProperty(
      prefix + "operator_new_count", measurement.globalHeap.allocationCount);
  recordProperty(
      prefix + "operator_new_max_per_step", measurement.maxOperatorNewPerStep);
  recordProperty(
      prefix + "raw_malloc_max_per_step", measurement.maxRawMallocPerStep);
  recordProperty(
      prefix + "operator_new_bytes", measurement.globalHeap.allocationBytes);
  recordProperty(
      prefix + "operator_new_count_per_step",
      perStep(
          measurement.globalHeap.allocationCount, measurement.measuredSteps));
  recordProperty(
      prefix + "operator_new_bytes_per_step",
      perStep(
          measurement.globalHeap.allocationBytes, measurement.measuredSteps));

  recordProperty(
      prefix + "raw_malloc_skipped",
      measurement.rawHeap.skipped ? "true" : "false");
  if (measurement.rawHeap.skipped) {
    recordProperty(
        prefix + "raw_malloc_skip_reason", measurement.rawHeap.skipReason);
  } else {
    recordProperty(
        prefix + "raw_malloc_count", measurement.rawHeap.allocationCount);
    recordProperty(
        prefix + "raw_malloc_bytes", measurement.rawHeap.allocationBytes);
    recordProperty(
        prefix + "raw_malloc_count_per_step",
        perStep(
            measurement.rawHeap.allocationCount, measurement.measuredSteps));
    recordProperty(
        prefix + "raw_malloc_bytes_per_step",
        perStep(
            measurement.rawHeap.allocationBytes, measurement.measuredSteps));
  }

  recordProperty(
      prefix + "counting_allocator_allocate_count",
      measurement.countingAllocator.allocationCount);
  recordProperty(
      prefix + "counting_allocator_allocate_bytes",
      measurement.countingAllocator.allocationBytes);
  recordProperty(
      prefix + "counting_allocator_deallocate_count",
      measurement.countingAllocator.deallocationCount);
  recordProperty(
      prefix + "counting_allocator_deallocate_bytes",
      measurement.countingAllocator.deallocationBytes);
  recordProperty(
      prefix + "counting_allocator_allocate_count_per_step",
      perStep(
          measurement.countingAllocator.allocationCount,
          measurement.measuredSteps));

  std::cout << "[StepAllocation] " << label
            << " boxes_per_side=" << kBoxesPerSide
            << " warmup_steps=" << measurement.warmupSteps
            << " measured_steps=" << measurement.measuredSteps
            << " last_step_contacts=" << measurement.lastStepContacts
            << " last_step_soft_soft_contacts="
            << measurement.lastStepSoftSoftContacts;
  if (!note.empty()) {
    std::cout << " note=\"" << note << "\"";
  }
  std::cout << '\n';
  std::cout << "  operator_new_count=" << measurement.globalHeap.allocationCount
            << " operator_new_bytes=" << measurement.globalHeap.allocationBytes
            << " operator_new_count_per_step="
            << perStep(
                   measurement.globalHeap.allocationCount,
                   measurement.measuredSteps)
            << " operator_new_bytes_per_step="
            << perStep(
                   measurement.globalHeap.allocationBytes,
                   measurement.measuredSteps)
            << '\n';
  if (measurement.rawHeap.skipped) {
    std::cout << "  raw_malloc_count=skipped raw_malloc_bytes=skipped reason=\""
              << measurement.rawHeap.skipReason << "\"\n";
  } else {
    std::cout << "  raw_malloc_count=" << measurement.rawHeap.allocationCount
              << " raw_malloc_bytes=" << measurement.rawHeap.allocationBytes
              << " raw_malloc_count_per_step="
              << perStep(
                     measurement.rawHeap.allocationCount,
                     measurement.measuredSteps)
              << " raw_malloc_bytes_per_step="
              << perStep(
                     measurement.rawHeap.allocationBytes,
                     measurement.measuredSteps)
              << '\n';
  }
  std::cout << "  counting_allocator_allocate_count="
            << measurement.countingAllocator.allocationCount
            << " counting_allocator_allocate_bytes="
            << measurement.countingAllocator.allocationBytes
            << " counting_allocator_deallocate_count="
            << measurement.countingAllocator.deallocationCount
            << " counting_allocator_deallocate_bytes="
            << measurement.countingAllocator.deallocationBytes
            << " counting_allocator_allocate_count_per_step="
            << perStep(
                   measurement.countingAllocator.allocationCount,
                   measurement.measuredSteps)
            << '\n';
}

dart::simulation::WorldPtr createStackedBoxesWorld(
    std::size_t dim,
    const dart::collision::CollisionDetectorPtr& collisionDetector,
    const dart::simulation::WorldConfig& config)
{
  auto world = dart::simulation::World::create(config);
  world->setNumSimulationThreads(1u);
  world->setTimeStep(0.001);
  world->getConstraintSolver()->setCollisionDetector(collisionDetector);

  std::size_t index = 0u;
  const double horizontalSpacing = kBoxEdge + 0.05;
  const double baseOffset = (static_cast<double>(dim) - 1.0) * 0.5;
  for (std::size_t i = 0u; i < dim; ++i) {
    for (std::size_t j = 0u; j < dim; ++j) {
      for (std::size_t k = 0u; k < dim; ++k) {
        const double x
            = (static_cast<double>(i) - baseOffset) * horizontalSpacing;
        const double y
            = (static_cast<double>(j) - baseOffset) * horizontalSpacing;
        const double z = 0.5 * kGroundThickness + 0.5 * kBoxEdge
                         + static_cast<double>(k) * kBoxEdge;
        const Eigen::Vector3d position(x, y, z);
        const Eigen::Vector3d size(kBoxEdge, kBoxEdge, kBoxEdge);
        const Eigen::Vector3d color(
            static_cast<double>(i + 1u) / static_cast<double>(dim + 1u),
            static_cast<double>(j + 1u) / static_cast<double>(dim + 1u),
            static_cast<double>(k + 1u) / static_cast<double>(dim + 1u));
        world->addSkeleton(createBox(index++, position, size, color));
      }
    }
  }

  world->addSkeleton(createGround());

  return world;
}

dart::simulation::WorldPtr createCountedStackedBoxesWorld(
    const std::string& name,
    const dart::collision::CollisionDetectorPtr& collisionDetector,
    dart::test::CountingMemoryAllocator& allocator)
{
  dart::simulation::WorldConfig config(name);
  config.baseAllocator = &allocator;
  return createStackedBoxesWorld(kBoxesPerSide, collisionDetector, config);
}

dart::simulation::WorldPtr createCountedNativeSoftBoxOnGroundWorld(
    const std::string& name, dart::test::CountingMemoryAllocator& allocator)
{
  dart::simulation::WorldConfig config(name);
  config.collisionDetector = dart::simulation::CollisionDetectorType::Dart;
  config.baseAllocator = &allocator;
  config.freeListInitialAllocation = 4u * 1024u * 1024u;
  config.frameScratchInitialCapacity = 1024u * 1024u;

  auto world = dart::simulation::World::create(config);
  world->setNumSimulationThreads(1u);
  world->setTimeStep(0.001);
  world->setGravity(0.0, 0.0, -9.81);

  world->addSkeleton(createGround());
  world->addSkeleton(createSoftBox(
      name + "_soft_box",
      Eigen::Vector3d(0.0, 0.0, 0.22),
      Eigen::Vector3d(0.4, 0.4, 0.4)));

  return world;
}

dart::simulation::WorldPtr createCountedNativeSoftStackWorld(
    const std::string& name, dart::test::CountingMemoryAllocator& allocator)
{
  dart::simulation::WorldConfig config(name);
  config.collisionDetector = dart::simulation::CollisionDetectorType::Dart;
  config.baseAllocator = &allocator;
  config.freeListInitialAllocation = 8u * 1024u * 1024u;
  config.frameScratchInitialCapacity = 2u * 1024u * 1024u;

  auto world = dart::simulation::World::create(config);
  world->setNumSimulationThreads(1u);
  world->setTimeStep(0.001);
  world->setGravity(0.0, 0.0, -9.81);

  world->addSkeleton(createGround());
  world->addSkeleton(createSoftBox(
      name + "_lower_soft_box",
      Eigen::Vector3d(0.0, 0.0, 0.22),
      Eigen::Vector3d(0.4, 0.4, 0.4)));
  world->addSkeleton(createSoftBox(
      name + "_upper_soft_box",
      Eigen::Vector3d(0.0, 0.0, 0.58),
      Eigen::Vector3d(0.4, 0.4, 0.4)));

  return world;
}

dart::simulation::WorldPtr createCountedNativeSoftSkelWorld(
    const std::string& name,
    const std::string& uri,
    dart::test::CountingMemoryAllocator& allocator)
{
  const auto sourceWorld = dart::utils::SkelParser::readWorld(uri);
  if (!sourceWorld)
    return nullptr;

  dart::simulation::WorldConfig config(name);
  config.collisionDetector = dart::simulation::CollisionDetectorType::Dart;
  config.baseAllocator = &allocator;
  config.freeListInitialAllocation = 64u * 1024u * 1024u;
  config.frameScratchInitialCapacity = 8u * 1024u * 1024u;

  auto world = dart::simulation::World::create(config);
  world->setNumSimulationThreads(1u);
  world->setTimeStep(sourceWorld->getTimeStep());
  world->setGravity(sourceWorld->getGravity());

  std::vector<dart::dynamics::SkeletonPtr> skeletons;
  skeletons.reserve(sourceWorld->getNumSkeletons());
  for (std::size_t i = 0u; i < sourceWorld->getNumSkeletons(); ++i) {
    const auto skeleton = sourceWorld->getSkeleton(i);
    if (skeleton)
      skeletons.push_back(skeleton);
  }

  for (const auto& skeleton : skeletons) {
    sourceWorld->removeSkeleton(skeleton);
    world->addSkeleton(skeleton);
  }

  return world;
}

void enableAdaptiveContactActivation(const dart::simulation::WorldPtr& world)
{
  ASSERT_TRUE(world != nullptr);
  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (!skeleton)
      continue;

    for (std::size_t j = 0u; j < skeleton->getNumSoftBodyNodes(); ++j) {
      auto* softBody = skeleton->getSoftBodyNode(j);
      ASSERT_TRUE(softBody != nullptr);
      softBody->setAdaptiveContactActivationEnabled(true);
    }
  }
}

enum class PreparationMode
{
  Explicit,
  Implicit,
};

StepAllocationMeasurement measurePreparedGateScene(
    const std::string& name,
    const dart::collision::CollisionDetectorPtr& collisionDetector,
    PreparationMode mode,
    dart::test::CountingMemoryAllocator& allocator)
{
  auto world
      = createCountedStackedBoxesWorld(name, collisionDetector, allocator);

  if (mode == PreparationMode::Explicit) {
    world->enterSimulationMode();
  } else {
    EXPECT_FALSE(world->isInSimulationMode());
    world->step();
  }
  EXPECT_TRUE(world->isInSimulationMode());

  return measureWorldStepsNow(world, allocator, 1);
}

StepAllocationMeasurement measurePreparedSoftGateScene(
    const std::string& name,
    PreparationMode mode,
    dart::test::CountingMemoryAllocator& allocator)
{
  auto world = createCountedNativeSoftBoxOnGroundWorld(name, allocator);

  if (mode == PreparationMode::Explicit) {
    world->enterSimulationMode();
  } else {
    EXPECT_FALSE(world->isInSimulationMode());
    world->step();
  }
  EXPECT_TRUE(world->isInSimulationMode());

  return measureWorldStepsNow(world, allocator, 1);
}

StepAllocationMeasurement measureNativeSoftStackSteadyState(
    const std::string& name, dart::test::CountingMemoryAllocator& allocator)
{
  auto world = createCountedNativeSoftStackWorld(name, allocator);
  world->enterSimulationMode();
  EXPECT_TRUE(world->isInSimulationMode());

  return measureWorldStepAllocations(world, allocator);
}

StepAllocationMeasurement measureNativeSoftSkelSteadyState(
    const dart::simulation::WorldPtr& world,
    dart::test::CountingMemoryAllocator& allocator,
    int warmupSteps = kWarmupSteps)
{
  world->enterSimulationMode();
  EXPECT_TRUE(world->isInSimulationMode());

  for (int i = 0; i < warmupSteps; ++i) {
    world->step();
  }

  return measureWorldStepsNow(world, allocator, kMeasuredSteps, warmupSteps);
}

StepAllocationMeasurement measureNativeSoftAdaptiveActivationSteadyState(
    const std::string& label, dart::test::CountingMemoryAllocator& allocator)
{
  auto world = createCountedNativeSoftSkelWorld(
      label, "dart://sample/skel/soft_cubes.skel", allocator);
  enableAdaptiveContactActivation(world);
  return measureNativeSoftSkelSteadyState(
      world, allocator, kSoftSkelContactWarmupSteps);
}

// The stacked boxes with CollisionOption::maxNumContacts at half their contact
// demand, so ConstraintSolver trims the contacts to the budget every step
// (#3056). Deactivation is off so no step skips the solver.
StepAllocationMeasurement measureSaturatedContactBudgetSteadyState(
    const std::string& name, dart::test::CountingMemoryAllocator& allocator)
{
  auto world = createCountedStackedBoxesWorld(
      name, dart::collision::DARTCollisionDetector::create(), allocator);
  dart::simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);
  for (int i = 0; i < kWarmupSteps; ++i) {
    world->step();
  }

  const std::size_t demand = world->getLastCollisionResult().getNumContacts();
  auto& option = world->getConstraintSolver()->getCollisionOption();
  option.maxNumContacts = demand / 2u;
  EXPECT_GT(option.maxNumContacts, 1u);
  for (int i = 0; i < kWarmupSteps; ++i) {
    world->step();
  }

  const auto measurement = measureWorldStepsNow(
      world, allocator, kMeasuredSteps, 2 * kWarmupSteps);
  EXPECT_EQ(measurement.lastStepContacts, option.maxNumContacts)
      << "the scene no longer exceeds its contact budget";
  return measurement;
}

::testing::AssertionResult hasNoGlobalHeapAllocations(
    const StepAllocationMeasurement& measurement)
{
  if (measurement.globalHeap.allocationCount != 0u
      || measurement.globalHeap.allocationBytes != 0u) {
    return ::testing::AssertionFailure()
           << "operator-new allocations: count="
           << measurement.globalHeap.allocationCount
           << " bytes=" << measurement.globalHeap.allocationBytes;
  }

  return ::testing::AssertionSuccess();
}

::testing::AssertionResult hasNoRawHeapAllocations(
    const StepAllocationMeasurement& measurement)
{
  if (measurement.rawHeap.skipped) {
    return ::testing::AssertionFailure()
           << "raw malloc counter skipped: " << measurement.rawHeap.skipReason;
  }

  if (measurement.rawHeap.allocationCount != 0u
      || measurement.rawHeap.allocationBytes != 0u) {
    return ::testing::AssertionFailure()
           << "raw malloc-family allocations: count="
           << measurement.rawHeap.allocationCount
           << " bytes=" << measurement.rawHeap.allocationBytes;
  }

  return ::testing::AssertionSuccess();
}

::testing::AssertionResult hasNoCountingAllocatorGrowth(
    const StepAllocationMeasurement& measurement)
{
  if (measurement.countingAllocator.allocationCount != 0u
      || measurement.countingAllocator.allocationBytes != 0u) {
    return ::testing::AssertionFailure()
           << "World base-allocator growth: count="
           << measurement.countingAllocator.allocationCount
           << " bytes=" << measurement.countingAllocator.allocationBytes;
  }

  return ::testing::AssertionSuccess();
}

void expectNoGlobalHeapAllocationsWhenReliable(
    const std::string& label, const StepAllocationMeasurement& measurement)
{
  const char* skipReason = strictGlobalHeapGateSkipReason();
  if (skipReason[0] != '\0') {
    recordProperty(label + "_operator_new_strict_gate_skipped", "true");
    recordProperty(label + "_operator_new_strict_gate_skip_reason", skipReason);
    std::cout << "[StepAllocation] " << label
              << " operator_new_strict_gate=skipped reason=\"" << skipReason
              << "\"\n";
    return;
  }

  EXPECT_TRUE(hasNoGlobalHeapAllocations(measurement));
}

void expectNativeGlobalAndBaseAllocatorGate(
    PreparationMode mode, const std::string& label)
{
  dart::test::CountingMemoryAllocator allocator;
  const auto measurement = measurePreparedGateScene(
      label, dart::collision::DARTCollisionDetector::create(), mode, allocator);
  reportMeasurement(label, measurement);
  EXPECT_GT(measurement.lastStepContacts, 0u);
  expectNoGlobalHeapAllocationsWhenReliable(label, measurement);
  EXPECT_TRUE(hasNoCountingAllocatorGrowth(measurement));
}

void expectNativeSoftGlobalAndBaseAllocatorGate(
    PreparationMode mode, const std::string& label)
{
  dart::test::CountingMemoryAllocator allocator;
  const auto measurement = measurePreparedSoftGateScene(label, mode, allocator);
  reportMeasurement(label, measurement);
  EXPECT_GT(measurement.lastStepContacts, 0u);
  expectNoGlobalHeapAllocationsWhenReliable(label, measurement);
  EXPECT_TRUE(hasNoCountingAllocatorGrowth(measurement));
}

void expectNativeRawHeapGate(PreparationMode mode, const std::string& label)
{
  dart::test::CountingMemoryAllocator allocator;
  const auto measurement = measurePreparedGateScene(
      label, dart::collision::DARTCollisionDetector::create(), mode, allocator);
  reportMeasurement(label, measurement);
  EXPECT_GT(measurement.lastStepContacts, 0u);
  if (measurement.rawHeap.skipped) {
    recordProperty(label + "_raw_malloc_skipped", "true");
    recordProperty(
        label + "_raw_malloc_skip_reason", measurement.rawHeap.skipReason);
    GTEST_SKIP() << measurement.rawHeap.skipReason;
  }
  EXPECT_TRUE(hasNoRawHeapAllocations(measurement));
}

void expectNativeSoftRawHeapGate(PreparationMode mode, const std::string& label)
{
  dart::test::CountingMemoryAllocator allocator;
  const auto measurement = measurePreparedSoftGateScene(label, mode, allocator);
  reportMeasurement(label, measurement);
  EXPECT_GT(measurement.lastStepContacts, 0u);
  if (measurement.rawHeap.skipped) {
    recordProperty(label + "_raw_malloc_skipped", "true");
    recordProperty(
        label + "_raw_malloc_skip_reason", measurement.rawHeap.skipReason);
    GTEST_SKIP() << measurement.rawHeap.skipReason;
  }
  EXPECT_TRUE(hasNoRawHeapAllocations(measurement));
}

void expectNativeSoftStackGlobalAndBaseAllocatorGate(const std::string& label)
{
  dart::test::CountingMemoryAllocator allocator;
  const auto measurement = measureNativeSoftStackSteadyState(label, allocator);
  reportMeasurement(label, measurement);
  EXPECT_GT(measurement.lastStepContacts, 0u);
  EXPECT_GT(measurement.lastStepSoftSoftContacts, 0u);
  expectNoGlobalHeapAllocationsWhenReliable(label, measurement);
  EXPECT_TRUE(hasNoCountingAllocatorGrowth(measurement));
}

void expectNativeSoftStackRawHeapGate(const std::string& label)
{
  dart::test::CountingMemoryAllocator allocator;
  const auto measurement = measureNativeSoftStackSteadyState(label, allocator);
  reportMeasurement(label, measurement);
  EXPECT_GT(measurement.lastStepContacts, 0u);
  EXPECT_GT(measurement.lastStepSoftSoftContacts, 0u);
  if (measurement.rawHeap.skipped) {
    recordProperty(label + "_raw_malloc_skipped", "true");
    recordProperty(
        label + "_raw_malloc_skip_reason", measurement.rawHeap.skipReason);
    GTEST_SKIP() << measurement.rawHeap.skipReason;
  }
  EXPECT_TRUE(hasNoRawHeapAllocations(measurement));
}

void expectNativeSoftSkelGlobalAndBaseAllocatorGate(
    const std::string& label,
    const std::string& uri,
    bool requireContacts = false,
    bool requireSoftSoftContacts = false,
    int warmupSteps = kWarmupSteps)
{
  dart::test::CountingMemoryAllocator allocator;
  const auto world = createCountedNativeSoftSkelWorld(label, uri, allocator);
  ASSERT_NE(nullptr, world);

  const auto stats = collectSoftSceneStats(world);
  recordProperty(label + "_soft_bodies", stats.softBodies);
  recordProperty(label + "_point_masses", stats.pointMasses);
  ASSERT_GT(stats.softBodies, 0u);
  ASSERT_GT(stats.pointMasses, 0u);

  const auto measurement
      = measureNativeSoftSkelSteadyState(world, allocator, warmupSteps);
  reportMeasurement(
      label,
      measurement,
      "native transferred SKEL soft scene",
      requireContacts);
  if (requireSoftSoftContacts) {
    EXPECT_GT(measurement.lastStepSoftSoftContacts, 0u);
  }
  expectNoGlobalHeapAllocationsWhenReliable(label, measurement);
  EXPECT_TRUE(hasNoCountingAllocatorGrowth(measurement));
}

void expectNativeSoftSkelRawHeapGate(
    const std::string& label,
    const std::string& uri,
    bool requireContacts = false,
    bool requireSoftSoftContacts = false,
    int warmupSteps = kWarmupSteps)
{
  dart::test::CountingMemoryAllocator allocator;
  const auto world = createCountedNativeSoftSkelWorld(label, uri, allocator);
  ASSERT_NE(nullptr, world);

  const auto stats = collectSoftSceneStats(world);
  recordProperty(label + "_soft_bodies", stats.softBodies);
  recordProperty(label + "_point_masses", stats.pointMasses);
  ASSERT_GT(stats.softBodies, 0u);
  ASSERT_GT(stats.pointMasses, 0u);

  const auto measurement
      = measureNativeSoftSkelSteadyState(world, allocator, warmupSteps);
  reportMeasurement(
      label,
      measurement,
      "native transferred SKEL soft scene",
      requireContacts);
  if (requireSoftSoftContacts) {
    EXPECT_GT(measurement.lastStepSoftSoftContacts, 0u);
  }
  if (measurement.rawHeap.skipped) {
    recordProperty(label + "_raw_malloc_skipped", "true");
    recordProperty(
        label + "_raw_malloc_skip_reason", measurement.rawHeap.skipReason);
    GTEST_SKIP() << measurement.rawHeap.skipReason;
  }
  EXPECT_TRUE(hasNoRawHeapAllocations(measurement));
}

void expectExternalBackendBaseAllocatorGate(
    const std::string& label,
    const dart::collision::CollisionDetectorPtr& collisionDetector,
    PreparationMode mode)
{
  dart::test::CountingMemoryAllocator allocator;
  const auto measurement
      = measurePreparedGateScene(label, collisionDetector, mode, allocator);
  reportMeasurement(
      label,
      measurement,
      "global/raw counters include collision-backend-internal allocations");
  EXPECT_GT(measurement.lastStepContacts, 0u);
  EXPECT_TRUE(hasNoCountingAllocatorGrowth(measurement));
}

StepAllocationMeasurement measureScene(
    const dart::collision::CollisionDetectorPtr& detector)
{
  dart::test::CountingMemoryAllocator allocator;
  auto world = createStackedBoxesWorld(kBoxesPerSide, detector);
  installCountingDantzigSolver(world, allocator);
  return measureWorldStepAllocations(world, allocator);
}

class AllocationGateConstraintSolver final
  : public dart::constraint::BoxedLcpConstraintSolver
{
public:
  std::size_t getNumConstrainedGroups() const
  {
    return mConstrainedGroups.size();
  }

  bool canSolveInParallel() const
  {
    return canSolveConstrainedGroupsInParallel();
  }
};

dart::dynamics::SkeletonPtr createAllocationGateBox(
    const std::string& name,
    const Eigen::Vector3d& pos,
    double edge,
    double restitution = 0.0)
{
  auto skel = dart::dynamics::Skeleton::create(name);
  auto* bn
      = skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>().second;
  auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d::Constant(edge));
  auto* sn = bn->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  sn->getDynamicsAspect()->setRestitutionCoeff(restitution);
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = pos;
  skel->getJoint(0)->setPositions(
      dart::dynamics::FreeJoint::convertToPositions(tf));
  return skel;
}

dart::dynamics::SkeletonPtr createAllocationGateGround(double width = 20.0)
{
  auto skel = dart::dynamics::Skeleton::create("ground");
  skel->setMobile(false);
  auto* bn
      = skel->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  auto* sn = bn->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(width, width, 1.0)));
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation().z() = -0.5;
  sn->setRelativeTransform(tf);
  return skel;
}

dart::simulation::WorldPtr createAllocationGateWorld(
    const dart::collision::CollisionDetectorPtr& detector,
    bool sleeping,
    std::size_t threads = 1u)
{
  auto world = dart::simulation::World::create("transient");
  world->setNumSimulationThreads(threads);
  if (threads > 1u) {
    world->setConstraintSolver(
        std::make_unique<AllocationGateConstraintSolver>());
  }
  world->setTimeStep(0.001);
  world->getConstraintSolver()->setCollisionDetector(detector);
  auto options = world->getDeactivationOptions();
  options.mEnabled = sleeping;
  world->setDeactivationOptions(options);
  world->addSkeleton(createAllocationGateGround(threads > 1u ? 40.0 : 20.0));
  return world;
}

void addGridBoxes(
    const dart::simulation::WorldPtr& world, int layers, int width = 3)
{
  for (int i = 0; i < width; ++i)
    for (int j = 0; j < width; ++j)
      for (int k = 0; k < layers; ++k)
        world->addSkeleton(createAllocationGateBox(
            "b" + std::to_string(i) + std::to_string(j) + std::to_string(k),
            Eigen::Vector3d(1.2 * i, 1.2 * j, 0.25 + 0.5 * k),
            0.5));
}

std::size_t countResting(const dart::simulation::WorldPtr& world)
{
  std::size_t n = 0;
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto s = world->getSkeleton(i);
    if (s->isMobile() && s->isResting())
      ++n;
  }
  return n;
}

// gz-physics look-alikes: BodyNodeCollisionFilter subclass with map lookups,
// ContactSurfaceHandler subclass deferring to the base with a callback.
class AllocationGateBitmaskFilter
  : public dart::collision::BodyNodeCollisionFilter
{
public:
  bool ignoresCollision(
      const dart::collision::CollisionObject* a,
      const dart::collision::CollisionObject* b) const override
  {
    if (BodyNodeCollisionFilter::ignoresCollision(a, b))
      return true;
    const auto i1 = mMask.find(a->getShapeFrame()->asShapeNode());
    const auto i2 = mMask.find(b->getShapeFrame()->asShapeNode());
    if (i1 != mMask.end() && i2 != mMask.end())
      return !(i1->second & i2->second);
    return false;
  }
  std::unordered_map<const dart::dynamics::ShapeNode*, unsigned> mMask;
};

class AllocationGateCallbackHandler
  : public dart::constraint::ContactSurfaceHandler
{
public:
  dart::constraint::ContactSurfaceParams createParams(
      const dart::collision::Contact& contact,
      std::size_t numContacts) const override
  {
    mCalls.fetch_add(1u, std::memory_order_relaxed);
    auto params = ContactSurfaceHandler::createParams(contact, numContacts);
    if (mCallback)
      mCallback(params);
    return params;
  }
  std::function<void(dart::constraint::ContactSurfaceParams&)> mCallback;
  mutable std::atomic<std::size_t> mCalls{0u};
};

// A flat JSON object keeps the allocation ratchet readable without adding a
// JSON dependency to this test. Counts are upper bounds per measured step;
// decreases belong in the same change that removes the allocations.
std::map<std::string, std::size_t> readAllocationGateBudgets()
{
  std::ifstream stream(DART_ROOT_PATH
                       "tests/integration/step_allocation_ratchet.json");
  std::map<std::string, std::size_t> budgets;
  char delimiter = '\0';
  if (!(stream >> delimiter) || delimiter != '{')
    throw std::runtime_error("Cannot read step allocation ratchet JSON");

  while (stream >> std::ws && stream.peek() != '}') {
    std::string key;
    std::size_t value = 0u;
    if (stream.peek() != '"' || !(stream >> std::quoted(key) >> delimiter)
        || delimiter != ':' || !(stream >> value)
        || !budgets.emplace(key, value).second || !(stream >> delimiter)
        || (delimiter != ',' && delimiter != '}')) {
      throw std::runtime_error("Invalid step allocation ratchet JSON");
    }
    if (delimiter == '}') {
      stream.unget();
      break;
    }
  }
  if (!(stream >> delimiter) || delimiter != '}'
      || (stream >> std::ws && !stream.eof())) {
    throw std::runtime_error("Invalid step allocation ratchet JSON ending");
  }
  return budgets;
}

void expectAllocationGateBudget(
    const std::string& row,
    const StepAllocationMeasurement& measurement,
    bool strict = true)
{
  reportMeasurement(row, measurement, "Z1a allocation gate", false);
  ASSERT_FALSE(measurement.rawHeap.skipped) << measurement.rawHeap.skipReason;
  if (strict) {
    EXPECT_EQ(measurement.globalHeap.allocationCount, 0u) << row;
    EXPECT_EQ(measurement.rawHeap.allocationCount, 0u) << row;
  }
  static const auto budgets = readAllocationGateBudgets();
  for (const auto& metric :
       {std::make_pair("operator_new", measurement.maxOperatorNewPerStep),
        std::make_pair("raw_malloc", measurement.maxRawMallocPerStep)}) {
    const auto key = row + "_" + metric.first + "_per_step";
    const auto found = budgets.find(key);
    ASSERT_NE(found, budgets.end())
        << "Missing allocation ratchet entry " << key;
    if (strict) {
      ASSERT_EQ(found->second, 0u) << key << " must remain a strict zero gate";
    }
    EXPECT_LE(metric.second, found->second)
        << key << " increased; lower budgets when removing allocations";
  }
}

} // namespace

TEST(StepAllocation, ScopedHeapAllocationCounterDetectsOperatorNew)
{
  dart::test::ScopedHeapAllocationCounter counter;
  void* allocation = ::operator new(128u);
  g_allocationSink = allocation;
  counter.stop();

  EXPECT_GT(counter.allocationCount(), 0u);
  EXPECT_GE(counter.allocationBytes(), 128u);

  ::operator delete(allocation);
  g_allocationSink = nullptr;
}

TEST(StepAllocation, RawHeapAllocationCounterDetectsMallocWhenAvailable)
{
  dart::test::ScopedRawHeapAllocationCounter counter;
  void* allocation = std::malloc(128u);
  ASSERT_NE(allocation, nullptr);
  // Escape the pointer so builtin-aware optimizers cannot elide the
  // malloc/free pair, which would make the interposer miss the allocation.
  g_allocationSink = allocation;
  std::free(allocation);
  g_allocationSink = nullptr;
  counter.stop();

  if (counter.skipped()) {
    GTEST_SKIP() << counter.snapshot().skipReason;
  }

  EXPECT_GT(counter.allocationCount(), 0u);
  EXPECT_GE(counter.allocationBytes(), 128u);
}

TEST(StepAllocation, CountingMemoryAllocatorDetectsAllocation)
{
  dart::test::CountingMemoryAllocator allocator;
  dart::test::ScopedCountingMemoryAllocatorCounter counter(allocator);
  void* allocation = allocator.allocate(256u);
  ASSERT_NE(allocation, nullptr);
  allocator.deallocate(allocation, 256u);
  counter.stop();

  const auto snapshot = counter.snapshot();
  EXPECT_EQ(snapshot.allocationCount, 1u);
  EXPECT_EQ(snapshot.allocationBytes, 256u);
  EXPECT_EQ(snapshot.deallocationCount, 1u);
  EXPECT_EQ(snapshot.deallocationBytes, 256u);
}

TEST(StepAllocation, AllocationGateRejectsInjectedAllocationMeasurement)
{
  StepAllocationMeasurement measurement;
  measurement.measuredSteps = 1;

  measurement.globalHeap.allocationCount = 1u;
  measurement.globalHeap.allocationBytes = 8u;
  EXPECT_FALSE(hasNoGlobalHeapAllocations(measurement));

  measurement.globalHeap = {};
  measurement.rawHeap.allocationCount = 1u;
  measurement.rawHeap.allocationBytes = 8u;
  EXPECT_FALSE(hasNoRawHeapAllocations(measurement));

  measurement.rawHeap = {};
  measurement.countingAllocator.allocationCount = 1u;
  measurement.countingAllocator.allocationBytes = 8u;
  EXPECT_FALSE(hasNoCountingAllocatorGrowth(measurement));
}

// The DOF accessors pass their own name down for error messages, which must
// not cost a std::string per call. The long-named accessors checked here
// exceed any small-string buffer, so a std::string would heap-allocate. The
// returned Eigen vectors use malloc, which this counter does not see.
TEST(StepAllocation, MetaSkeletonDofAccessorsHaveNoOperatorNewAllocations)
{
  const char* skipReason = strictGlobalHeapGateSkipReason();
  if (skipReason[0] != '\0') {
    GTEST_SKIP() << skipReason;
  }

  auto skeleton = dart::dynamics::Skeleton::create();
  skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  const std::vector<std::size_t> indices{0u, 5u};
  Eigen::VectorXd all;
  Eigen::VectorXd some;

  dart::test::ScopedHeapAllocationCounter counter;
  all = skeleton->getAccelerationLowerLimits();
  some = skeleton->getAccelerationLowerLimits(indices);
  skeleton->setAccelerationLowerLimits(all);
  skeleton->setAccelerationLowerLimits(indices, some);
  skeleton->setAccelerationLowerLimit(
      0u, skeleton->getAccelerationLowerLimit(0u));
  counter.stop();

  EXPECT_EQ(counter.allocationCount(), 0u);
  EXPECT_EQ(all.size(), 6);
  EXPECT_EQ(some.size(), 2);
}

TEST(
    StepAllocation, NativeExplicitFirstPostBakeHasNoGlobalOrBaseAllocatorGrowth)
{
  expectNativeGlobalAndBaseAllocatorGate(
      PreparationMode::Explicit, "native_dart_explicit_first_post_bake_gate");
}

TEST(StepAllocation, DartThreadedReversedRigidDispatchRetainsScratch)
{
  constexpr std::size_t kNumSpheres = 32u;
  constexpr std::size_t kWarmupQueries = 20u;
  constexpr std::size_t kMeasuredQueries = 50u;

  auto detector = dart::collision::DARTCollisionDetector::create();
  detector->setNumCollisionThreads(4u);
  auto group = detector->createCollisionGroup();

  std::vector<dart::dynamics::SimpleFramePtr> frames;
  frames.reserve(kNumSpheres + 1u);

  auto plane = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  plane->setShape(std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d::UnitZ(), 0.0));
  group->addShapeFrame(plane.get());
  frames.push_back(plane);

  const auto sphereShape = std::make_shared<dart::dynamics::SphereShape>(1.0);
  for (std::size_t i = 0u; i < kNumSpheres; ++i) {
    auto sphere = dart::dynamics::SimpleFrame::createShared(
        dart::dynamics::Frame::World());
    sphere->setShape(sphereShape);
    sphere->setTranslation(
        Eigen::Vector3d(3.0 * static_cast<double>(i), 0.0, 0.999));
    group->addShapeFrame(sphere.get());
    frames.push_back(sphere);
  }

  const dart::collision::CollisionOption option(true, 1000u);
  dart::collision::CollisionResult result;
  for (std::size_t i = 0u; i < kWarmupQueries; ++i)
    ASSERT_TRUE(group->collide(option, &result));

  bool allQueriesCollided = true;
  dart::test::ScopedHeapAllocationCounter globalCounter;
  dart::test::ScopedRawHeapAllocationCounter rawCounter;
  for (std::size_t i = 0u; i < kMeasuredQueries; ++i)
    allQueriesCollided = group->collide(option, &result) && allQueriesCollided;
  globalCounter.stop();
  rawCounter.stop();

  ASSERT_TRUE(allQueriesCollided);
  ASSERT_EQ(kNumSpheres, result.getNumContacts());

  StepAllocationMeasurement measurement;
  measurement.globalHeap = globalCounter.snapshot();
  measurement.rawHeap = rawCounter.snapshot();
  measurement.measuredSteps = static_cast<int>(kMeasuredQueries);
  measurement.lastStepContacts = result.getNumContacts();
  reportMeasurement(
      "dart_threaded_reversed_rigid_dispatch", measurement, "", true);
  expectNoGlobalHeapAllocationsWhenReliable(
      "dart_threaded_reversed_rigid_dispatch", measurement);
  if (!measurement.rawHeap.skipped) {
    EXPECT_TRUE(hasNoRawHeapAllocations(measurement));
  }
}

TEST(StepAllocation, NativeImplicitSecondStepHasNoGlobalOrBaseAllocatorGrowth)
{
  expectNativeGlobalAndBaseAllocatorGate(
      PreparationMode::Implicit, "native_dart_implicit_second_step_gate");
}

TEST(
    StepAllocation,
    NativeContactHandlerExplicitFirstPostBakeHasNoGlobalOrBaseAllocatorGrowth)
{
  // With a user contact surface handler (gz-physics installs one), every step
  // builds its contact constraints anew from a process-wide pool, so the
  // preparation must grow that pool for a whole step without running the
  // handler. The pool keeps what earlier tests in this process grew it to, so
  // this gate relies on them not having needed more than this scene does.
  const std::string label = "native_dart_contact_handler_first_post_bake_gate";
  dart::test::CountingMemoryAllocator allocator;
  auto world = createCountedStackedBoxesWorld(
      label, dart::collision::DARTCollisionDetector::create(), allocator);
  world->getConstraintSolver()->addContactSurfaceHandler(
      std::make_shared<dart::constraint::ContactSurfaceHandler>());
  world->enterSimulationMode();

  const auto measurement = measureWorldStepsNow(world, allocator, 1);
  reportMeasurement(label, measurement);
  EXPECT_GT(measurement.lastStepContacts, 0u);
  expectNoGlobalHeapAllocationsWhenReliable(label, measurement);
  EXPECT_TRUE(hasNoCountingAllocatorGrowth(measurement));
}

TEST(
    StepAllocation,
    NativeSoftExplicitFirstPostBakeHasNoGlobalOrBaseAllocatorGrowth)
{
  expectNativeSoftGlobalAndBaseAllocatorGate(
      PreparationMode::Explicit,
      "native_dart_soft_explicit_first_post_bake_gate");
}

TEST(
    StepAllocation,
    NativeSoftImplicitSecondStepHasNoGlobalOrBaseAllocatorGrowth)
{
  expectNativeSoftGlobalAndBaseAllocatorGate(
      PreparationMode::Implicit, "native_dart_soft_implicit_second_step_gate");
}

TEST(StepAllocation, NativeExplicitFirstPostBakeHasNoRawMallocWhenAvailable)
{
  expectNativeRawHeapGate(
      PreparationMode::Explicit,
      "native_dart_explicit_first_post_bake_raw_gate");
}

TEST(StepAllocation, NativeImplicitSecondStepHasNoRawMallocWhenAvailable)
{
  expectNativeRawHeapGate(
      PreparationMode::Implicit, "native_dart_implicit_second_step_raw_gate");
}

TEST(StepAllocation, NativeSoftExplicitFirstPostBakeHasNoRawMallocWhenAvailable)
{
  expectNativeSoftRawHeapGate(
      PreparationMode::Explicit,
      "native_dart_soft_explicit_first_post_bake_raw_gate");
}

TEST(StepAllocation, NativeSoftImplicitSecondStepHasNoRawMallocWhenAvailable)
{
  expectNativeSoftRawHeapGate(
      PreparationMode::Implicit,
      "native_dart_soft_implicit_second_step_raw_gate");
}

TEST(StepAllocation, NativeSoftStackSteadyStateHasNoGlobalOrBaseAllocatorGrowth)
{
  expectNativeSoftStackGlobalAndBaseAllocatorGate(
      "native_dart_soft_stack_steady_state_gate");
}

TEST(StepAllocation, NativeSoftStackSteadyStateHasNoRawMallocWhenAvailable)
{
  expectNativeSoftStackRawHeapGate(
      "native_dart_soft_stack_steady_state_raw_gate");
}

TEST(
    StepAllocation,
    NativeSaturatedContactBudgetSteadyStateHasNoGlobalOrBaseAllocatorGrowth)
{
  const std::string label = "native_dart_saturated_contact_budget_gate";
  dart::test::CountingMemoryAllocator allocator;
  const auto measurement
      = measureSaturatedContactBudgetSteadyState(label, allocator);
  reportMeasurement(label, measurement, "contact demand above the budget");
  expectNoGlobalHeapAllocationsWhenReliable(label, measurement);
  EXPECT_TRUE(hasNoCountingAllocatorGrowth(measurement));
}

TEST(
    StepAllocation,
    NativeSaturatedContactBudgetSteadyStateHasNoRawMallocWhenAvailable)
{
  const std::string label = "native_dart_saturated_contact_budget_raw_gate";
  dart::test::CountingMemoryAllocator allocator;
  const auto measurement
      = measureSaturatedContactBudgetSteadyState(label, allocator);
  reportMeasurement(label, measurement, "contact demand above the budget");
  if (measurement.rawHeap.skipped) {
    recordProperty(label + "_raw_malloc_skipped", "true");
    recordProperty(
        label + "_raw_malloc_skip_reason", measurement.rawHeap.skipReason);
    GTEST_SKIP() << measurement.rawHeap.skipReason;
  }
  EXPECT_TRUE(hasNoRawHeapAllocations(measurement));
}

TEST(
    StepAllocation, NativeSoftBodiesSkelSteadyStateHasNoGlobalOrAllocatorGrowth)
{
  expectNativeSoftSkelGlobalAndBaseAllocatorGate(
      "native_dart_soft_bodies_skel_steady_state_gate",
      "dart://sample/skel/softBodies.skel");
}

TEST(StepAllocation, NativeSoftBodiesSkelSteadyStateHasNoRawMallocWhenAvailable)
{
  expectNativeSoftSkelRawHeapGate(
      "native_dart_soft_bodies_skel_steady_state_raw_gate",
      "dart://sample/skel/softBodies.skel");
}

TEST(
    StepAllocation,
    NativeSoftOpenChainSkelSteadyStateHasNoGlobalOrAllocatorGrowth)
{
  expectNativeSoftSkelGlobalAndBaseAllocatorGate(
      "native_dart_soft_open_chain_skel_steady_state_gate",
      "dart://sample/skel/soft_open_chain.skel",
      false,
      false,
      kSoftSkelContactWarmupSteps);
}

TEST(
    StepAllocation,
    NativeSoftOpenChainSkelSteadyStateHasNoRawMallocWhenAvailable)
{
  expectNativeSoftSkelRawHeapGate(
      "native_dart_soft_open_chain_skel_steady_state_raw_gate",
      "dart://sample/skel/soft_open_chain.skel",
      false,
      false,
      kSoftSkelContactWarmupSteps);
}

TEST(StepAllocation, NativeSoftCubesSkelSteadyStateHasNoGlobalOrAllocatorGrowth)
{
  expectNativeSoftSkelGlobalAndBaseAllocatorGate(
      "native_dart_soft_cubes_skel_steady_state_gate",
      "dart://sample/skel/soft_cubes.skel",
      true,
      false,
      kSoftSkelContactWarmupSteps);
}

TEST(StepAllocation, NativeSoftCubesSkelSteadyStateHasNoRawMallocWhenAvailable)
{
  expectNativeSoftSkelRawHeapGate(
      "native_dart_soft_cubes_skel_steady_state_raw_gate",
      "dart://sample/skel/soft_cubes.skel",
      true,
      false,
      kSoftSkelContactWarmupSteps);
}

TEST(
    StepAllocation,
    NativeSoftAdaptiveActivationSteadyStateHasNoGlobalOrAllocatorGrowth)
{
  dart::test::CountingMemoryAllocator allocator;
  const auto measurement = measureNativeSoftAdaptiveActivationSteadyState(
      "native_dart_soft_adaptive_activation_steady_state_gate", allocator);
  reportMeasurement(
      "native_dart_soft_adaptive_activation_steady_state_gate", measurement);
  expectNoGlobalHeapAllocationsWhenReliable(
      "native_dart_soft_adaptive_activation_steady_state_gate", measurement);
  EXPECT_TRUE(hasNoCountingAllocatorGrowth(measurement));
}

TEST(
    StepAllocation,
    NativeSoftAdaptiveActivationSteadyStateHasNoRawMallocWhenAvailable)
{
  dart::test::CountingMemoryAllocator allocator;
  const auto measurement = measureNativeSoftAdaptiveActivationSteadyState(
      "native_dart_soft_adaptive_activation_steady_state_raw_gate", allocator);
  reportMeasurement(
      "native_dart_soft_adaptive_activation_steady_state_raw_gate",
      measurement);
  if (measurement.rawHeap.skipped) {
    recordProperty(
        "native_dart_soft_adaptive_activation_steady_state_raw_gate_"
        "raw_malloc_skipped",
        "true");
    recordProperty(
        "native_dart_soft_adaptive_activation_steady_state_raw_gate_"
        "raw_malloc_skip_reason",
        measurement.rawHeap.skipReason);
    GTEST_SKIP() << measurement.rawHeap.skipReason;
  }
  EXPECT_TRUE(hasNoRawHeapAllocations(measurement));
}

TEST(
    StepAllocation,
    ExternalBackendsExplicitFirstPostBakeDoNotGrowWorldBaseAllocator)
{
  bool ranBackend = false;

#if HAVE_BULLET
  ranBackend = true;
  expectExternalBackendBaseAllocatorGate(
      "bullet_explicit_first_post_bake_base_gate",
      dart::collision::BulletCollisionDetector::create(),
      PreparationMode::Explicit);
#endif

#if HAVE_ODE
  ranBackend = true;
  expectExternalBackendBaseAllocatorGate(
      "ode_explicit_first_post_bake_base_gate",
      dart::collision::OdeCollisionDetector::create(),
      PreparationMode::Explicit);
#endif

  if (!ranBackend) {
    GTEST_SKIP() << "Bullet and ODE collision backends are unavailable";
  }
}

TEST(
    StepAllocation,
    ExternalBackendsImplicitSecondStepDoNotGrowWorldBaseAllocator)
{
  bool ranBackend = false;

#if HAVE_BULLET
  ranBackend = true;
  expectExternalBackendBaseAllocatorGate(
      "bullet_implicit_second_step_base_gate",
      dart::collision::BulletCollisionDetector::create(),
      PreparationMode::Implicit);
#endif

#if HAVE_ODE
  ranBackend = true;
  expectExternalBackendBaseAllocatorGate(
      "ode_implicit_second_step_base_gate",
      dart::collision::OdeCollisionDetector::create(),
      PreparationMode::Implicit);
#endif

  if (!ranBackend) {
    GTEST_SKIP() << "Bullet and ODE collision backends are unavailable";
  }
}

TEST(WorldSimulationModeMemoryManager, ExplicitEnterMatchesImplicitSteps)
{
  auto explicitWorld = createFallingBoxWorld("explicit_enter_world");
  auto implicitWorld = createFallingBoxWorld("implicit_enter_world");

  explicitWorld->enterSimulationMode();
  ASSERT_TRUE(explicitWorld->isInSimulationMode());
  ASSERT_FALSE(implicitWorld->isInSimulationMode());

  for (int i = 0; i < 20; ++i) {
    explicitWorld->step();
    implicitWorld->step();
  }

  EXPECT_TRUE(implicitWorld->isInSimulationMode());
  expectWorldStateExactlyEqual(*explicitWorld, *implicitWorld);
}

TEST(WorldSimulationModeMemoryManager, ImplicitFirstStepMatchesExplicitEnter)
{
  auto explicitWorld = createFallingBoxWorld("explicit_first_step_world");
  auto implicitWorld = createFallingBoxWorld("implicit_first_step_world");

  explicitWorld->enterSimulationMode();
  explicitWorld->step();
  implicitWorld->step();

  EXPECT_TRUE(explicitWorld->isInSimulationMode());
  EXPECT_TRUE(implicitWorld->isInSimulationMode());
  expectWorldStateExactlyEqual(*explicitWorld, *implicitWorld);
}

TEST(
    WorldSimulationModeMemoryManager, ExplicitEnterPreservesLastCollisionResult)
{
  auto world = dart::simulation::World::create("preserve_last_collision_world");
  world->setNumSimulationThreads(1u);
  world->getConstraintSolver()->setCollisionDetector(
      dart::collision::DARTCollisionDetector::create());

  world->addSkeleton(createGround());
  world->addSkeleton(createBox(
      0u,
      Eigen::Vector3d(0.0, 0.0, 0.14),
      Eigen::Vector3d(0.2, 0.2, 0.2),
      Eigen::Vector3d(0.2, 0.4, 0.8)));

  ASSERT_EQ(world->getLastCollisionResult().getNumContacts(), 0u);
  world->enterSimulationMode();
  EXPECT_TRUE(world->isInSimulationMode());
  EXPECT_EQ(world->getLastCollisionResult().getNumContacts(), 0u);

  world->step();
  EXPECT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
}

TEST(WorldSimulationModeMemoryManager, ExplicitEnterPreservesCollidingFlags)
{
  auto world
      = dart::simulation::World::create("preserve_colliding_flags_world");
  world->setNumSimulationThreads(1u);
  world->getConstraintSolver()->setCollisionDetector(
      dart::collision::DARTCollisionDetector::create());

  auto ground = createGround();
  auto box = createBox(
      0u,
      Eigen::Vector3d(0.0, 0.0, 0.14),
      Eigen::Vector3d(0.2, 0.2, 0.2),
      Eigen::Vector3d(0.2, 0.4, 0.8));
  auto* groundBody = ground->getBodyNode(0u);
  auto* boxBody = box->getBodyNode(0u);
  world->addSkeleton(ground);
  world->addSkeleton(box);

  DART_SUPPRESS_DEPRECATED_BEGIN
  ASSERT_FALSE(groundBody->isColliding());
  ASSERT_FALSE(boxBody->isColliding());
  DART_SUPPRESS_DEPRECATED_END

  world->enterSimulationMode();
  EXPECT_TRUE(world->isInSimulationMode());

  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_FALSE(groundBody->isColliding());
  EXPECT_FALSE(boxBody->isColliding());
  DART_SUPPRESS_DEPRECATED_END

  world->step();
  EXPECT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_TRUE(groundBody->isColliding());
  EXPECT_TRUE(boxBody->isColliding());
  DART_SUPPRESS_DEPRECATED_END
}

TEST(WorldSimulationModeMemoryManager, ShapeChangeInvalidatesAndRebakes)
{
  auto world = createFallingBoxWorld("rebake_world");
  world->enterSimulationMode();
  ASSERT_TRUE(world->isInSimulationMode());

  world->addSkeleton(createBox(
      1u,
      Eigen::Vector3d(0.4, 0.0, 1.2),
      Eigen::Vector3d(0.2, 0.2, 0.2),
      Eigen::Vector3d(0.8, 0.4, 0.2)));
  EXPECT_FALSE(world->isInSimulationMode());

  world->step();
  EXPECT_TRUE(world->isInSimulationMode());

  std::size_t expectedDofs = 0u;
  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i)
    expectedDofs += world->getSkeleton(i)->getNumDofs();
  EXPECT_EQ(
      world->getIndex(static_cast<int>(world->getNumSkeletons())),
      static_cast<int>(expectedDofs));

  const auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "rebake_frame");
  world->addSimpleFrame(frame);
  EXPECT_FALSE(world->isInSimulationMode());
  world->enterSimulationMode();
  EXPECT_TRUE(world->isInSimulationMode());
  world->removeSimpleFrame(frame);
  EXPECT_FALSE(world->isInSimulationMode());
}

TEST(WorldSimulationModeMemoryManager, ThreadCountChangeInvalidatesBake)
{
  auto world = createFallingBoxWorld("thread_count_rebake_world");
  world->setNumSimulationThreads(1u);
  world->enterSimulationMode();
  ASSERT_TRUE(world->isInSimulationMode());

  world->setNumSimulationThreads(2u);
  EXPECT_FALSE(world->isInSimulationMode());

  world->step();
  EXPECT_TRUE(world->isInSimulationMode());
}

TEST(WorldSimulationModeMemoryManager, CollisionDetectorChangeInvalidatesBake)
{
  auto world = createFallingBoxWorld("detector_change_rebake_world");
  world->setCollisionDetector(dart::collision::DARTCollisionDetector::create());
  world->enterSimulationMode();
  ASSERT_TRUE(world->isInSimulationMode());

  world->setCollisionDetector(world->getCollisionDetector());
  EXPECT_TRUE(world->isInSimulationMode());

  world->setCollisionDetector(dart::collision::DARTCollisionDetector::create());
  EXPECT_FALSE(world->isInSimulationMode());

  world->step();
  EXPECT_TRUE(world->isInSimulationMode());
}

TEST(
    WorldSimulationModeMemoryManager,
    ConstraintSolverDetectorChangeInvalidatesBake)
{
  auto world = createFallingBoxWorld("solver_detector_change_rebake_world");
  world->setCollisionDetector(dart::collision::DARTCollisionDetector::create());
  world->enterSimulationMode();
  ASSERT_TRUE(world->isInSimulationMode());

  auto* solver = world->getConstraintSolver();
  solver->setCollisionDetector(world->getCollisionDetector());
  EXPECT_TRUE(world->isInSimulationMode());

  solver->setCollisionDetector(
      dart::collision::DARTCollisionDetector::create());
  EXPECT_FALSE(world->isInSimulationMode());

  world->step();
  EXPECT_TRUE(world->isInSimulationMode());
}

TEST(WorldSimulationModeMemoryManager, CollisionOptionChangeInvalidatesBake)
{
  auto world = createFallingBoxWorld("collision_option_rebake_world");
  world->addSkeleton(createGround());
  world->setCollisionDetector(dart::collision::DARTCollisionDetector::create());

  auto* boxBody = world->getSkeleton(0u)->getBodyNode(0u);
  auto* groundBody = world->getSkeleton(1u)->getBodyNode(0u);
  auto* solver = world->getConstraintSolver();
  auto& option = solver->getCollisionOption();
  option.maxNumContacts = 8u;
  option.maxNumContactsPerPair = 4u;
  option.collisionFilter
      = std::make_shared<dart::collision::BodyNodeCollisionFilter>();

  world->enterSimulationMode();
  ASSERT_TRUE(world->isInSimulationMode());

  option.maxNumContacts = 16u;
  EXPECT_FALSE(world->isInSimulationMode());
  option.maxNumContacts = 8u;
  world->enterSimulationMode();
  ASSERT_TRUE(world->isInSimulationMode());

  option.collisionFilter
      = std::make_shared<dart::collision::BodyNodeCollisionFilter>();
  EXPECT_FALSE(world->isInSimulationMode());
  world->enterSimulationMode();
  ASSERT_TRUE(world->isInSimulationMode());

  auto* filter = dynamic_cast<dart::collision::BodyNodeCollisionFilter*>(
      option.collisionFilter.get());
  ASSERT_NE(filter, nullptr);
  filter->addBodyNodePairToBlackList(boxBody, groundBody);
  EXPECT_FALSE(world->isInSimulationMode());

  world->step();
  EXPECT_TRUE(world->isInSimulationMode());

  option.collisionFilter = std::make_shared<PassThroughCollisionFilter>();
  world->enterSimulationMode();
  EXPECT_TRUE(world->isInSimulationMode());

  option.collisionFilter = std::make_shared<PassThroughCollisionFilter>();
  EXPECT_FALSE(world->isInSimulationMode());
}

TEST(WorldSimulationModeMemoryManager, CollisionGroupContentInvalidatesBake)
{
  auto world = createFallingBoxWorld("collision_group_content_rebake_world");
  world->setCollisionDetector(dart::collision::DARTCollisionDetector::create());
  world->enterSimulationMode();
  ASSERT_TRUE(world->isInSimulationMode());

  const auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "collision_group_rebake_frame");
  frame->setShape(std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d::Constant(0.1)));

  world->getConstraintSolver()->getCollisionGroup()->addShapeFrame(frame.get());
  EXPECT_FALSE(world->isInSimulationMode());

  world->step();
  EXPECT_TRUE(world->isInSimulationMode());
}

TEST(WorldSimulationModeMemoryManager, FrameArenaResetsEachStep)
{
  auto world = createFallingBoxWorld("frame_arena_reset_world");
  world->enterSimulationMode();
  ASSERT_TRUE(world->isInSimulationMode());

  auto& frameAllocator = world->getMemoryManager().getFrameAllocator();
  ASSERT_NE(frameAllocator.allocate(128u), nullptr);
  ASSERT_GT(frameAllocator.used(), 0u);
  world->step();
  EXPECT_EQ(frameAllocator.used(), 0u);

  ASSERT_NE(frameAllocator.allocate(256u), nullptr);
  ASSERT_GT(frameAllocator.used(), 0u);
  world->step();
  EXPECT_EQ(frameAllocator.used(), 0u);
}

TEST(WorldSimulationModeMemoryManager, BakedWorldBaseAllocatorDoesNotGrow)
{
  dart::test::CountingMemoryAllocator allocator;
  dart::simulation::WorldConfig config("counted_allocator_world");
  config.baseAllocator = &allocator;
  config.freeListInitialAllocation = 1024u;
  config.frameScratchInitialCapacity = 4096u;

  auto world = dart::simulation::World::create(config);
  EXPECT_EQ(&world->getMemoryManager().getBaseAllocator(), &allocator);
  world->setNumSimulationThreads(1u);
  world->addSkeleton(createBox(
      0u,
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(0.2, 0.2, 0.2),
      Eigen::Vector3d(0.2, 0.6, 0.3)));
  world->enterSimulationMode();
  ASSERT_TRUE(world->isInSimulationMode());

  dart::test::ScopedCountingMemoryAllocatorCounter counter(allocator);
  for (int i = 0; i < 10; ++i)
    world->step();
  counter.stop();

  const auto snapshot = counter.snapshot();
  EXPECT_EQ(snapshot.allocationCount, 0u);
  EXPECT_EQ(snapshot.allocationBytes, 0u);
}

TEST(StepAllocation, ReportsWorldStepAllocationBaseline)
{
  const auto nativeMeasurement
      = measureScene(dart::collision::DARTCollisionDetector::create());
  reportMeasurement("native_dart_boxes", nativeMeasurement);

#if HAVE_BULLET
  const auto bulletMeasurement
      = measureScene(dart::collision::BulletCollisionDetector::create());
  reportMeasurement(
      "bullet_boxes",
      bulletMeasurement,
      "includes collision-backend-internal allocations");
#else
  recordProperty("bullet_boxes_skipped", "true");
  recordProperty("bullet_boxes_skip_reason", "Bullet is unavailable");
  std::cout << "[StepAllocation] bullet_boxes skipped reason=\"Bullet is "
               "unavailable\"\n";
#endif
}

// Z1: prepared, awake contact scenes retain their high-water storage on the
// submitting thread and on the solver workers.
TEST(StepAllocation, AwakeGridSteadyState)
{
  if (!dart::test::ScopedRawHeapAllocationCounter::isAvailable())
    GTEST_SKIP() << dart::test::ScopedRawHeapAllocationCounter::skipReason();

  for (const std::size_t threads : {1u, 4u}) {
    SCOPED_TRACE(threads);
    auto world = createAllocationGateWorld(
        dart::collision::DARTCollisionDetector::create(), false, threads);
    ASSERT_EQ(world->getConstraintSolver()->getNumSimulationThreads(), threads);
    // 144 independent ground-contact islands exceed the 128-group threshold
    // for parallel LCP solving, as well as the World dispatch thresholds.
    addGridBoxes(world, threads == 4u ? 1 : 3, threads == 4u ? 12 : 3);
    for (int i = 0; i < 300; ++i)
      world->step();
    if (threads == 4u) {
      const auto* solver = static_cast<const AllocationGateConstraintSolver*>(
          world->getConstraintSolver());
      ASSERT_GE(solver->getNumConstrainedGroups(), 128u);
      ASSERT_TRUE(solver->canSolveInParallel());
    }
    dart::test::CountingMemoryAllocator allocator;
    const auto measurement = measureWorldStepsNow(world, allocator, 100, 300);
    EXPECT_GT(measurement.lastStepContacts, 0u);
    EXPECT_EQ(countResting(world), 0u);
    expectAllocationGateBudget(
        "dart_awake_grid_threads_" + std::to_string(threads), measurement);
  }
}

// Z2: explicit preparation covers both the freeze event and the first
// all-resting snapshot; neither may allocate during subsequent steps.
TEST(StepAllocation, SleepTransition)
{
  if (!dart::test::ScopedRawHeapAllocationCounter::isAvailable())
    GTEST_SKIP() << dart::test::ScopedRawHeapAllocationCounter::skipReason();

  auto world = createAllocationGateWorld(
      dart::collision::DARTCollisionDetector::create(), true);
  addGridBoxes(world, 3);
  ASSERT_EQ(countResting(world), 0u);
  world->enterSimulationMode();
  dart::test::CountingMemoryAllocator allocator;
  const auto measurement = measureWorldStepsNow(world, allocator, 600);
  EXPECT_EQ(countResting(world), 27u);
  expectAllocationGateBudget("dart_sleep_transition", measurement);
}

TEST(StepAllocation, WakeTransition)
{
  if (!dart::test::ScopedRawHeapAllocationCounter::isAvailable())
    GTEST_SKIP() << dart::test::ScopedRawHeapAllocationCounter::skipReason();

  auto world = createAllocationGateWorld(
      dart::collision::DARTCollisionDetector::create(), true);
  addGridBoxes(world, 3);
  for (int i = 0; i < 600; ++i)
    world->step();
  ASSERT_EQ(countResting(world), 27u);
  auto* top = world->getSkeleton(3)->getBodyNode(0);
  std::size_t restingAfterWake = 27u;
  dart::test::CountingMemoryAllocator allocator;
  const auto measurement
      = measureWorldStepsNow(world, allocator, 800, 600, [&](int step) {
          if (step == 0)
            top->addExtForce(Eigen::Vector3d(200.0, 0.0, 0.0));
          if (step == 1)
            restingAfterWake = countResting(world);
        });
  EXPECT_LT(restingAfterWake, 27u) << "the external push must wake an island";
  EXPECT_EQ(countResting(world), 27u) << "the awakened stack must re-settle";
  expectAllocationGateBudget("dart_wake_transition", measurement);
}

TEST(StepAllocation, IslandMergeSplitSecondCycle)
{
  if (!dart::test::ScopedRawHeapAllocationCounter::isAvailable())
    GTEST_SKIP() << dart::test::ScopedRawHeapAllocationCounter::skipReason();

#if !HAVE_BULLET
  GTEST_SKIP() << "Bullet is required for the Z1a island-retention gate";
#else
  // Native manifold-cache node reuse is F9, explicitly deferred to Z1b.
  // Bullet keeps the same merge/split fixture focused on F5 island storage.
  const auto detector = dart::collision::BulletCollisionDetector::create();
  auto world = createAllocationGateWorld(detector, false);
  auto solver = std::make_unique<AllocationGateConstraintSolver>();
  auto* inspectedSolver = solver.get();
  world->setConstraintSolver(std::move(solver));
  // setFromOtherConstraintSolver copies constraints and skeletons, but the
  // replacement solver starts with its default detector.
  inspectedSolver->setCollisionDetector(detector);
  ASSERT_EQ(
      inspectedSolver->getCollisionDetector()->getType(),
      dart::collision::BulletCollisionDetector::getStaticType());
  std::vector<dart::dynamics::SkeletonPtr> movers;
  for (int i = 0; i < 6; ++i) {
    for (double x : {-0.6, 0.6}) {
      auto box = createAllocationGateBox(
          "m" + std::to_string(i) + (x < 0 ? "a" : "b"),
          Eigen::Vector3d(x, 1.0 * i, 0.1),
          0.2,
          1.0);
      box->getBodyNode(0)
          ->getShapeNode(0)
          ->getDynamicsAspect()
          ->setFrictionCoeff(0.0);
      world->addSkeleton(box);
      movers.push_back(box);
    }
  }
  const auto kick = [&] {
    for (std::size_t i = 0; i < movers.size(); ++i) {
      auto* joint
          = static_cast<dart::dynamics::FreeJoint*>(movers[i]->getJoint(0));
      Eigen::Vector6d q = Eigen::Vector6d::Zero();
      q[3] = (i % 2 == 0) ? -0.6 : 0.6;
      q[4] = static_cast<double>(i / 2);
      q[5] = 0.1;
      joint->setPositionsStatic(q);
      Eigen::Vector6d v = Eigen::Vector6d::Zero();
      v[3] = (i % 2 == 0) ? 2.0 : -2.0;
      joint->setVelocitiesStatic(v);
    }
  };
  kick();
  for (int i = 0; i < 700; ++i)
    world->step();
  kick();
  bool sawMerge = false;
  bool sawSplitAfterMerge = false;
  std::size_t minGroups = movers.size();
  std::size_t maxGroups = 0u;
  dart::test::CountingMemoryAllocator allocator;
  const auto measurement
      = measureWorldStepsNow(world, allocator, 700, 700, [&](int) {
          const auto groups = inspectedSolver->getNumConstrainedGroups();
          minGroups = std::min(minGroups, groups);
          maxGroups = std::max(maxGroups, groups);
          bool merged = false;
          for (const auto& contact :
               world->getLastCollisionResult().getContacts()) {
            const auto body1 = contact.getBodyNodePtr1();
            const auto body2 = contact.getBodyNodePtr2();
            if (body1 && body2 && body1->getSkeleton()->isMobile()
                && body2->getSkeleton()->isMobile()) {
              merged = true;
              break;
            }
          }
          sawSplitAfterMerge |= sawMerge && !merged;
          sawMerge |= merged;
        });
  recordProperty("merge_split_min_groups", minGroups);
  recordProperty("merge_split_max_groups", maxGroups);
  EXPECT_LT(minGroups, maxGroups);
  EXPECT_TRUE(sawMerge) << "the second cycle must merge moving-body islands";
  EXPECT_TRUE(sawSplitAfterMerge) << "the merged islands must split again";
  expectAllocationGateBudget("bullet_merge_split_second_cycle", measurement);
#endif
}

TEST(StepAllocation, ArticulatedJointLimitsSteadyState)
{
  if (!dart::test::ScopedRawHeapAllocationCounter::isAvailable())
    GTEST_SKIP() << dart::test::ScopedRawHeapAllocationCounter::skipReason();

  auto world
      = dart::utils::SkelParser::readWorld("dart://sample/skel/fullbody1.skel");
  ASSERT_NE(world, nullptr);
  world->setNumSimulationThreads(1u);
  world->getConstraintSolver()->setCollisionDetector(
      dart::collision::DARTCollisionDetector::create());
  auto deactivation = world->getDeactivationOptions();
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);
  std::size_t limitedJoints = 0u;
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    auto skel = world->getSkeleton(i);
    for (std::size_t j = 0; j < skel->getNumJoints(); ++j) {
      auto* joint = skel->getJoint(j);
      if (joint->getNumDofs() == 0u || joint->getNumDofs() == 6u)
        continue;
      for (std::size_t d = 0; d < joint->getNumDofs(); ++d) {
        joint->setPositionLowerLimit(d, -0.5);
        joint->setPositionUpperLimit(d, 0.5);
      }
      joint->setLimitEnforcement(true);
      ++limitedJoints;
    }
  }
  ASSERT_EQ(limitedJoints, 19u);
  for (int i = 0; i < 2000; ++i)
    world->step();
  ASSERT_EQ(countResting(world), 0u);
  dart::test::CountingMemoryAllocator allocator;
  expectAllocationGateBudget(
      "dart_articulated_limits_steady",
      measureWorldStepsNow(world, allocator, 500, 2000));
}

// Z3: structural edits may allocate in the API call and the one re-bake step.
TEST(StepAllocation, StructuralChangeRebakesThenZero)
{
  if (!dart::test::ScopedRawHeapAllocationCounter::isAvailable())
    GTEST_SKIP() << dart::test::ScopedRawHeapAllocationCounter::skipReason();

  auto world = createAllocationGateWorld(
      dart::collision::DARTCollisionDetector::create(), false);
  addGridBoxes(world, 1);
  for (int i = 0; i < 200; ++i)
    world->step();
  auto extra
      = createAllocationGateBox("extra", Eigen::Vector3d(5.0, 5.0, 0.25), 0.5);
  world->addSkeleton(extra);
  world->step();
  dart::test::CountingMemoryAllocator allocator;
  expectAllocationGateBudget(
      "dart_after_add_skeleton",
      measureWorldStepsNow(world, allocator, 200, 1));
  world->removeSkeleton(extra);
  world->step();
  expectAllocationGateBudget(
      "dart_after_remove_skeleton",
      measureWorldStepsNow(world, allocator, 200, 1));

  // A direct Skeleton topology edit can leave a previously valid resting
  // snapshot with the same DOF count but a different BodyNode count.
  auto sleepingWorld = createAllocationGateWorld(
      dart::collision::DARTCollisionDetector::create(), true);
  addGridBoxes(sleepingWorld, 3);
  for (int i = 0; i < 600; ++i)
    sleepingWorld->step();
  ASSERT_EQ(countResting(sleepingWorld), 27u);
  const auto changedSkeleton = sleepingWorld->getSkeleton(3);
  const auto dofsBefore = changedSkeleton->getNumDofs();
  const auto bodiesBefore = changedSkeleton->getNumBodyNodes();
  changedSkeleton->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(
      changedSkeleton->getBodyNode(0));
  ASSERT_EQ(changedSkeleton->getNumDofs(), dofsBefore);
  ASSERT_EQ(changedSkeleton->getNumBodyNodes(), bodiesBefore + 1u);
  EXPECT_FALSE(sleepingWorld->isInSimulationMode());
  sleepingWorld->enterSimulationMode();
  EXPECT_TRUE(sleepingWorld->isInSimulationMode());
  const auto topologyMeasurement
      = measureWorldStepsNow(sleepingWorld, allocator, 600, 1);
  EXPECT_EQ(countResting(sleepingWorld), 27u);
  expectAllocationGateBudget(
      "dart_after_body_topology_change", topologyMeasurement);
}

// Bullet and dart have strict gates. FCL and ODE retain backend-internal
// allocations, bounded by the adjacent checked-in JSON ratchet.
TEST(StepAllocation, DetectorSwitchPerBackendSteadyState)
{
  if (!dart::test::ScopedRawHeapAllocationCounter::isAvailable())
    GTEST_SKIP() << dart::test::ScopedRawHeapAllocationCounter::skipReason();

  auto world = createAllocationGateWorld(
      dart::collision::DARTCollisionDetector::create(), false);
  addGridBoxes(world, 1);
  for (int i = 0; i < 200; ++i)
    world->step();
  const std::pair<const char*, dart::collision::CollisionDetectorPtr>
      detectors[] = {
#if HAVE_BULLET
        {"bullet", dart::collision::BulletCollisionDetector::create()},
#endif
        {"fcl", dart::collision::FCLCollisionDetector::create()},
#if HAVE_ODE
        {"ode", dart::collision::OdeCollisionDetector::create()},
#endif
        {"dart", dart::collision::DARTCollisionDetector::create()},
      };
  for (const auto& [name, detector] : detectors) {
    SCOPED_TRACE(name);
    world->getConstraintSolver()->setCollisionDetector(detector);
    for (int i = 0; i < 100; ++i)
      world->step();
    dart::test::CountingMemoryAllocator allocator;
    const auto measurement = measureWorldStepsNow(world, allocator, 200, 100);
    EXPECT_GT(measurement.lastStepContacts, 0u);
    const std::string row = std::string(name) + "_after_switch_steady";
    expectAllocationGateBudget(
        row,
        measurement,
        std::string(name) == "bullet" || std::string(name) == "dart");
  }
}

TEST(StepAllocation, GzLikeFilterAndHandlerSteadyState)
{
  if (!dart::test::ScopedRawHeapAllocationCounter::isAvailable())
    GTEST_SKIP() << dart::test::ScopedRawHeapAllocationCounter::skipReason();

  for (const std::size_t threads : {1u, 4u}) {
    for (const bool bullet : {false, true}) {
#if !HAVE_BULLET
      if (bullet)
        continue;
#endif
      SCOPED_TRACE(threads);
      SCOPED_TRACE(bullet);
      dart::collision::CollisionDetectorPtr detector
          = dart::collision::DARTCollisionDetector::create();
#if HAVE_BULLET
      if (bullet)
        detector = dart::collision::BulletCollisionDetector::create();
#endif
      // Current main allows custom filters to sleep. Keep this row awake so
      // every measured step invokes the filter and the handler.
      auto world = createAllocationGateWorld(detector, false, threads);
      ASSERT_EQ(
          world->getConstraintSolver()->getNumSimulationThreads(), threads);
      // The four-thread variant also exercises parallel group solving.
      addGridBoxes(world, threads == 4u ? 1 : 2, threads == 4u ? 12 : 3);
      auto filter = std::make_shared<AllocationGateBitmaskFilter>();
      for (std::size_t i = 0; i < world->getNumSkeletons(); ++i)
        filter->mMask[world->getSkeleton(i)->getBodyNode(0)->getShapeNode(0)]
            = 0xffu;
      auto* solver = world->getConstraintSolver();
      solver->getCollisionOption().collisionFilter = filter;
      auto handler = std::make_shared<AllocationGateCallbackHandler>();
      handler->mCallback = [](dart::constraint::ContactSurfaceParams& params) {
        params.mRestitutionCoeff = 0.0;
      };
      solver->addContactSurfaceHandler(handler);
      for (int i = 0; i < 300; ++i)
        world->step();
      if (threads == 4u) {
        const auto* inspectedSolver
            = static_cast<const AllocationGateConstraintSolver*>(solver);
        ASSERT_GE(inspectedSolver->getNumConstrainedGroups(), 128u);
        ASSERT_TRUE(inspectedSolver->canSolveInParallel());
      }
      const auto callbacksBeforeMeasurement = handler->mCalls.load();
      dart::test::CountingMemoryAllocator allocator;
      const auto measurement = measureWorldStepsNow(world, allocator, 200, 300);
      EXPECT_GT(handler->mCalls.load(), callbacksBeforeMeasurement)
          << "the contact handler must execute in the measured window";
      EXPECT_GT(measurement.lastStepContacts, 0u);
      EXPECT_EQ(countResting(world), 0u);
      expectAllocationGateBudget(
          std::string(bullet ? "bullet" : "dart") + "_gzlike_threads_"
              + std::to_string(threads),
          measurement);
    }
  }
}
