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
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/BoxedLcpSolver.hpp"
#include "dart/dynamics/dynamics.hpp"
#include "dart/lcpsolver/dantzig/DantzigLcp.hpp"
#include "dart/simulation/World.hpp"

#if HAVE_BULLET
  #include "dart/collision/bullet/bullet.hpp"
#endif
#if HAVE_ODE
  #include "dart/collision/ode/ode.hpp"
#endif

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <cstddef>
#include <cstdlib>

namespace {

constexpr std::size_t kBoxesPerSide = 3u;
constexpr int kWarmupSteps = 50;
constexpr int kMeasuredSteps = 100;
constexpr double kBoxEdge = 0.9;
constexpr double kGroundThickness = 0.1;

void* volatile g_allocationSink = nullptr;

bool canEnforceGlobalHeapAllocationGate()
{
#if defined(_WIN32)
  return false;
#else
  return true;
#endif
}

const char* globalHeapAllocationGateSkipReason()
{
#if defined(_WIN32)
  return "global operator-new counter is advisory on Windows";
#else
  return "";
#endif
}

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
  int measuredSteps = 0;
  std::size_t lastStepContacts = 0u;
};

StepAllocationMeasurement measureWorldStepsNow(
    const dart::simulation::WorldPtr& world,
    dart::test::CountingMemoryAllocator& allocator,
    int measuredSteps)
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

  for (int i = 0; i < measuredSteps; ++i) {
    world->step();
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
      measuredSteps,
      world->getLastCollisionResult().getNumContacts()};
}

StepAllocationMeasurement measureWorldStepAllocations(
    const dart::simulation::WorldPtr& world,
    dart::test::CountingMemoryAllocator& allocator)
{
  for (int i = 0; i < kWarmupSteps; ++i) {
    world->step();
  }

  return measureWorldStepsNow(world, allocator, kMeasuredSteps);
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

void reportMeasurement(
    const std::string& label,
    const StepAllocationMeasurement& measurement,
    const std::string& note = "")
{
  const std::string prefix = label + "_";

  recordProperty(prefix + "boxes_per_side", kBoxesPerSide);
  recordProperty(prefix + "warmup_steps", kWarmupSteps);
  recordProperty(prefix + "measured_steps", measurement.measuredSteps);
  recordProperty(prefix + "last_step_contacts", measurement.lastStepContacts);

  // Scene validity, not an allocation assertion: the baseline is only
  // meaningful if the measured window actually exercises contact solving.
  EXPECT_GT(measurement.lastStepContacts, 0u)
      << label << " scene produced no contacts in the measured window";
  recordProperty(
      prefix + "operator_new_count", measurement.globalHeap.allocationCount);
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
            << " warmup_steps=" << kWarmupSteps
            << " measured_steps=" << measurement.measuredSteps
            << " last_step_contacts=" << measurement.lastStepContacts;
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

void expectNativeGlobalAndBaseAllocatorGate(
    PreparationMode mode, const std::string& label)
{
  dart::test::CountingMemoryAllocator allocator;
  const auto measurement = measurePreparedGateScene(
      label, dart::collision::DARTCollisionDetector::create(), mode, allocator);
  reportMeasurement(label, measurement);
  EXPECT_GT(measurement.lastStepContacts, 0u);
  if (canEnforceGlobalHeapAllocationGate()) {
    EXPECT_TRUE(hasNoGlobalHeapAllocations(measurement));
  } else {
    recordProperty(label + "_operator_new_gate_skipped", "true");
    recordProperty(
        label + "_operator_new_gate_skip_reason",
        globalHeapAllocationGateSkipReason());
  }
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

TEST(
    StepAllocation, NativeExplicitFirstPostBakeHasNoGlobalOrBaseAllocatorGrowth)
{
  expectNativeGlobalAndBaseAllocatorGate(
      PreparationMode::Explicit, "native_dart_explicit_first_post_bake_gate");
}

TEST(StepAllocation, NativeImplicitSecondStepHasNoGlobalOrBaseAllocatorGrowth)
{
  expectNativeGlobalAndBaseAllocatorGate(
      PreparationMode::Implicit, "native_dart_implicit_second_step_gate");
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
