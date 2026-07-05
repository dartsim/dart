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
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/BoxedLcpSolver.hpp"
#include "dart/dynamics/dynamics.hpp"
#include "dart/lcpsolver/dantzig/DantzigLcp.hpp"
#include "dart/simulation/World.hpp"

#if HAVE_BULLET
  #include "dart/collision/bullet/bullet.hpp"
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
  std::size_t lastStepContacts = 0u;
};

StepAllocationMeasurement measureWorldStepAllocations(
    const dart::simulation::WorldPtr& world,
    dart::test::CountingMemoryAllocator& allocator)
{
  for (int i = 0; i < kWarmupSteps; ++i) {
    world->step();
  }

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

  for (int i = 0; i < kMeasuredSteps; ++i) {
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
      world->getLastCollisionResult().getNumContacts()};
}

std::string perStep(std::size_t count)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(3)
     << static_cast<double>(count) / static_cast<double>(kMeasuredSteps);
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
  recordProperty(prefix + "measured_steps", kMeasuredSteps);
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
      perStep(measurement.globalHeap.allocationCount));
  recordProperty(
      prefix + "operator_new_bytes_per_step",
      perStep(measurement.globalHeap.allocationBytes));

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
        perStep(measurement.rawHeap.allocationCount));
    recordProperty(
        prefix + "raw_malloc_bytes_per_step",
        perStep(measurement.rawHeap.allocationBytes));
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
      perStep(measurement.countingAllocator.allocationCount));

  std::cout << "[StepAllocation] " << label
            << " boxes_per_side=" << kBoxesPerSide
            << " warmup_steps=" << kWarmupSteps
            << " measured_steps=" << kMeasuredSteps
            << " last_step_contacts=" << measurement.lastStepContacts;
  if (!note.empty()) {
    std::cout << " note=\"" << note << "\"";
  }
  std::cout << '\n';
  std::cout << "  operator_new_count=" << measurement.globalHeap.allocationCount
            << " operator_new_bytes=" << measurement.globalHeap.allocationBytes
            << " operator_new_count_per_step="
            << perStep(measurement.globalHeap.allocationCount)
            << " operator_new_bytes_per_step="
            << perStep(measurement.globalHeap.allocationBytes) << '\n';
  if (measurement.rawHeap.skipped) {
    std::cout << "  raw_malloc_count=skipped raw_malloc_bytes=skipped reason=\""
              << measurement.rawHeap.skipReason << "\"\n";
  } else {
    std::cout << "  raw_malloc_count=" << measurement.rawHeap.allocationCount
              << " raw_malloc_bytes=" << measurement.rawHeap.allocationBytes
              << " raw_malloc_count_per_step="
              << perStep(measurement.rawHeap.allocationCount)
              << " raw_malloc_bytes_per_step="
              << perStep(measurement.rawHeap.allocationBytes) << '\n';
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
            << perStep(measurement.countingAllocator.allocationCount) << '\n';
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
