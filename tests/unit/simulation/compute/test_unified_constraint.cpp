/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

#include <dart/simulation/body/contact.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/comps/dynamics.hpp>
#include <dart/simulation/comps/multibody.hpp>
#include <dart/simulation/compute/multibody_dynamics.hpp>
#include <dart/simulation/compute/parallel_executor.hpp>
#include <dart/simulation/compute/rigid_body_constraint.hpp>
#include <dart/simulation/compute/unified_constraint.hpp>
#include <dart/simulation/detail/entity_conversion.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/multibody/joint.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>

#include <dart/common/memory_manager.hpp>

#include <entt/entt.hpp>
#include <gtest/gtest.h>

#include <atomic>
#include <limits>
#include <new>
#include <span>
#include <vector>

#include <cmath>
#include <cstdlib>

#if defined(_WIN32)
  #include <malloc.h>
#endif

namespace {

namespace sx = dart::simulation;

//==============================================================================
void expectMatrixExactlyEqual(
    const Eigen::MatrixXd& lhs, const Eigen::MatrixXd& rhs)
{
  ASSERT_EQ(lhs.rows(), rhs.rows());
  ASSERT_EQ(lhs.cols(), rhs.cols());
  for (Eigen::Index row = 0; row < lhs.rows(); ++row) {
    for (Eigen::Index col = 0; col < lhs.cols(); ++col) {
      EXPECT_EQ(lhs(row, col), rhs(row, col))
          << "at (" << row << ", " << col << ")";
    }
  }
}

template <typename Vector>
void expectVectorExactlyEqual(const Vector& lhs, const Vector& rhs)
{
  ASSERT_EQ(lhs.size(), rhs.size());
  for (Eigen::Index i = 0; i < lhs.size(); ++i) {
    EXPECT_EQ(lhs[i], rhs[i]) << "at " << i;
  }
}

std::atomic<bool> g_heapAllocationTrackingEnabled{false};
std::atomic<std::size_t> g_heapAllocationCount{0};
std::atomic<std::size_t> g_heapAllocationBytes{0};

void recordHeapAllocation(std::size_t bytes) noexcept
{
  if (g_heapAllocationTrackingEnabled.load(std::memory_order_relaxed)) {
    g_heapAllocationCount.fetch_add(1, std::memory_order_relaxed);
    g_heapAllocationBytes.fetch_add(bytes, std::memory_order_relaxed);
  }
}

[[nodiscard]] void* allocateRaw(std::size_t bytes) noexcept
{
  return std::malloc(bytes == 0 ? 1 : bytes);
}

[[nodiscard]] void* allocateAlignedRaw(
    std::size_t bytes, std::size_t alignment) noexcept
{
  if (alignment <= __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
    return allocateRaw(bytes);
  }

#if defined(_WIN32)
  return _aligned_malloc(bytes == 0 ? 1 : bytes, alignment);
#else
  const auto requested = bytes == 0 ? 1 : bytes;
  if (alignment == 0 || (alignment & (alignment - 1)) != 0) {
    return nullptr;
  }
  if (alignment < alignof(void*)) {
    alignment = alignof(void*);
  }
  void* pointer = nullptr;
  return posix_memalign(&pointer, alignment, requested) == 0 ? pointer
                                                             : nullptr;
#endif
}

void deallocateAlignedRaw(void* pointer, std::size_t alignment) noexcept
{
  if (alignment <= __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
    std::free(pointer);
    return;
  }

#if defined(_WIN32)
  _aligned_free(pointer);
#else
  std::free(pointer);
#endif
}

class ScopedHeapAllocationCounter final
{
public:
  ScopedHeapAllocationCounter()
  {
    g_heapAllocationCount.store(0, std::memory_order_relaxed);
    g_heapAllocationBytes.store(0, std::memory_order_relaxed);
    g_heapAllocationTrackingEnabled.store(true, std::memory_order_relaxed);
  }

  ~ScopedHeapAllocationCounter()
  {
    g_heapAllocationTrackingEnabled.store(false, std::memory_order_relaxed);
  }

  ScopedHeapAllocationCounter(const ScopedHeapAllocationCounter&) = delete;
  ScopedHeapAllocationCounter& operator=(const ScopedHeapAllocationCounter&)
      = delete;

  [[nodiscard]] std::size_t allocations() const noexcept
  {
    return g_heapAllocationCount.load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::size_t bytes() const noexcept
  {
    return g_heapAllocationBytes.load(std::memory_order_relaxed);
  }
};

//==============================================================================
TEST(UnifiedConstraint, ProblemContainersUseProvidedAllocator)
{
  namespace common = dart::common;

  common::MemoryManager memoryManager;
  auto& freeList = memoryManager.getFreeListAllocator();
  const auto allocationsBeforeProblem = freeList.getAllocationCount();

  {
    sx::compute::RigidBodyContactProblem rigidProblem(
        memoryManager.getFreeAllocator());
    sx::compute::UnifiedConstraintProblem problem(
        memoryManager.getFreeAllocator());

    const common::StlAllocator<sx::compute::RigidBodyContactConstraint>
        rigidConstraintAllocator{memoryManager.getFreeAllocator()};
    const common::StlAllocator<sx::compute::UnifiedRowOwner> rowOwnerAllocator{
        memoryManager.getFreeAllocator()};
    const common::StlAllocator<sx::compute::UnifiedMultibodyBlock>
        blockAllocator{memoryManager.getFreeAllocator()};
    const common::StlAllocator<sx::compute::MultibodyLinkContactRow>
        linkRowAllocator{memoryManager.getFreeAllocator()};

    EXPECT_EQ(
        rigidProblem.constraints.get_allocator(), rigidConstraintAllocator);
    EXPECT_EQ(problem.rowOwners.get_allocator(), rowOwnerAllocator);
    EXPECT_EQ(
        problem.rigidConstraints.get_allocator(), rigidConstraintAllocator);
    EXPECT_EQ(problem.multibodyBlocks.get_allocator(), blockAllocator);

    rigidProblem.constraints.reserve(1);
    problem.rowOwners.reserve(3);
    problem.rigidConstraints.reserve(1);
    problem.multibodyBlocks.reserve(1);
    problem.resizeMultibodyBlocks(1);
    ASSERT_EQ(problem.multibodyBlocks.size(), 1u);
    EXPECT_EQ(
        problem.multibodyBlocks[0].rows.get_allocator(), linkRowAllocator);
    problem.multibodyBlocks[0].rows.reserve(2);

    EXPECT_GT(freeList.getAllocationCount(), allocationsBeforeProblem)
        << "allocator-aware unified problem containers should reserve from "
           "the provided free allocator";
  }

  EXPECT_EQ(freeList.getAllocationCount(), allocationsBeforeProblem);
}

//==============================================================================
TEST(UnifiedConstraint, SolveScratchVectorsUseProvidedAllocator)
{
  namespace common = dart::common;

  common::MemoryManager memoryManager;
  auto& freeList = memoryManager.getFreeListAllocator();
  const auto allocationsBeforeScratch = freeList.getAllocationCount();

  {
    sx::compute::UnifiedConstraintSolveScratch scratch(
        memoryManager.getFreeAllocator());

    const common::StlAllocator<Eigen::Index> indexAllocator{
        memoryManager.getFreeAllocator()};
    const common::StlAllocator<char> charAllocator{
        memoryManager.getFreeAllocator()};
    const common::StlAllocator<double> doubleAllocator{
        memoryManager.getFreeAllocator()};
    const common::StlAllocator<std::size_t> sizeAllocator{
        memoryManager.getFreeAllocator()};

    EXPECT_EQ(scratch.islandRows.get_allocator(), indexAllocator);
    EXPECT_EQ(scratch.islandOffsets.get_allocator(), indexAllocator);
    EXPECT_EQ(scratch.visitedRows.get_allocator(), charAllocator);
    EXPECT_EQ(scratch.rowStack.get_allocator(), indexAllocator);
    EXPECT_EQ(scratch.localIndex.get_allocator(), indexAllocator);
    EXPECT_EQ(scratch.normalRows.get_allocator(), indexAllocator);
    EXPECT_EQ(scratch.rigidTangent1.get_allocator(), doubleAllocator);
    EXPECT_EQ(scratch.rigidTangent2.get_allocator(), doubleAllocator);
    EXPECT_EQ(scratch.linkTangentOffsets.get_allocator(), sizeAllocator);
    EXPECT_EQ(scratch.linkTangent1.get_allocator(), doubleAllocator);
    EXPECT_EQ(scratch.linkTangent2.get_allocator(), doubleAllocator);

    scratch.islandRows.reserve(6);
    scratch.islandOffsets.reserve(3);
    scratch.visitedRows.reserve(6);
    scratch.rowStack.reserve(6);
    scratch.localIndex.reserve(6);
    scratch.normalRows.reserve(2);
    scratch.rigidTangent1.reserve(2);
    scratch.rigidTangent2.reserve(2);
    scratch.linkTangentOffsets.reserve(2);
    scratch.linkTangent1.reserve(2);
    scratch.linkTangent2.reserve(2);

    EXPECT_GT(freeList.getAllocationCount(), allocationsBeforeScratch)
        << "allocator-aware unified solve scratch vectors should reserve from "
           "the provided free allocator";
  }

  EXPECT_EQ(freeList.getAllocationCount(), allocationsBeforeScratch);
}

//==============================================================================
// Three rigid bodies in a vertical stack on a static ground, mirroring the
// rigid-only determinism fixture: two single-point contacts that share the
// middle body.
sx::compute::RigidBodyContactProblem buildRigidStackProblem(sx::World& world)
{
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setFriction(0.25);

  sx::RigidBodyOptions lowerOptions;
  lowerOptions.position = Eigen::Vector3d(0.0, 0.0, 0.5);
  lowerOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto lower = world.addRigidBody("lower", lowerOptions);
  lower.setFriction(1.0);

  sx::RigidBodyOptions upperOptions;
  upperOptions.position = Eigen::Vector3d(0.0, 0.0, 1.5);
  upperOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -2.0);
  auto upper = world.addRigidBody("upper", upperOptions);
  upper.setFriction(0.25);

  std::vector<sx::Contact> contacts;
  sx::Contact groundLower;
  groundLower.bodyA = sx::CollisionBody(ground.getEntity(), &world);
  groundLower.bodyB = sx::CollisionBody(lower.getEntity(), &world);
  groundLower.point = Eigen::Vector3d(0.0, 0.0, 0.0);
  groundLower.normal = Eigen::Vector3d::UnitZ();
  groundLower.depth = 0.02;
  contacts.push_back(groundLower);

  sx::Contact lowerUpper;
  lowerUpper.bodyA = sx::CollisionBody(lower.getEntity(), &world);
  lowerUpper.bodyB = sx::CollisionBody(upper.getEntity(), &world);
  lowerUpper.point = Eigen::Vector3d(0.0, 0.0, 1.0);
  lowerUpper.normal = Eigen::Vector3d::UnitZ();
  lowerUpper.depth = 0.03;
  contacts.push_back(lowerUpper);

  return sx::compute::assembleRigidBodyContactProblem(
      dart::simulation::detail::registryOf(world), contacts);
}

} // namespace

void* operator new(std::size_t bytes)
{
  recordHeapAllocation(bytes);
  if (auto* ptr = allocateRaw(bytes)) {
    return ptr;
  }
  throw std::bad_alloc();
}

void* operator new[](std::size_t bytes)
{
  recordHeapAllocation(bytes);
  if (auto* ptr = allocateRaw(bytes)) {
    return ptr;
  }
  throw std::bad_alloc();
}

void* operator new(std::size_t bytes, const std::nothrow_t&) noexcept
{
  recordHeapAllocation(bytes);
  return allocateRaw(bytes);
}

void* operator new[](std::size_t bytes, const std::nothrow_t&) noexcept
{
  recordHeapAllocation(bytes);
  return allocateRaw(bytes);
}

void* operator new(std::size_t bytes, std::align_val_t alignment)
{
  recordHeapAllocation(bytes);
  if (auto* ptr
      = allocateAlignedRaw(bytes, static_cast<std::size_t>(alignment))) {
    return ptr;
  }
  throw std::bad_alloc();
}

void* operator new[](std::size_t bytes, std::align_val_t alignment)
{
  recordHeapAllocation(bytes);
  if (auto* ptr
      = allocateAlignedRaw(bytes, static_cast<std::size_t>(alignment))) {
    return ptr;
  }
  throw std::bad_alloc();
}

void* operator new(
    std::size_t bytes,
    std::align_val_t alignment,
    const std::nothrow_t&) noexcept
{
  recordHeapAllocation(bytes);
  return allocateAlignedRaw(bytes, static_cast<std::size_t>(alignment));
}

void* operator new[](
    std::size_t bytes,
    std::align_val_t alignment,
    const std::nothrow_t&) noexcept
{
  recordHeapAllocation(bytes);
  return allocateAlignedRaw(bytes, static_cast<std::size_t>(alignment));
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
  std::free(pointer);
}

void operator delete[](void* pointer, const std::nothrow_t&) noexcept
{
  std::free(pointer);
}

void operator delete(void* pointer, std::align_val_t alignment) noexcept
{
  deallocateAlignedRaw(pointer, static_cast<std::size_t>(alignment));
}

void operator delete[](void* pointer, std::align_val_t alignment) noexcept
{
  deallocateAlignedRaw(pointer, static_cast<std::size_t>(alignment));
}

void operator delete(
    void* pointer, std::size_t, std::align_val_t alignment) noexcept
{
  deallocateAlignedRaw(pointer, static_cast<std::size_t>(alignment));
}

void operator delete[](
    void* pointer, std::size_t, std::align_val_t alignment) noexcept
{
  deallocateAlignedRaw(pointer, static_cast<std::size_t>(alignment));
}

void operator delete(
    void* pointer, std::align_val_t alignment, const std::nothrow_t&) noexcept
{
  deallocateAlignedRaw(pointer, static_cast<std::size_t>(alignment));
}

void operator delete[](
    void* pointer, std::align_val_t alignment, const std::nothrow_t&) noexcept
{
  deallocateAlignedRaw(pointer, static_cast<std::size_t>(alignment));
}

//==============================================================================
TEST(UnifiedConstraint, MultibodyFreeReproducesRigidProblemByteForByte)
{
  sx::World world;
  const auto rigid = buildRigidStackProblem(world);
  ASSERT_EQ(rigid.constraints.size(), 2u);

  const std::span<const sx::compute::UnifiedMultibodyContact> noMultibodies{};
  const auto unified
      = sx::compute::assembleUnifiedConstraintProblem(rigid, noMultibodies);

  // Same dimensions and the rigid A,b,lo,hi,findex copied verbatim.
  ASSERT_EQ(unified.delassus.rows(), rigid.delassus.rows());
  expectMatrixExactlyEqual(unified.delassus, rigid.delassus);
  expectVectorExactlyEqual(unified.rhs, rigid.rhs);
  expectVectorExactlyEqual(unified.lo, rigid.lo);
  expectVectorExactlyEqual(unified.hi, rigid.hi);
  expectVectorExactlyEqual(unified.findex, rigid.findex);

  EXPECT_TRUE(unified.multibodyBlocks.empty());
  ASSERT_EQ(unified.rigidConstraints.size(), rigid.constraints.size());

  // Every row is a rigid row with the same normal-row grouping (3-stride).
  ASSERT_EQ(
      unified.rowOwners.size(), static_cast<std::size_t>(rigid.rhs.size()));
  for (std::size_t i = 0; i < unified.rowOwners.size(); ++i) {
    const auto& owner = unified.rowOwners[i];
    EXPECT_EQ(owner.domain, sx::compute::UnifiedContactDomain::Rigid);
    EXPECT_EQ(owner.multibodyIndex, -1);
    EXPECT_EQ(owner.normalRowGlobalIndex, static_cast<Eigen::Index>(i / 3) * 3);
  }
}

//==============================================================================
TEST(UnifiedConstraint, InPlaceAssemblerMatchesReturnValue)
{
  sx::World world;
  const auto rigid = buildRigidStackProblem(world);
  ASSERT_EQ(rigid.constraints.size(), 2u);

  const std::span<const sx::compute::UnifiedMultibodyContact> noMultibodies{};
  const auto expected
      = sx::compute::assembleUnifiedConstraintProblem(rigid, noMultibodies);

  sx::compute::UnifiedConstraintProblem actual;
  actual.delassus.resize(1, 1);
  actual.rhs.resize(1);
  actual.lo.resize(1);
  actual.hi.resize(1);
  actual.findex.resize(1);
  actual.rowOwners.reserve(32);
  actual.rigidConstraints.reserve(4);
  actual.multibodyBlocks.reserve(4);

  sx::compute::assembleUnifiedConstraintProblemInto(
      actual, rigid, noMultibodies);

  expectMatrixExactlyEqual(actual.delassus, expected.delassus);
  expectVectorExactlyEqual(actual.rhs, expected.rhs);
  expectVectorExactlyEqual(actual.lo, expected.lo);
  expectVectorExactlyEqual(actual.hi, expected.hi);
  expectVectorExactlyEqual(actual.findex, expected.findex);
  ASSERT_EQ(actual.rowOwners.size(), expected.rowOwners.size());
  for (std::size_t i = 0; i < actual.rowOwners.size(); ++i) {
    EXPECT_EQ(actual.rowOwners[i].domain, expected.rowOwners[i].domain);
    EXPECT_EQ(actual.rowOwners[i].direction, expected.rowOwners[i].direction);
    EXPECT_EQ(
        actual.rowOwners[i].normalRowGlobalIndex,
        expected.rowOwners[i].normalRowGlobalIndex);
    EXPECT_EQ(
        actual.rowOwners[i].sourceIndex, expected.rowOwners[i].sourceIndex);
    EXPECT_EQ(
        actual.rowOwners[i].multibodyIndex,
        expected.rowOwners[i].multibodyIndex);
  }
  EXPECT_EQ(actual.rigidConstraints.size(), expected.rigidConstraints.size());
  EXPECT_TRUE(actual.multibodyBlocks.empty());
}

//==============================================================================
TEST(UnifiedConstraint, InPlaceAssemblerReusesSameShapeLinkStorage)
{
  sx::compute::RigidBodyContactProblem rigid;

  sx::compute::MultibodyLinkContactProblem linkProblem;
  linkProblem.inverseMass.resize(2, 2);
  linkProblem.inverseMass << 2.0, 0.25, 0.25, 1.5;
  linkProblem.rows.resize(2);
  for (std::size_t i = 0; i < linkProblem.rows.size(); ++i) {
    auto& row = linkProblem.rows[i];
    row.active = true;
    row.normalJacobian.resize(2);
    row.tangentJacobian1.resize(2);
    row.tangentJacobian2.resize(2);
    row.otherNormalJacobian.resize(2);
    row.otherTangentJacobian1.resize(2);
    row.otherTangentJacobian2.resize(2);
    row.normalJacobian << 1.0 + static_cast<double>(i), 0.5;
    row.tangentJacobian1 << 0.25, 0.75 + static_cast<double>(i);
    row.tangentJacobian2 << 0.5, -0.25 - static_cast<double>(i);
    row.otherNormalJacobian.setZero();
    row.otherTangentJacobian1.setZero();
    row.otherTangentJacobian2.setZero();
    row.normalRhs = 1.0 + static_cast<double>(i);
    row.tangentRhs1 = -0.25;
    row.tangentRhs2 = 0.125;
    row.friction = 0.4;
  }

  std::vector<sx::compute::UnifiedMultibodyContact> contacts(1);
  contacts[0].multibody = static_cast<entt::entity>(1);
  contacts[0].borrowedProblem = &linkProblem;

  sx::compute::UnifiedConstraintProblem problem;
  sx::compute::assembleUnifiedConstraintProblemInto(problem, rigid, contacts);
  sx::compute::assembleUnifiedConstraintProblemInto(problem, rigid, contacts);

  std::size_t allocations = 0;
  std::size_t bytes = 0;
  {
    ScopedHeapAllocationCounter heapCounter;
    sx::compute::assembleUnifiedConstraintProblemInto(problem, rigid, contacts);
    allocations = heapCounter.allocations();
    bytes = heapCounter.bytes();
  }

  EXPECT_EQ(allocations, 0u) << "allocated " << bytes << " bytes";
  ASSERT_EQ(problem.multibodyBlocks.size(), 1u);
  ASSERT_EQ(problem.multibodyBlocks[0].rows.size(), 2u);
  EXPECT_EQ(problem.rhs.size(), 6);
}

//==============================================================================
TEST(UnifiedConstraint, InPlaceAssemblerReusesSameShapeMixedStorage)
{
  sx::World world;
  const auto rigid = buildRigidStackProblem(world);
  ASSERT_EQ(rigid.constraints.size(), 2u);

  sx::compute::MultibodyLinkContactProblem linkProblem;
  linkProblem.inverseMass.resize(1, 1);
  linkProblem.inverseMass << 2.0;
  linkProblem.rows.resize(1);
  auto& row = linkProblem.rows[0];
  row.active = true;
  row.normalJacobian.resize(1);
  row.tangentJacobian1.resize(1);
  row.tangentJacobian2.resize(1);
  row.otherNormalJacobian.resize(1);
  row.otherTangentJacobian1.resize(1);
  row.otherTangentJacobian2.resize(1);
  row.normalJacobian << 1.0;
  row.tangentJacobian1 << 0.25;
  row.tangentJacobian2 << -0.5;
  row.otherNormalJacobian.setZero();
  row.otherTangentJacobian1.setZero();
  row.otherTangentJacobian2.setZero();
  row.normalRhs = 0.5;
  row.tangentRhs1 = 0.125;
  row.tangentRhs2 = -0.25;
  row.friction = 0.3;

  std::vector<sx::compute::UnifiedMultibodyContact> contacts(1);
  contacts[0].multibody = static_cast<entt::entity>(1);
  contacts[0].borrowedProblem = &linkProblem;

  sx::compute::UnifiedConstraintProblem problem;
  sx::compute::assembleUnifiedConstraintProblemInto(problem, rigid, contacts);
  sx::compute::assembleUnifiedConstraintProblemInto(problem, rigid, contacts);

  std::size_t allocations = 0;
  std::size_t bytes = 0;
  {
    ScopedHeapAllocationCounter heapCounter;
    sx::compute::assembleUnifiedConstraintProblemInto(problem, rigid, contacts);
    allocations = heapCounter.allocations();
    bytes = heapCounter.bytes();
  }

  EXPECT_EQ(allocations, 0u) << "allocated " << bytes << " bytes";
  ASSERT_EQ(problem.rigidConstraints.size(), rigid.constraints.size());
  ASSERT_EQ(problem.multibodyBlocks.size(), 1u);
  ASSERT_EQ(problem.multibodyBlocks[0].rows.size(), 1u);
  EXPECT_EQ(problem.rhs.size(), 9);
}

//==============================================================================
TEST(UnifiedConstraint, PublicLinkScratchFeedsBorrowedUnifiedAssembly)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "slider";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto link = robot.addLink("link", base, spec);
  link.setMass(1.0);

  const auto robotEntity
      = dart::simulation::detail::toRegistryEntity(robot.getEntity());
  const auto linkEntity
      = dart::simulation::detail::toRegistryEntity(link.getEntity());

  sx::compute::LinkContact contact;
  contact.link = linkEntity;
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.point = Eigen::Vector3d::Zero();
  contact.depth = 0.01;
  contact.friction = 0.5;

  Eigen::VectorXd nextVelocity(1);
  nextVelocity << -0.5;
  const std::vector<sx::compute::LinkContact> linkContacts{contact};

  auto& registry = dart::simulation::detail::registryOf(world);
  const auto& structure
      = registry.get<sx::comps::MultibodyStructure>(robotEntity);
  sx::compute::MultibodyLinkContactAssemblyScratch linkScratch;
  ASSERT_TRUE(
      sx::compute::assembleMultibodyLinkContactProblemInto(
          linkScratch, registry, structure, nextVelocity, 0.01, linkContacts));

  const auto expected = sx::compute::assembleMultibodyLinkContactProblem(
      registry, structure, nextVelocity, 0.01, linkContacts);
  ASSERT_EQ(linkScratch.getProblem().rows.size(), expected.rows.size());
  ASSERT_EQ(linkScratch.getProblem().rows.size(), 1u);
  EXPECT_EQ(linkScratch.getProblem().inverseMass, expected.inverseMass);
  EXPECT_EQ(
      linkScratch.getProblem().rows[0].normalJacobian,
      expected.rows[0].normalJacobian);

  sx::compute::RigidBodyContactProblem emptyRigid;
  std::vector<sx::compute::UnifiedMultibodyContact> multibodyContacts(1);
  multibodyContacts[0].multibody = robotEntity;
  multibodyContacts[0].borrowedProblem = &linkScratch.getProblem();

  sx::compute::UnifiedConstraintProblem unified;
  sx::compute::assembleUnifiedConstraintProblemInto(
      unified, emptyRigid, multibodyContacts);
  ASSERT_TRUE(
      sx::compute::assembleMultibodyLinkContactProblemInto(
          linkScratch, registry, structure, nextVelocity, 0.01, linkContacts));
  sx::compute::assembleUnifiedConstraintProblemInto(
      unified, emptyRigid, multibodyContacts);

  std::size_t allocations = 0;
  std::size_t bytes = 0;
  {
    ScopedHeapAllocationCounter heapCounter;
    ASSERT_TRUE(
        sx::compute::assembleMultibodyLinkContactProblemInto(
            linkScratch,
            registry,
            structure,
            nextVelocity,
            0.01,
            linkContacts));
    sx::compute::assembleUnifiedConstraintProblemInto(
        unified, emptyRigid, multibodyContacts);
    allocations = heapCounter.allocations();
    bytes = heapCounter.bytes();
  }

  EXPECT_EQ(allocations, 0u) << "allocated " << bytes << " bytes";
  ASSERT_EQ(unified.multibodyBlocks.size(), 1u);
  ASSERT_EQ(unified.multibodyBlocks[0].rows.size(), 1u);
  EXPECT_EQ(unified.rhs.size(), 3);
}

//==============================================================================
TEST(UnifiedConstraint, RigidFreeLinkBlockMatchesWithinMultibodyCoupling)
{
  // A fixed-base revolute link about +Z with the link frame on the axis; two
  // contacts at different radii give two ACTIVE rows with distinct point
  // Jacobians, so the within-multibody off-diagonal coupling is non-trivial.
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");

  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto arm = robot.addLink("arm", base, spec);
  arm.setMass(1.0);
  arm.setInertia(Eigen::Matrix3d::Identity());

  sx::compute::LinkContact nearContact;
  nearContact.link
      = dart::simulation::detail::toRegistryEntity(arm.getEntity());
  nearContact.normal = Eigen::Vector3d::UnitY();
  nearContact.point = Eigen::Vector3d(1.0, 0.0, 0.0);
  nearContact.depth = 0.01;
  nearContact.friction = 0.5;

  sx::compute::LinkContact farContact;
  farContact.link = dart::simulation::detail::toRegistryEntity(arm.getEntity());
  farContact.normal = Eigen::Vector3d::UnitY();
  farContact.point = Eigen::Vector3d(2.0, 0.0, 0.0);
  farContact.depth = 0.0;
  farContact.friction = 0.5;

  Eigen::VectorXd nextVelocity(1);
  nextVelocity << -0.3;

  const std::vector<sx::compute::LinkContact> linkContacts{
      nearContact, farContact};
  auto& registry = dart::simulation::detail::registryOf(world);
  const auto& structure = registry.get<sx::comps::MultibodyStructure>(
      dart::simulation::detail::toRegistryEntity(robot.getEntity()));
  auto linkProblem = sx::compute::assembleMultibodyLinkContactProblem(
      registry, structure, nextVelocity, 0.01, linkContacts);

  std::vector<sx::compute::UnifiedMultibodyContact> multibodyContacts;
  multibodyContacts.push_back(
      {dart::simulation::detail::toRegistryEntity(robot.getEntity()),
       linkProblem});

  sx::compute::RigidBodyContactProblem emptyRigid; // no rigid contacts
  const auto unified = sx::compute::assembleUnifiedConstraintProblem(
      emptyRigid, multibodyContacts);

  // Two active link contacts -> 6 rows, all link, one block at base 0.
  ASSERT_EQ(unified.delassus.rows(), 6);
  EXPECT_TRUE(unified.rigidConstraints.empty());
  ASSERT_EQ(unified.multibodyBlocks.size(), 1u);
  const auto& block = unified.multibodyBlocks[0];
  EXPECT_EQ(
      block.multibody,
      dart::simulation::detail::toRegistryEntity(robot.getEntity()));
  EXPECT_EQ(block.blockBase, 0);
  ASSERT_EQ(block.rows.size(), 2u);

  const Eigen::MatrixXd& inverseMass = block.inverseMass;
  const auto jacobianOf = [](const sx::compute::MultibodyLinkContactRow& row,
                             int dir) -> const Eigen::VectorXd& {
    return dir == 0 ? row.normalJacobian
                    : (dir == 1 ? row.tangentJacobian1 : row.tangentJacobian2);
  };

  // The full dense within-multibody block equals J_i^T M^-1 J_j, evaluated in
  // the assembler's exact order (so the diagonal is bit-identical to the stored
  // denominators).
  for (Eigen::Index ci = 0; ci < 2; ++ci) {
    for (int a = 0; a < 3; ++a) {
      const Eigen::VectorXd inverseMassJacobian
          = inverseMass * jacobianOf(block.rows[ci], a);
      for (Eigen::Index cj = 0; cj < 2; ++cj) {
        for (int b = 0; b < 3; ++b) {
          const double expected
              = jacobianOf(block.rows[cj], b).dot(inverseMassJacobian);
          EXPECT_DOUBLE_EQ(unified.delassus(ci * 3 + a, cj * 3 + b), expected)
              << "block entry (" << ci << "," << a << ")x(" << cj << "," << b
              << ")";
        }
      }
    }
  }

  // The diagonal reproduces the stored row denominators (obstacle-free).
  EXPECT_DOUBLE_EQ(unified.delassus(0, 0), block.rows[0].normalDenominator);
  EXPECT_DOUBLE_EQ(unified.delassus(1, 1), block.rows[0].tangentDenominator1);
  EXPECT_DOUBLE_EQ(unified.delassus(2, 2), block.rows[0].tangentDenominator2);
  EXPECT_DOUBLE_EQ(unified.delassus(3, 3), block.rows[1].normalDenominator);

  // The two contacts have distinct normal Jacobians, so the normal-normal
  // off-diagonal coupling is genuinely non-trivial and present.
  EXPECT_NE(block.rows[0].normalJacobian, block.rows[1].normalJacobian);
  EXPECT_NE(unified.delassus(0, 3), 0.0);
  EXPECT_TRUE(unified.delassus.isApprox(unified.delassus.transpose(), 1e-12));

  // Right-hand sides carried from the rows.
  EXPECT_DOUBLE_EQ(unified.rhs[0], block.rows[0].normalRhs);
  EXPECT_DOUBLE_EQ(unified.rhs[1], block.rows[0].tangentRhs1);
  EXPECT_DOUBLE_EQ(unified.rhs[2], block.rows[0].tangentRhs2);
  EXPECT_DOUBLE_EQ(unified.rhs[3], block.rows[1].normalRhs);

  // Bounds + findex against GLOBAL row indices; rows are all Link.
  for (Eigen::Index c = 0; c < 2; ++c) {
    const Eigen::Index normalRow = c * 3;
    EXPECT_DOUBLE_EQ(unified.lo[normalRow], 0.0);
    EXPECT_TRUE(std::isinf(unified.hi[normalRow]));
    EXPECT_EQ(unified.findex[normalRow], -1);
    for (int t = 1; t < 3; ++t) {
      EXPECT_DOUBLE_EQ(unified.lo[normalRow + t], -block.rows[c].friction);
      EXPECT_DOUBLE_EQ(unified.hi[normalRow + t], block.rows[c].friction);
      EXPECT_EQ(unified.findex[normalRow + t], static_cast<int>(normalRow));
    }
    for (int t = 0; t < 3; ++t) {
      const auto& owner
          = unified.rowOwners[static_cast<std::size_t>(normalRow + t)];
      EXPECT_EQ(owner.domain, sx::compute::UnifiedContactDomain::Link);
      EXPECT_EQ(owner.multibodyIndex, 0);
      EXPECT_EQ(owner.normalRowGlobalIndex, normalRow);
    }
  }
}

//==============================================================================
TEST(UnifiedConstraint, CrossMultibodyLinkRowAddsSecondArticulatedEnd)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  const auto addPrismaticRobot = [&](const std::string& name) {
    auto robot = world.addMultibody(name);
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "slider";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitZ();
    auto link = robot.addLink("link", base, spec);
    link.setMass(1.0);
    return std::pair{
        dart::simulation::detail::toRegistryEntity(robot.getEntity()),
        dart::simulation::detail::toRegistryEntity(link.getEntity())};
  };

  const auto [robotA, linkA] = addPrismaticRobot("a");
  const auto [robotB, linkB] = addPrismaticRobot("b");

  Eigen::VectorXd velocityA(1);
  velocityA << -0.5;
  Eigen::VectorXd velocityB(1);
  velocityB << 0.25;

  sx::compute::LinkContact cross;
  cross.link = linkA;
  cross.otherLink = linkB;
  cross.otherMultibody = robotB;
  cross.normal = Eigen::Vector3d::UnitZ();
  cross.point = Eigen::Vector3d::Zero();
  cross.depth = 0.0;
  cross.friction = 0.5;

  auto& registry = dart::simulation::detail::registryOf(world);
  const auto& structureA = registry.get<sx::comps::MultibodyStructure>(robotA);
  const auto& structureB = registry.get<sx::comps::MultibodyStructure>(robotB);

  std::vector<sx::compute::LinkContact> contactsA{cross};
  auto problemA = sx::compute::assembleMultibodyLinkContactProblem(
      registry, structureA, velocityA, 0.01, contactsA);
  ASSERT_EQ(problemA.rows.size(), 1u);
  auto& crossRow = problemA.rows[0];
  EXPECT_FALSE(crossRow.active); // completed by UnifiedConstraintStage normally
  crossRow.otherNormalJacobian = Eigen::VectorXd::Ones(1);
  crossRow.otherTangentJacobian1 = Eigen::VectorXd::Zero(1);
  crossRow.otherTangentJacobian2 = Eigen::VectorXd::Zero(1);

  std::vector<sx::compute::LinkContact> contactB{sx::compute::LinkContact{
      linkB, Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(), 0.0, 0.5, 0.0}};
  auto problemB = sx::compute::assembleMultibodyLinkContactProblem(
      registry, structureB, velocityB, 0.01, contactB);
  ASSERT_EQ(problemB.rows.size(), 1u);

  crossRow.normalDenominator += crossRow.otherNormalJacobian.dot(
      problemB.inverseMass * crossRow.otherNormalJacobian);
  crossRow.normalRhs = 0.75; // -(velocityA - velocityB)
  crossRow.active = true;

  std::vector<sx::compute::UnifiedMultibodyContact> multibodyContacts;
  multibodyContacts.push_back({robotA, problemA});
  multibodyContacts.push_back({robotB, problemB});
  sx::compute::RigidBodyContactProblem emptyRigid;
  const auto unified = sx::compute::assembleUnifiedConstraintProblem(
      emptyRigid, multibodyContacts);

  ASSERT_EQ(unified.multibodyBlocks.size(), 2u);
  ASSERT_EQ(unified.multibodyBlocks[0].rows.size(), 1u);
  ASSERT_EQ(unified.multibodyBlocks[1].rows.size(), 1u);
  EXPECT_EQ(unified.multibodyBlocks[0].rows[0].otherMultibodyIndex, 1);
  ASSERT_EQ(unified.delassus.rows(), 6);

  // Cross row self-term = A-side unit inverse mass + B-side unit inverse mass.
  EXPECT_DOUBLE_EQ(unified.delassus(0, 0), 2.0);
  // Coupling to B's own link row carries the opposite sign on the shared
  // articulated B end.
  EXPECT_DOUBLE_EQ(unified.delassus(0, 3), -1.0);
  EXPECT_DOUBLE_EQ(unified.delassus(3, 0), -1.0);
  EXPECT_TRUE(unified.delassus.isApprox(unified.delassus.transpose(), 1e-12));

  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(6);
  lambda[0] = 1.0;
  std::vector<Eigen::VectorXd> velocities{
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1)};
  sx::compute::applyUnifiedConstraintImpulses(
      registry, unified, lambda, std::span<Eigen::VectorXd>(velocities));

  EXPECT_DOUBLE_EQ(velocities[0][0], 1.0);
  EXPECT_DOUBLE_EQ(velocities[1][0], -1.0);
}

//==============================================================================
TEST(UnifiedConstraint, FallbackFrictionUpdatesCrossMultibodyOtherEnd)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  const auto addTwoAxisRobot = [&](const std::string& name) {
    auto robot = world.addMultibody(name);
    auto base = robot.addLink("base");
    sx::JointSpec zSpec;
    zSpec.name = "z";
    zSpec.type = sx::JointType::Prismatic;
    zSpec.axis = Eigen::Vector3d::UnitZ();
    auto zLink = robot.addLink("z_link", base, zSpec);
    sx::JointSpec xSpec;
    xSpec.name = "x";
    xSpec.type = sx::JointType::Prismatic;
    xSpec.axis = Eigen::Vector3d::UnitX();
    auto tip = robot.addLink("tip", zLink, xSpec);
    tip.setMass(1.0);
    return std::pair{
        dart::simulation::detail::toRegistryEntity(robot.getEntity()),
        dart::simulation::detail::toRegistryEntity(tip.getEntity())};
  };

  const auto [robotA, linkA] = addTwoAxisRobot("a");
  const auto [robotB, linkB] = addTwoAxisRobot("b");

  Eigen::VectorXd velocityA(2);
  velocityA << -1.0, 1.0;
  Eigen::VectorXd velocityB(2);
  velocityB << 1.0, -1.0;

  sx::compute::LinkContact cross;
  cross.link = linkA;
  cross.otherLink = linkB;
  cross.otherMultibody = robotB;
  cross.normal = Eigen::Vector3d::UnitZ();
  cross.point = Eigen::Vector3d::Zero();
  cross.friction = 0.5;

  sx::compute::LinkContact probe;
  probe.link = linkB;
  probe.normal = cross.normal;
  probe.point = cross.point;
  probe.friction = cross.friction;

  auto& registry = dart::simulation::detail::registryOf(world);
  const auto& structureA = registry.get<sx::comps::MultibodyStructure>(robotA);
  const auto& structureB = registry.get<sx::comps::MultibodyStructure>(robotB);
  std::vector<sx::compute::LinkContact> contactsA{cross};
  std::vector<sx::compute::LinkContact> probeContactsB{probe};

  auto problemA = sx::compute::assembleMultibodyLinkContactProblem(
      registry, structureA, velocityA, 0.01, contactsA);
  auto probeProblemB = sx::compute::assembleMultibodyLinkContactProblem(
      registry, structureB, velocityB, 0.01, probeContactsB);
  auto problemB = sx::compute::assembleMultibodyLinkContactProblem(
      registry, structureB, velocityB, 0.01, {});
  ASSERT_EQ(problemA.rows.size(), 1u);
  ASSERT_EQ(probeProblemB.rows.size(), 1u);

  auto& row = problemA.rows[0];
  row.otherNormalJacobian = probeProblemB.rows[0].normalJacobian;
  row.otherTangentJacobian1 = probeProblemB.rows[0].tangentJacobian1;
  row.otherTangentJacobian2 = probeProblemB.rows[0].tangentJacobian2;
  row.normalDenominator += row.otherNormalJacobian.dot(
      problemB.inverseMass * row.otherNormalJacobian);
  row.tangentDenominator1 += row.otherTangentJacobian1.dot(
      problemB.inverseMass * row.otherTangentJacobian1);
  row.tangentDenominator2 += row.otherTangentJacobian2.dot(
      problemB.inverseMass * row.otherTangentJacobian2);
  row.normalRhs
      = -(row.normalJacobian.dot(velocityA)
          - row.otherNormalJacobian.dot(velocityB));
  row.tangentRhs1
      = -(row.tangentJacobian1.dot(velocityA)
          - row.otherTangentJacobian1.dot(velocityB));
  row.tangentRhs2
      = -(row.tangentJacobian2.dot(velocityA)
          - row.otherTangentJacobian2.dot(velocityB));
  row.active = true;

  std::vector<sx::compute::UnifiedMultibodyContact> multibodyContacts;
  multibodyContacts.push_back({robotA, problemA});
  multibodyContacts.push_back({robotB, problemB});
  sx::compute::RigidBodyContactProblem emptyRigid;
  const auto unified = sx::compute::assembleUnifiedConstraintProblem(
      emptyRigid, multibodyContacts);
  ASSERT_EQ(unified.multibodyBlocks.size(), 2u);
  ASSERT_EQ(unified.multibodyBlocks[0].rows.size(), 1u);
  ASSERT_TRUE(unified.multibodyBlocks[1].rows.empty());
  const auto& unifiedRow = unified.multibodyBlocks[0].rows[0];
  EXPECT_EQ(unifiedRow.otherMultibodyIndex, 1);

  std::vector<Eigen::VectorXd> velocities{velocityA, velocityB};
  const double tangentBefore
      = unifiedRow.tangentJacobian2.dot(velocities[0])
        - unifiedRow.otherTangentJacobian2.dot(velocities[1]);
  sx::compute::applyUnifiedConstraintFallback(
      registry, unified, std::span<Eigen::VectorXd>(velocities), 8);

  const double normalAfter
      = unifiedRow.normalJacobian.dot(velocities[0])
        - unifiedRow.otherNormalJacobian.dot(velocities[1]);
  const double tangentAfter
      = unifiedRow.tangentJacobian2.dot(velocities[0])
        - unifiedRow.otherTangentJacobian2.dot(velocities[1]);
  EXPECT_NEAR(normalAfter, 0.0, 1e-9);
  EXPECT_LT(std::abs(tangentAfter), std::abs(tangentBefore));
  EXPECT_LT(velocities[0][1], velocityA[1]);
  EXPECT_GT(velocities[1][1], velocityB[1]);

  sx::compute::UnifiedConstraintSolveScratch scratch;
  std::vector<Eigen::VectorXd> scratchVelocities{velocityA, velocityB};
  sx::compute::applyUnifiedConstraintFallback(
      registry,
      unified,
      std::span<Eigen::VectorXd>(scratchVelocities),
      8,
      scratch);
  scratchVelocities = {velocityA, velocityB};
  sx::compute::applyUnifiedConstraintFallback(
      registry,
      unified,
      std::span<Eigen::VectorXd>(scratchVelocities),
      8,
      scratch);

  std::size_t allocations = 0;
  std::size_t bytes = 0;
  scratchVelocities = {velocityA, velocityB};
  {
    ScopedHeapAllocationCounter heapCounter;
    sx::compute::applyUnifiedConstraintFallback(
        registry,
        unified,
        std::span<Eigen::VectorXd>(scratchVelocities),
        8,
        scratch);
    allocations = heapCounter.allocations();
    bytes = heapCounter.bytes();
  }

  EXPECT_EQ(allocations, 0u) << "allocated " << bytes << " bytes";
}

//==============================================================================
TEST(UnifiedConstraint, FindexReferencesValidNormalRows)
{
  sx::World world;
  const auto rigid = buildRigidStackProblem(world);

  sx::World linkWorld;
  linkWorld.setGravity(Eigen::Vector3d::Zero());
  auto robot = linkWorld.addMultibody("robot");
  auto baseLink = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "slider";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto leg = robot.addLink("leg", baseLink, spec);
  leg.setMass(1.0);

  sx::compute::LinkContact contact;
  contact.link = dart::simulation::detail::toRegistryEntity(leg.getEntity());
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.point = Eigen::Vector3d::Zero();
  contact.depth = 0.01;
  contact.friction = 0.4;

  Eigen::VectorXd nextVelocity(1);
  nextVelocity << -0.5;
  const std::vector<sx::compute::LinkContact> linkContacts{contact};
  auto& registry = dart::simulation::detail::registryOf(linkWorld);
  const auto& structure = registry.get<sx::comps::MultibodyStructure>(
      dart::simulation::detail::toRegistryEntity(robot.getEntity()));
  auto linkProblem = sx::compute::assembleMultibodyLinkContactProblem(
      registry, structure, nextVelocity, 0.01, linkContacts);

  std::vector<sx::compute::UnifiedMultibodyContact> multibodyContacts;
  multibodyContacts.push_back(
      {dart::simulation::detail::toRegistryEntity(robot.getEntity()),
       linkProblem});
  const auto unified
      = sx::compute::assembleUnifiedConstraintProblem(rigid, multibodyContacts);

  const Eigen::Index size = unified.findex.size();
  for (Eigen::Index r = 0; r < size; ++r) {
    const int fi = unified.findex[r];
    if (fi < 0) {
      // A normal row references nothing.
      EXPECT_EQ(
          unified.rowOwners[static_cast<std::size_t>(r)].direction,
          sx::compute::UnifiedContactDirection::Normal);
      continue;
    }
    // A friction row references a valid normal row, never itself, in range.
    EXPECT_LT(fi, size);
    EXPECT_NE(fi, static_cast<int>(r));
    EXPECT_EQ(unified.findex[fi], -1) << "findex target must be a normal row";
    EXPECT_EQ(
        unified.rowOwners[static_cast<std::size_t>(fi)].direction,
        sx::compute::UnifiedContactDirection::Normal);
    // The friction row points at its OWN contact's normal row.
    EXPECT_EQ(
        fi,
        static_cast<int>(unified.rowOwners[static_cast<std::size_t>(r)]
                             .normalRowGlobalIndex));
  }
}

//==============================================================================
TEST(UnifiedConstraint, CompactsInactiveLinkRows)
{
  // A prismatic-Z leg cannot move along X, so an X-normal contact has zero
  // normal denominator and is left inactive; it must be compacted out so it
  // never enters the matrix nor makes it singular.
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  auto robot = world.addMultibody("robot");
  auto baseLink = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "slider";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto leg = robot.addLink("leg", baseLink, spec);
  leg.setMass(1.0);

  sx::compute::LinkContact active;
  active.link = dart::simulation::detail::toRegistryEntity(leg.getEntity());
  active.normal = Eigen::Vector3d::UnitZ(); // along the slide axis -> active
  active.point = Eigen::Vector3d::Zero();
  active.depth = 0.01;
  active.friction = 0.5;

  sx::compute::LinkContact inactive;
  inactive.link = dart::simulation::detail::toRegistryEntity(leg.getEntity());
  inactive.normal = Eigen::Vector3d::UnitX(); // orthogonal to slide -> inactive
  inactive.point = Eigen::Vector3d::Zero();
  inactive.depth = 0.01;
  inactive.friction = 0.5;

  Eigen::VectorXd nextVelocity(1);
  nextVelocity << -0.5;
  const std::vector<sx::compute::LinkContact> linkContacts{active, inactive};
  auto& registry = dart::simulation::detail::registryOf(world);
  const auto& structure = registry.get<sx::comps::MultibodyStructure>(
      dart::simulation::detail::toRegistryEntity(robot.getEntity()));
  auto linkProblem = sx::compute::assembleMultibodyLinkContactProblem(
      registry, structure, nextVelocity, 0.01, linkContacts);

  // The assembler emitted two rows; exactly one is active.
  ASSERT_EQ(linkProblem.rows.size(), 2u);
  int activeCount = 0;
  for (const auto& row : linkProblem.rows) {
    activeCount += row.active ? 1 : 0;
  }
  ASSERT_EQ(activeCount, 1);

  std::vector<sx::compute::UnifiedMultibodyContact> multibodyContacts;
  multibodyContacts.push_back(
      {dart::simulation::detail::toRegistryEntity(robot.getEntity()),
       linkProblem});
  sx::compute::RigidBodyContactProblem emptyRigid;
  const auto unified = sx::compute::assembleUnifiedConstraintProblem(
      emptyRigid, multibodyContacts);

  // Only the active contact survives: 3 rows, a non-singular diagonal.
  ASSERT_EQ(unified.multibodyBlocks.size(), 1u);
  ASSERT_EQ(unified.multibodyBlocks[0].rows.size(), 1u);
  ASSERT_EQ(unified.delassus.rows(), 3);
  EXPECT_GT(unified.delassus(0, 0), 0.0);
}

//==============================================================================
TEST(UnifiedConstraint, CouplesSharedDynamicObstacleAcrossDomains)
{
  // A dynamic rigid body R touches both a rigid contact (R on a static ground)
  // and a link contact (a prismatic-Z leg pushing on R). The unified system
  // must couple the rigid and link rows through R.
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("robot");
  auto baseLink = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "slider";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto leg = robot.addLink("leg", baseLink, spec);
  leg.setMass(1.0);

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.mass = 1.0;
  obstacleOptions.inertia = Eigen::Matrix3d::Identity();
  obstacleOptions.position = Eigen::Vector3d(0.05, -0.1, 1.0);
  auto obstacle = world.addRigidBody("obstacle", obstacleOptions);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);

  // Rigid contact: ground (A, static) vs obstacle (B, dynamic) -> R = bodyB.
  std::vector<sx::Contact> rigidContacts;
  sx::Contact groundObstacle;
  groundObstacle.bodyA = sx::CollisionBody(ground.getEntity(), &world);
  groundObstacle.bodyB = sx::CollisionBody(obstacle.getEntity(), &world);
  groundObstacle.point = Eigen::Vector3d(0.05, -0.1, -0.5);
  groundObstacle.normal = Eigen::Vector3d::UnitZ();
  groundObstacle.depth = 0.01;
  rigidContacts.push_back(groundObstacle);
  const auto rigid = sx::compute::assembleRigidBodyContactProblem(
      dart::simulation::detail::registryOf(world), rigidContacts);
  ASSERT_EQ(rigid.constraints.size(), 1u);
  ASSERT_EQ(
      rigid.constraints[0].bodyB,
      dart::simulation::detail::toRegistryEntity(obstacle.getEntity()));

  // Link contact: the prismatic leg pushes on the dynamic obstacle R.
  sx::compute::LinkContact legObstacle;
  legObstacle.link
      = dart::simulation::detail::toRegistryEntity(leg.getEntity());
  legObstacle.normal = Eigen::Vector3d::UnitZ();
  legObstacle.point = Eigen::Vector3d(0.05, -0.1, 0.5);
  legObstacle.depth = 0.0;
  legObstacle.friction = 0.5;
  legObstacle.otherBody
      = dart::simulation::detail::toRegistryEntity(obstacle.getEntity());

  Eigen::VectorXd nextVelocity(1);
  nextVelocity << -0.4;
  const std::vector<sx::compute::LinkContact> linkContacts{legObstacle};
  auto& registry = dart::simulation::detail::registryOf(world);
  const auto& structure = registry.get<sx::comps::MultibodyStructure>(
      dart::simulation::detail::toRegistryEntity(robot.getEntity()));
  auto linkProblem = sx::compute::assembleMultibodyLinkContactProblem(
      registry, structure, nextVelocity, 0.01, linkContacts);
  ASSERT_EQ(linkProblem.rows.size(), 1u);
  ASSERT_TRUE(linkProblem.rows[0].active);

  std::vector<sx::compute::UnifiedMultibodyContact> multibodyContacts;
  multibodyContacts.push_back(
      {dart::simulation::detail::toRegistryEntity(robot.getEntity()),
       linkProblem});
  const auto unified
      = sx::compute::assembleUnifiedConstraintProblem(rigid, multibodyContacts);

  // 3 rigid rows + 3 link rows.
  ASSERT_EQ(unified.delassus.rows(), 6);
  const auto& rigidConstraint = unified.rigidConstraints[0];
  const auto& linkRow = unified.multibodyBlocks[0].rows[0];

  // The assembled operator is symmetric (a consistent Delassus).
  EXPECT_TRUE(unified.delassus.isApprox(unified.delassus.transpose(), 1e-12));

  // The rigid-rigid block is untouched by the cross pass.
  expectMatrixExactlyEqual(
      unified.delassus.topLeftCorner(3, 3), rigid.delassus);

  // Single-source reconciliation: R is well-conditioned, so the rigid and link
  // inverse inertia agree and the reconciliation is a no-op of equal values.
  EXPECT_DOUBLE_EQ(rigidConstraint.invMassB, 1.0);
  EXPECT_DOUBLE_EQ(linkRow.otherInvMass, 1.0);
  EXPECT_TRUE(
      linkRow.otherInvInertia.isApprox(rigidConstraint.invInertiaB, 1e-12));

  // The rigid<->link cross block equals the shared-body Delassus term for R
  // over ALL nine direction pairs. R is bodyB of the rigid contact (sign +1)
  // and the link's obstacle (sign -1).
  const auto rigidDir = [&](int a) -> const Eigen::Vector3d& {
    return a == 0
               ? rigidConstraint.normal
               : (a == 1 ? rigidConstraint.tangent1 : rigidConstraint.tangent2);
  };
  const auto linkDir = [&](int b) -> const Eigen::Vector3d& {
    return b == 0 ? linkRow.normal
                  : (b == 1 ? linkRow.tangent1 : linkRow.tangent2);
  };
  for (int a = 0; a < 3; ++a) {
    for (int b = 0; b < 3; ++b) {
      const Eigen::Vector3d& dirI = rigidDir(a);
      const Eigen::Vector3d& dirJ = linkDir(b);
      const double expected = (+1.0) * (-1.0)
                              * (rigidConstraint.invMassB * dirI.dot(dirJ)
                                 + dirI.dot((rigidConstraint.invInertiaB
                                             * linkRow.otherArm.cross(dirJ))
                                                .cross(rigidConstraint.armB)));
      EXPECT_DOUBLE_EQ(unified.delassus(a, 3 + b), expected)
          << "cross (rigid dir " << a << ", link dir " << b << ")";
    }
  }
  // The normal-normal cross coupling is genuinely present.
  EXPECT_NE(unified.delassus(0, 3), 0.0);

  // The link normal diagonal now completes to the stored denominator (J M^-1 J
  // plus the obstacle self-term), which the within-domain slice left short.
  EXPECT_NEAR(unified.delassus(3, 3), linkRow.normalDenominator, 1e-12);
  EXPECT_GT(linkRow.normalDenominator, 0.0);
}

//==============================================================================
TEST(UnifiedConstraint, SolvedSolutionSatisfiesBoxedLcpConditions)
{
  // A box sliding tangentially while pressing on a static ground exercises the
  // Coulomb cone: the solved friction must respect |lambda_t| <= mu*lambda_n.
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setFriction(0.3);

  sx::RigidBodyOptions boxOptions;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.5);
  boxOptions.linearVelocity
      = Eigen::Vector3d(2.0, 0.0, -1.0); // slide + descend
  auto box = world.addRigidBody("box", boxOptions);
  box.setFriction(0.3);

  std::vector<sx::Contact> contacts;
  sx::Contact groundBox;
  groundBox.bodyA = sx::CollisionBody(ground.getEntity(), &world);
  groundBox.bodyB = sx::CollisionBody(box.getEntity(), &world);
  groundBox.point = Eigen::Vector3d(0.0, 0.0, 0.0);
  groundBox.normal = Eigen::Vector3d::UnitZ();
  groundBox.depth = 0.01;
  contacts.push_back(groundBox);

  const auto rigid = sx::compute::assembleRigidBodyContactProblem(
      dart::simulation::detail::registryOf(world), contacts);
  const std::span<const sx::compute::UnifiedMultibodyContact> noMultibodies{};
  const auto unified
      = sx::compute::assembleUnifiedConstraintProblem(rigid, noMultibodies);
  const auto solution = sx::compute::solveUnifiedConstraintProblem(unified);
  ASSERT_TRUE(solution.succeeded);
  ASSERT_EQ(solution.lambda.size(), unified.rhs.size());

  const Eigen::VectorXd w = unified.delassus * solution.lambda - unified.rhs;
  const double normalImpulse = std::max(0.0, solution.lambda[0]);
  EXPECT_GE(solution.lambda[0], -1e-9);       // unilateral normal
  EXPECT_GE(w[0], -1e-9);                     // separating (w_n >= 0)
  EXPECT_LE(solution.lambda[0] * w[0], 1e-9); // complementarity
  const double mu = unified.hi[1];            // friction coefficient
  EXPECT_LE(std::abs(solution.lambda[1]), mu * normalImpulse + 1e-9);
  EXPECT_LE(std::abs(solution.lambda[2]), mu * normalImpulse + 1e-9);
  // Friction is genuinely active (the box is sliding).
  EXPECT_GT(std::abs(solution.lambda[1]) + std::abs(solution.lambda[2]), 1e-9);
}

//==============================================================================
TEST(UnifiedConstraint, SolvesIndependentIslandsWithLocalFindex)
{
  sx::compute::UnifiedConstraintProblem problem;
  problem.delassus = Eigen::MatrixXd::Identity(6, 6);
  problem.rhs.resize(6);
  problem.rhs << 1.0, 2.0, -2.0, 2.0, -3.0, 3.0;
  problem.lo.resize(6);
  problem.lo << 0.0, -0.5, -0.5, 0.0, -0.25, -0.25;
  problem.hi.resize(6);
  problem.hi << std::numeric_limits<double>::infinity(), 0.5, 0.5,
      std::numeric_limits<double>::infinity(), 0.25, 0.25;
  problem.findex.resize(6);
  problem.findex << -1, 0, 0, -1, 3, 3;

  const auto solution = sx::compute::solveUnifiedConstraintProblem(problem);
  ASSERT_TRUE(solution.succeeded);
  ASSERT_EQ(solution.lambda.size(), 6);
  EXPECT_DOUBLE_EQ(solution.lambda[0], 1.0);
  EXPECT_DOUBLE_EQ(solution.lambda[1], 0.5);
  EXPECT_DOUBLE_EQ(solution.lambda[2], -0.5);
  EXPECT_DOUBLE_EQ(solution.lambda[3], 2.0);
  EXPECT_DOUBLE_EQ(solution.lambda[4], -0.5);
  EXPECT_DOUBLE_EQ(solution.lambda[5], 0.5);
}

//==============================================================================
TEST(UnifiedConstraint, ExecutorParallelIslandSolveMatchesSequential)
{
  sx::compute::UnifiedConstraintProblem problem;
  problem.delassus = Eigen::MatrixXd::Identity(6, 6);
  problem.rhs.resize(6);
  problem.rhs << 1.0, 2.0, -2.0, 2.0, -3.0, 3.0;
  problem.lo.resize(6);
  problem.lo << 0.0, -0.5, -0.5, 0.0, -0.25, -0.25;
  problem.hi.resize(6);
  problem.hi << std::numeric_limits<double>::infinity(), 0.5, 0.5,
      std::numeric_limits<double>::infinity(), 0.25, 0.25;
  problem.findex.resize(6);
  problem.findex << -1, 0, 0, -1, 3, 3;

  sx::compute::UnifiedConstraintSolveScratch sequentialScratch;
  ASSERT_TRUE(
      sx::compute::solveUnifiedConstraintProblemInto(
          problem, sequentialScratch));

  sx::compute::ParallelExecutor executor(2);
  sx::compute::UnifiedConstraintSolveScratch parallelScratch;
  ASSERT_TRUE(
      sx::compute::solveUnifiedConstraintProblemInto(
          problem, parallelScratch, executor));

  ASSERT_EQ(parallelScratch.lambda.size(), sequentialScratch.lambda.size());
  for (Eigen::Index i = 0; i < sequentialScratch.lambda.size(); ++i) {
    EXPECT_DOUBLE_EQ(parallelScratch.lambda[i], sequentialScratch.lambda[i]);
  }
}

//==============================================================================
TEST(UnifiedConstraint, ReusedScratchAvoidsHeapAllocationForSameShapeIslands)
{
  sx::compute::UnifiedConstraintProblem problem;
  problem.delassus = Eigen::MatrixXd::Identity(6, 6);
  problem.rhs.resize(6);
  problem.rhs << 1.0, 2.0, -2.0, 2.0, -3.0, 3.0;
  problem.lo.resize(6);
  problem.lo << 0.0, -0.5, -0.5, 0.0, -0.25, -0.25;
  problem.hi.resize(6);
  problem.hi << std::numeric_limits<double>::infinity(), 0.5, 0.5,
      std::numeric_limits<double>::infinity(), 0.25, 0.25;
  problem.findex.resize(6);
  problem.findex << -1, 0, 0, -1, 3, 3;

  sx::compute::UnifiedConstraintSolveScratch scratch;
  ASSERT_TRUE(sx::compute::solveUnifiedConstraintProblemInto(problem, scratch));
  ASSERT_TRUE(sx::compute::solveUnifiedConstraintProblemInto(problem, scratch));

  bool solved = false;
  std::size_t allocations = 0;
  std::size_t bytes = 0;
  {
    ScopedHeapAllocationCounter heapCounter;
    solved = sx::compute::solveUnifiedConstraintProblemInto(problem, scratch);
    allocations = heapCounter.allocations();
    bytes = heapCounter.bytes();
  }

  EXPECT_TRUE(solved);
  EXPECT_EQ(allocations, 0u) << "allocated " << bytes << " bytes";
  ASSERT_EQ(scratch.lambda.size(), 6);
  EXPECT_DOUBLE_EQ(scratch.lambda[0], 1.0);
  EXPECT_DOUBLE_EQ(scratch.lambda[1], 0.5);
  EXPECT_DOUBLE_EQ(scratch.lambda[2], -0.5);
  EXPECT_DOUBLE_EQ(scratch.lambda[3], 2.0);
  EXPECT_DOUBLE_EQ(scratch.lambda[4], -0.5);
  EXPECT_DOUBLE_EQ(scratch.lambda[5], 0.5);
}

//==============================================================================
TEST(UnifiedConstraint, ReusedSolutionAvoidsHeapAllocationForSameShapeIslands)
{
  sx::compute::UnifiedConstraintProblem problem;
  problem.delassus = Eigen::MatrixXd::Identity(6, 6);
  problem.rhs.resize(6);
  problem.rhs << 1.0, 2.0, -2.0, 2.0, -3.0, 3.0;
  problem.lo.resize(6);
  problem.lo << 0.0, -0.5, -0.5, 0.0, -0.25, -0.25;
  problem.hi.resize(6);
  problem.hi << std::numeric_limits<double>::infinity(), 0.5, 0.5,
      std::numeric_limits<double>::infinity(), 0.25, 0.25;
  problem.findex.resize(6);
  problem.findex << -1, 0, 0, -1, 3, 3;

  sx::compute::UnifiedConstraintSolveScratch scratch;
  sx::compute::UnifiedConstraintSolution solution;
  ASSERT_TRUE(
      sx::compute::solveUnifiedConstraintProblemInto(
          problem, solution, scratch));
  ASSERT_TRUE(
      sx::compute::solveUnifiedConstraintProblemInto(
          problem, solution, scratch));

  bool solved = false;
  std::size_t allocations = 0;
  std::size_t bytes = 0;
  {
    ScopedHeapAllocationCounter heapCounter;
    solved = sx::compute::solveUnifiedConstraintProblemInto(
        problem, solution, scratch);
    allocations = heapCounter.allocations();
    bytes = heapCounter.bytes();
  }

  EXPECT_TRUE(solved);
  EXPECT_TRUE(solution.succeeded);
  EXPECT_EQ(allocations, 0u) << "allocated " << bytes << " bytes";
  ASSERT_EQ(solution.lambda.size(), 6);
  EXPECT_DOUBLE_EQ(solution.lambda[0], 1.0);
  EXPECT_DOUBLE_EQ(solution.lambda[1], 0.5);
  EXPECT_DOUBLE_EQ(solution.lambda[2], -0.5);
  EXPECT_DOUBLE_EQ(solution.lambda[3], 2.0);
  EXPECT_DOUBLE_EQ(solution.lambda[4], -0.5);
  EXPECT_DOUBLE_EQ(solution.lambda[5], 0.5);
}

//==============================================================================
TEST(
    UnifiedConstraint,
    ReusedScratchAvoidsHeapAllocationWhenApplyingLinkImpulses)
{
  sx::World world;

  sx::compute::UnifiedConstraintProblem problem;
  problem.multibodyBlocks.resize(2);

  auto& block = problem.multibodyBlocks[0];
  block.blockBase = 0;
  block.inverseMass.resize(2, 2);
  block.inverseMass << 2.0, 0.25, 0.25, 1.5;
  block.rows.resize(1);
  auto& row = block.rows[0];
  row.normalJacobian.resize(2);
  row.tangentJacobian1.resize(2);
  row.tangentJacobian2.resize(2);
  row.otherNormalJacobian.resize(2);
  row.otherTangentJacobian1.resize(2);
  row.otherTangentJacobian2.resize(2);
  row.normalJacobian << 1.0, 0.5;
  row.tangentJacobian1 << 0.25, 0.75;
  row.tangentJacobian2 << -0.5, 0.125;
  row.otherNormalJacobian << 0.5, -0.25;
  row.otherTangentJacobian1 << -0.125, 0.625;
  row.otherTangentJacobian2 << 0.375, 0.5;
  row.otherMultibodyIndex = 1;

  auto& otherBlock = problem.multibodyBlocks[1];
  otherBlock.blockBase = 3;
  otherBlock.inverseMass.resize(2, 2);
  otherBlock.inverseMass << 1.25, 0.1, 0.1, 1.75;

  Eigen::VectorXd lambda(3);
  lambda << 0.8, -0.2, 0.125;

  std::vector<Eigen::VectorXd> multibodyVelocities(2);
  multibodyVelocities[0].resize(2);
  multibodyVelocities[0] << -0.5, 0.25;
  multibodyVelocities[1].resize(2);
  multibodyVelocities[1] << 0.125, -0.75;

  sx::compute::UnifiedConstraintSolveScratch scratch;
  sx::compute::applyUnifiedConstraintImpulses(
      dart::simulation::detail::registryOf(world),
      problem,
      lambda,
      std::span<Eigen::VectorXd>(multibodyVelocities),
      scratch);
  sx::compute::applyUnifiedConstraintImpulses(
      dart::simulation::detail::registryOf(world),
      problem,
      lambda,
      std::span<Eigen::VectorXd>(multibodyVelocities),
      scratch);

  const Eigen::VectorXd beforePrimary = multibodyVelocities[0];
  const Eigen::VectorXd beforeOther = multibodyVelocities[1];
  std::size_t allocations = 0;
  std::size_t bytes = 0;
  {
    ScopedHeapAllocationCounter heapCounter;
    sx::compute::applyUnifiedConstraintImpulses(
        dart::simulation::detail::registryOf(world),
        problem,
        lambda,
        std::span<Eigen::VectorXd>(multibodyVelocities),
        scratch);
    allocations = heapCounter.allocations();
    bytes = heapCounter.bytes();
  }

  EXPECT_EQ(allocations, 0u) << "allocated " << bytes << " bytes";

  const Eigen::VectorXd primaryImpulse = lambda[0] * row.normalJacobian
                                         + lambda[1] * row.tangentJacobian1
                                         + lambda[2] * row.tangentJacobian2;
  const Eigen::VectorXd otherImpulse = lambda[0] * row.otherNormalJacobian
                                       + lambda[1] * row.otherTangentJacobian1
                                       + lambda[2] * row.otherTangentJacobian2;
  const Eigen::VectorXd expectedPrimary
      = beforePrimary + block.inverseMass * primaryImpulse;
  const Eigen::VectorXd expectedOther
      = beforeOther - otherBlock.inverseMass * otherImpulse;
  EXPECT_TRUE(multibodyVelocities[0].isApprox(expectedPrimary, 1e-12));
  EXPECT_TRUE(multibodyVelocities[1].isApprox(expectedOther, 1e-12));
}

//==============================================================================
TEST(UnifiedConstraint, SolveAndApplyStopsHeadOnRigidContact)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions leftOptions;
  leftOptions.position = Eigen::Vector3d(-0.5, 0.0, 0.0);
  leftOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto left = world.addRigidBody("left", leftOptions);

  sx::RigidBodyOptions rightOptions;
  rightOptions.position = Eigen::Vector3d(0.5, 0.0, 0.0);
  rightOptions.linearVelocity = Eigen::Vector3d(-1.0, 0.0, 0.0);
  auto right = world.addRigidBody("right", rightOptions);

  std::vector<sx::Contact> contacts;
  sx::Contact contact;
  contact.bodyA = sx::CollisionBody(left.getEntity(), &world);
  contact.bodyB = sx::CollisionBody(right.getEntity(), &world);
  contact.point = Eigen::Vector3d::Zero();
  contact.normal = Eigen::Vector3d::UnitX(); // points left -> right
  contact.depth = 0.0;
  contacts.push_back(contact);

  const auto rigid = sx::compute::assembleRigidBodyContactProblem(
      dart::simulation::detail::registryOf(world), contacts);
  const std::span<const sx::compute::UnifiedMultibodyContact> noMultibodies{};
  const auto unified
      = sx::compute::assembleUnifiedConstraintProblem(rigid, noMultibodies);
  const auto solution = sx::compute::solveUnifiedConstraintProblem(unified);
  ASSERT_TRUE(solution.succeeded);

  std::vector<Eigen::VectorXd> noMultibodyVelocities;
  sx::compute::applyUnifiedConstraintImpulses(
      dart::simulation::detail::registryOf(world),
      unified,
      solution.lambda,
      std::span<Eigen::VectorXd>(noMultibodyVelocities));

  auto& registry = dart::simulation::detail::registryOf(world);
  const Eigen::Vector3d velocityLeft
      = registry
            .get<sx::comps::Velocity>(
                dart::simulation::detail::toRegistryEntity(left.getEntity()))
            .linear;
  const Eigen::Vector3d velocityRight
      = registry
            .get<sx::comps::Velocity>(
                dart::simulation::detail::toRegistryEntity(right.getEntity()))
            .linear;
  // No restitution: the approaching normal velocity is driven to zero.
  const double relativeNormal
      = (velocityRight - velocityLeft).dot(Eigen::Vector3d::UnitX());
  EXPECT_NEAR(relativeNormal, 0.0, 1e-9);
}

//==============================================================================
TEST(UnifiedConstraint, SolveAndApplyDrivesLinkContactToTarget)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  auto robot = world.addMultibody("robot");
  auto baseLink = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "slider";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto leg = robot.addLink("leg", baseLink, spec);
  leg.setMass(1.0);

  sx::compute::LinkContact contact;
  contact.link = dart::simulation::detail::toRegistryEntity(leg.getEntity());
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.point = Eigen::Vector3d::Zero();
  contact.depth = 0.02; // penetration -> Baumgarte push-out target
  contact.friction = 0.5;

  Eigen::VectorXd nextVelocity(1);
  nextVelocity << -0.5;
  const double timeStep = 0.01;
  const std::vector<sx::compute::LinkContact> linkContacts{contact};
  auto& registry = dart::simulation::detail::registryOf(world);
  const auto& structure = registry.get<sx::comps::MultibodyStructure>(
      dart::simulation::detail::toRegistryEntity(robot.getEntity()));
  auto linkProblem = sx::compute::assembleMultibodyLinkContactProblem(
      registry, structure, nextVelocity, timeStep, linkContacts);

  std::vector<sx::compute::UnifiedMultibodyContact> multibodyContacts;
  multibodyContacts.push_back(
      {dart::simulation::detail::toRegistryEntity(robot.getEntity()),
       linkProblem});
  sx::compute::RigidBodyContactProblem emptyRigid;
  const auto unified = sx::compute::assembleUnifiedConstraintProblem(
      emptyRigid, multibodyContacts);
  const auto solution = sx::compute::solveUnifiedConstraintProblem(unified);
  ASSERT_TRUE(solution.succeeded);

  std::vector<Eigen::VectorXd> multibodyVelocities{nextVelocity};
  sx::compute::applyUnifiedConstraintImpulses(
      dart::simulation::detail::registryOf(world),
      unified,
      solution.lambda,
      std::span<Eigen::VectorXd>(multibodyVelocities));

  // One-sided link contact: the post-solve normal velocity reaches the LCP
  // target max(bias, restitutionTarget).
  const auto& row = unified.multibodyBlocks[0].rows[0];
  const double normalVelocity = row.normalJacobian.dot(multibodyVelocities[0]);
  EXPECT_NEAR(normalVelocity, std::max(row.bias, row.restitutionTarget), 1e-9);
  EXPECT_GT(row.bias, 0.0); // the contact is penetrating
}

//==============================================================================
TEST(UnifiedConstraint, SolveAndApplyDeliversNewtonImpulseToObstacle)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  auto robot = world.addMultibody("robot");
  auto baseLink = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "slider";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto leg = robot.addLink("leg", baseLink, spec);
  leg.setMass(1.0);

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.mass = 1.0;
  obstacleOptions.inertia = Eigen::Matrix3d::Identity();
  obstacleOptions.position = Eigen::Vector3d(0.0, 0.0, 0.5);
  auto obstacle = world.addRigidBody("obstacle", obstacleOptions);

  sx::compute::LinkContact contact;
  contact.link = dart::simulation::detail::toRegistryEntity(leg.getEntity());
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.point = Eigen::Vector3d(0.1, 0.0, 0.5); // offset -> nonzero arm
  contact.depth = 0.0;
  contact.friction = 0.5;
  contact.otherBody
      = dart::simulation::detail::toRegistryEntity(obstacle.getEntity());

  // The normal points into the link, so a negative normal velocity is the leg
  // approaching the obstacle -> an active contact with a positive impulse.
  Eigen::VectorXd nextVelocity(1);
  nextVelocity << -1.0;
  const std::vector<sx::compute::LinkContact> linkContacts{contact};
  auto& registry = dart::simulation::detail::registryOf(world);
  const auto& structure = registry.get<sx::comps::MultibodyStructure>(
      dart::simulation::detail::toRegistryEntity(robot.getEntity()));
  auto linkProblem = sx::compute::assembleMultibodyLinkContactProblem(
      registry, structure, nextVelocity, 0.01, linkContacts);

  std::vector<sx::compute::UnifiedMultibodyContact> multibodyContacts;
  multibodyContacts.push_back(
      {dart::simulation::detail::toRegistryEntity(robot.getEntity()),
       linkProblem});
  sx::compute::RigidBodyContactProblem emptyRigid;
  const auto unified = sx::compute::assembleUnifiedConstraintProblem(
      emptyRigid, multibodyContacts);
  const auto solution = sx::compute::solveUnifiedConstraintProblem(unified);
  ASSERT_TRUE(solution.succeeded);

  std::vector<Eigen::VectorXd> multibodyVelocities{nextVelocity};
  sx::compute::applyUnifiedConstraintImpulses(
      dart::simulation::detail::registryOf(world),
      unified,
      solution.lambda,
      std::span<Eigen::VectorXd>(multibodyVelocities));

  // The obstacle receives the exact equal-and-opposite (Newton) impulse.
  const auto& row = unified.multibodyBlocks[0].rows[0];
  const double normalImpulse = std::max(0.0, solution.lambda[0]);
  const double tangentImpulse1 = solution.lambda[1];
  const double tangentImpulse2 = solution.lambda[2];
  ASSERT_GT(normalImpulse, 0.0); // the contact is active

  const Eigen::Vector3d expectedLinear
      = -(normalImpulse * row.otherInvMass * row.normal
          + tangentImpulse1 * row.otherInvMass * row.tangent1
          + tangentImpulse2 * row.otherInvMass * row.tangent2);
  const Eigen::Vector3d expectedAngular = -(
      normalImpulse * row.otherInvInertia * row.otherArm.cross(row.normal)
      + tangentImpulse1 * row.otherInvInertia * row.otherArm.cross(row.tangent1)
      + tangentImpulse2 * row.otherInvInertia
            * row.otherArm.cross(row.tangent2));
  const auto& obstacleVelocity = registry.get<sx::comps::Velocity>(
      dart::simulation::detail::toRegistryEntity(obstacle.getEntity()));
  EXPECT_TRUE(obstacleVelocity.linear.isApprox(expectedLinear, 1e-12));
  EXPECT_TRUE(obstacleVelocity.angular.isApprox(expectedAngular, 1e-12));
}

//==============================================================================
TEST(UnifiedConstraint, FallbackStopsHeadOnRigidContact)
{
  // The fallback path (normal-only LCP + friction sweep) resolves a simple
  // head-on contact identically to the joint solve.
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions leftOptions;
  leftOptions.position = Eigen::Vector3d(-0.5, 0.0, 0.0);
  leftOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto left = world.addRigidBody("left", leftOptions);

  sx::RigidBodyOptions rightOptions;
  rightOptions.position = Eigen::Vector3d(0.5, 0.0, 0.0);
  rightOptions.linearVelocity = Eigen::Vector3d(-1.0, 0.0, 0.0);
  auto right = world.addRigidBody("right", rightOptions);

  std::vector<sx::Contact> contacts;
  sx::Contact contact;
  contact.bodyA = sx::CollisionBody(left.getEntity(), &world);
  contact.bodyB = sx::CollisionBody(right.getEntity(), &world);
  contact.point = Eigen::Vector3d::Zero();
  contact.normal = Eigen::Vector3d::UnitX();
  contact.depth = 0.0;
  contacts.push_back(contact);

  const auto rigid = sx::compute::assembleRigidBodyContactProblem(
      dart::simulation::detail::registryOf(world), contacts);
  const std::span<const sx::compute::UnifiedMultibodyContact> noMultibodies{};
  const auto unified
      = sx::compute::assembleUnifiedConstraintProblem(rigid, noMultibodies);

  std::vector<Eigen::VectorXd> noMultibodyVelocities;
  sx::compute::applyUnifiedConstraintFallback(
      dart::simulation::detail::registryOf(world),
      unified,
      std::span<Eigen::VectorXd>(noMultibodyVelocities),
      8);

  auto& registry = dart::simulation::detail::registryOf(world);
  const double relativeNormal
      = (registry
             .get<sx::comps::Velocity>(
                 dart::simulation::detail::toRegistryEntity(right.getEntity()))
             .linear
         - registry
               .get<sx::comps::Velocity>(
                   dart::simulation::detail::toRegistryEntity(left.getEntity()))
               .linear)
            .dot(Eigen::Vector3d::UnitX());
  EXPECT_NEAR(relativeNormal, 0.0, 1e-9);
}

//==============================================================================
TEST(UnifiedConstraint, FallbackFrictionOpposesSlidingWithoutReversing)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setFriction(0.4);

  sx::RigidBodyOptions boxOptions;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.5);
  boxOptions.linearVelocity = Eigen::Vector3d(2.0, 0.0, -1.0);
  auto box = world.addRigidBody("box", boxOptions);
  box.setFriction(0.4);

  std::vector<sx::Contact> contacts;
  sx::Contact contact;
  contact.bodyA = sx::CollisionBody(ground.getEntity(), &world);
  contact.bodyB = sx::CollisionBody(box.getEntity(), &world);
  contact.point = Eigen::Vector3d::Zero();
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.depth = 0.01;
  contacts.push_back(contact);

  const auto rigid = sx::compute::assembleRigidBodyContactProblem(
      dart::simulation::detail::registryOf(world), contacts);
  const std::span<const sx::compute::UnifiedMultibodyContact> noMultibodies{};
  const auto unified
      = sx::compute::assembleUnifiedConstraintProblem(rigid, noMultibodies);

  std::vector<Eigen::VectorXd> noMultibodyVelocities;
  sx::compute::applyUnifiedConstraintFallback(
      dart::simulation::detail::registryOf(world),
      unified,
      std::span<Eigen::VectorXd>(noMultibodyVelocities),
      16);

  const Eigen::Vector3d boxVelocity
      = dart::simulation::detail::registryOf(world)
            .get<sx::comps::Velocity>(
                dart::simulation::detail::toRegistryEntity(box.getEntity()))
            .linear;
  // The normal contact arrests the descent and friction opposes (but does not
  // reverse) the tangential slide.
  EXPECT_GE(boxVelocity.z(), -1e-9);
  EXPECT_GT(boxVelocity.x(), 0.0); // still sliding in the same direction
  EXPECT_LT(boxVelocity.x(), 2.0); // but decelerated by friction

  sx::compute::UnifiedConstraintSolveScratch scratch;
  sx::compute::applyUnifiedConstraintFallback(
      dart::simulation::detail::registryOf(world),
      unified,
      std::span<Eigen::VectorXd>(noMultibodyVelocities),
      16,
      scratch);
  sx::compute::applyUnifiedConstraintFallback(
      dart::simulation::detail::registryOf(world),
      unified,
      std::span<Eigen::VectorXd>(noMultibodyVelocities),
      16,
      scratch);

  std::size_t allocations = 0;
  std::size_t bytes = 0;
  {
    ScopedHeapAllocationCounter heapCounter;
    sx::compute::applyUnifiedConstraintFallback(
        dart::simulation::detail::registryOf(world),
        unified,
        std::span<Eigen::VectorXd>(noMultibodyVelocities),
        16,
        scratch);
    allocations = heapCounter.allocations();
    bytes = heapCounter.bytes();
  }

  EXPECT_EQ(allocations, 0u) << "allocated " << bytes << " bytes";
}

//==============================================================================
TEST(UnifiedConstraint, ResolveUsesJointSolveWhenWellPosed)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions leftOptions;
  leftOptions.position = Eigen::Vector3d(-0.5, 0.0, 0.0);
  leftOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto left = world.addRigidBody("left", leftOptions);
  sx::RigidBodyOptions rightOptions;
  rightOptions.position = Eigen::Vector3d(0.5, 0.0, 0.0);
  rightOptions.linearVelocity = Eigen::Vector3d(-1.0, 0.0, 0.0);
  auto right = world.addRigidBody("right", rightOptions);

  std::vector<sx::Contact> contacts;
  sx::Contact contact;
  contact.bodyA = sx::CollisionBody(left.getEntity(), &world);
  contact.bodyB = sx::CollisionBody(right.getEntity(), &world);
  contact.point = Eigen::Vector3d::Zero();
  contact.normal = Eigen::Vector3d::UnitX();
  contact.depth = 0.0;
  contacts.push_back(contact);

  const auto rigid = sx::compute::assembleRigidBodyContactProblem(
      dart::simulation::detail::registryOf(world), contacts);
  const std::span<const sx::compute::UnifiedMultibodyContact> noMultibodies{};
  const auto unified
      = sx::compute::assembleUnifiedConstraintProblem(rigid, noMultibodies);

  std::vector<Eigen::VectorXd> noMultibodyVelocities;
  const bool jointSolved = sx::compute::resolveUnifiedConstraints(
      dart::simulation::detail::registryOf(world),
      unified,
      std::span<Eigen::VectorXd>(noMultibodyVelocities),
      8);
  EXPECT_TRUE(jointSolved); // a single contact is full-rank

  auto& registry = dart::simulation::detail::registryOf(world);
  const double relativeNormal
      = (registry
             .get<sx::comps::Velocity>(
                 dart::simulation::detail::toRegistryEntity(right.getEntity()))
             .linear
         - registry
               .get<sx::comps::Velocity>(
                   dart::simulation::detail::toRegistryEntity(left.getEntity()))
               .linear)
            .dot(Eigen::Vector3d::UnitX());
  EXPECT_NEAR(relativeNormal, 0.0, 1e-9);
}

//==============================================================================
TEST(UnifiedConstraint, FallbackResolvesCoplanarBoxOnPlane)
{
  // Four coplanar contacts of a box on a plane are the canonical rank-deficient
  // set the fallback exists for. Exercising the fallback directly (the coupled
  // normal-only LCP + diagonal-projection last resort + friction sweep) must
  // still arrest the box's descent.
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);

  sx::RigidBodyOptions boxOptions;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.5);
  boxOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -2.0);
  auto box = world.addRigidBody("box", boxOptions);

  std::vector<sx::Contact> contacts;
  const double corners[4][2]
      = {{0.5, 0.5}, {0.5, -0.5}, {-0.5, 0.5}, {-0.5, -0.5}};
  for (const auto& corner : corners) {
    sx::Contact contact;
    contact.bodyA = sx::CollisionBody(ground.getEntity(), &world);
    contact.bodyB = sx::CollisionBody(box.getEntity(), &world);
    contact.point = Eigen::Vector3d(corner[0], corner[1], 0.0);
    contact.normal = Eigen::Vector3d::UnitZ();
    contact.depth = 0.01;
    contacts.push_back(contact);
  }

  const auto rigid = sx::compute::assembleRigidBodyContactProblem(
      dart::simulation::detail::registryOf(world), contacts);
  ASSERT_EQ(rigid.constraints.size(), 4u);
  const std::span<const sx::compute::UnifiedMultibodyContact> noMultibodies{};
  const auto unified
      = sx::compute::assembleUnifiedConstraintProblem(rigid, noMultibodies);

  std::vector<Eigen::VectorXd> noMultibodyVelocities;
  sx::compute::applyUnifiedConstraintFallback(
      dart::simulation::detail::registryOf(world),
      unified,
      std::span<Eigen::VectorXd>(noMultibodyVelocities),
      8);

  // The fallback arrests the box's descent and never injects upward velocity.
  const double boxVerticalVelocity
      = dart::simulation::detail::registryOf(world)
            .get<sx::comps::Velocity>(
                dart::simulation::detail::toRegistryEntity(box.getEntity()))
            .linear.z();
  EXPECT_NEAR(boxVerticalVelocity, 0.0, 1e-6);
}
