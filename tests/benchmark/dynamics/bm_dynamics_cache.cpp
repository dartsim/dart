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

/// @file bm_dynamics_cache.cpp
/// @brief Comprehensive dynamics benchmark suite for DART.
///
/// Benchmarks are parametric across:
///   - Scale: 1 to 500 bodies
///   - Topology: serial chain, binary tree, star
///   - Joint type: Revolute, Ball, Free
///   - Operation: FK, FD, ID, mass matrix, Coriolis+gravity, integration,
///                World::step
///   - Multi-skeleton: 1 to 100 skeletons in a world
///   - Parallel worlds: 1 to 4 threads
///
/// Run all:     pixi run bm -- dynamics_cache
/// Run subset:  pixi run bm -- dynamics_cache
///                --benchmark_filter="BM_Serial.*Revolute.*ForwardDynamics"

#include <dart/simulation/world.hpp>

#include <dart/dynamics/ball_joint.hpp>
#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/random.hpp>

#include <benchmark/benchmark.h>

#include <atomic>
#include <queue>
#include <thread>
#include <vector>

#include <cassert>
#include <cmath>
#include <cstdlib>

// ============================================================================
// Global allocation tracking for Section P (Allocation Metrics)
// ============================================================================
// These overrides are active for the entire benchmark binary. They add minimal
// overhead (atomic increment) which is constant across baseline and optimized
// runs, so relative comparisons remain valid.
static std::atomic<int64_t> g_alloc_count{0};
static std::atomic<int64_t> g_alloc_bytes{0};

void* operator new(std::size_t size)
{
  g_alloc_count.fetch_add(1, std::memory_order_relaxed);
  g_alloc_bytes.fetch_add(
      static_cast<int64_t>(size), std::memory_order_relaxed);
  void* ptr = std::malloc(size);
  if (!ptr) {
    throw std::bad_alloc();
  }
  return ptr;
}

void* operator new[](std::size_t size)
{
  g_alloc_count.fetch_add(1, std::memory_order_relaxed);
  g_alloc_bytes.fetch_add(
      static_cast<int64_t>(size), std::memory_order_relaxed);
  void* ptr = std::malloc(size);
  if (!ptr) {
    throw std::bad_alloc();
  }
  return ptr;
}

void operator delete(void* ptr) noexcept
{
  std::free(ptr);
}

void operator delete[](void* ptr) noexcept
{
  std::free(ptr);
}

void operator delete(void* ptr, std::size_t) noexcept
{
  std::free(ptr);
}

void operator delete[](void* ptr, std::size_t) noexcept
{
  std::free(ptr);
}

using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;

// ============================================================================
// Section A: Skeleton Builder Helpers
// ============================================================================

/// Joint property helper: set transform offset so links are spaced apart.
template <typename JointType>
void setJointOffset(
    typename JointType::Properties& /*props*/, [[maybe_unused]] int /*index*/)
{
  // Default: no special offset setup needed (Ball/Free joints don't use axis)
}

template <>
void setJointOffset<RevoluteJoint>(
    RevoluteJoint::Properties& props, int /*index*/)
{
  props.mAxis = Eigen::Vector3d::UnitZ();
}

/// Create body properties with uniform mass and inertia.
inline BodyNode::Properties makeBodyProps(const std::string& name)
{
  BodyNode::Properties props;
  props.mName = name;
  props.mInertia.setMass(1.0);
  props.mInertia.setMoment(0.1 * Eigen::Matrix3d::Identity());
  return props;
}

/// Create body properties with specified mass and inertia scale.
inline BodyNode::Properties makeBodyProps(
    const std::string& name, double mass, double inertiaScale)
{
  BodyNode::Properties props;
  props.mName = name;
  props.mInertia.setMass(mass);
  props.mInertia.setMoment(inertiaScale * Eigen::Matrix3d::Identity());
  return props;
}

/// Build a serial chain (linear: body[i] is parent of body[i+1]).
template <typename JointType>
SkeletonPtr makeSerialChain(int N, const std::string& prefix = "serial")
{
  auto skel = Skeleton::create(prefix + "_" + std::to_string(N));
  BodyNode* parent = nullptr;
  for (int i = 0; i < N; ++i) {
    typename JointType::Properties jprops;
    jprops.mName = prefix + "_j" + std::to_string(i);
    setJointOffset<JointType>(jprops, i);

    auto bprops = makeBodyProps(prefix + "_b" + std::to_string(i));
    auto [joint, body]
        = skel->createJointAndBodyNodePair<JointType>(parent, jprops, bprops);

    // Space links apart so geometry is non-degenerate
    joint->setTransformFromParentBodyNode(
        Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.1)));

    parent = body;
  }
  return skel;
}

/// Build a binary tree (breadth-first). Total bodies >= N.
template <typename JointType>
SkeletonPtr makeBinaryTree(int N, const std::string& prefix = "btree")
{
  auto skel = Skeleton::create(prefix + "_" + std::to_string(N));

  if (N <= 0) {
    return skel;
  }

  // Create root
  typename JointType::Properties jprops;
  jprops.mName = prefix + "_j0";
  setJointOffset<JointType>(jprops, 0);
  auto bprops = makeBodyProps(prefix + "_b0");
  auto [rootJoint, rootBody]
      = skel->createJointAndBodyNodePair<JointType>(nullptr, jprops, bprops);
  rootJoint->setTransformFromParentBodyNode(
      Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.1)));

  int count = 1;
  std::queue<BodyNode*> frontier;
  frontier.push(rootBody);

  while (count < N && !frontier.empty()) {
    BodyNode* parent = frontier.front();
    frontier.pop();

    // Create left child
    if (count >= N) {
      break;
    }
    typename JointType::Properties ljprops;
    ljprops.mName = prefix + "_j" + std::to_string(count);
    setJointOffset<JointType>(ljprops, count);
    auto lbprops = makeBodyProps(prefix + "_b" + std::to_string(count));
    auto [lj, lb]
        = skel->createJointAndBodyNodePair<JointType>(parent, ljprops, lbprops);
    lj->setTransformFromParentBodyNode(
        Eigen::Isometry3d(Eigen::Translation3d(0.05, 0.0, 0.1)));
    frontier.push(lb);
    ++count;

    // Create right child
    if (count >= N) {
      break;
    }
    typename JointType::Properties rjprops;
    rjprops.mName = prefix + "_j" + std::to_string(count);
    setJointOffset<JointType>(rjprops, count);
    auto rbprops = makeBodyProps(prefix + "_b" + std::to_string(count));
    auto [rj, rb]
        = skel->createJointAndBodyNodePair<JointType>(parent, rjprops, rbprops);
    rj->setTransformFromParentBodyNode(
        Eigen::Isometry3d(Eigen::Translation3d(-0.05, 0.0, 0.1)));
    frontier.push(rb);
    ++count;
  }
  return skel;
}

/// Build a star topology (single root, N-1 children directly on root).
template <typename JointType>
SkeletonPtr makeStar(int N, const std::string& prefix = "star")
{
  auto skel = Skeleton::create(prefix + "_" + std::to_string(N));

  if (N <= 0) {
    return skel;
  }

  // Create root
  typename JointType::Properties jprops;
  jprops.mName = prefix + "_j0";
  setJointOffset<JointType>(jprops, 0);
  auto bprops = makeBodyProps(prefix + "_b0");
  auto [rootJoint, rootBody]
      = skel->createJointAndBodyNodePair<JointType>(nullptr, jprops, bprops);
  rootJoint->setTransformFromParentBodyNode(
      Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.1)));

  // All remaining bodies are children of root
  for (int i = 1; i < N; ++i) {
    typename JointType::Properties cjprops;
    cjprops.mName = prefix + "_j" + std::to_string(i);
    setJointOffset<JointType>(cjprops, i);
    auto cbprops = makeBodyProps(prefix + "_b" + std::to_string(i));
    auto [cj, cb] = skel->createJointAndBodyNodePair<JointType>(
        rootBody, cjprops, cbprops);

    const double angle = 2.0 * M_PI * i / (N - 1);
    cj->setTransformFromParentBodyNode(
        Eigen::Isometry3d(
            Eigen::Translation3d(
                0.1 * std::cos(angle), 0.1 * std::sin(angle), 0.0)));
  }
  return skel;
}

/// Build a Unitree G1-like humanoid skeleton with 29 DOF.
///
/// Topology:
///   - FreeJoint root "pelvis" (6 DOF)
///   - Torso chain from pelvis: waist_yaw, waist_pitch, waist_roll (3 DOF)
///   - Left arm from torso: 7 revolute joints (7 DOF)
///   - Right arm from torso: 7 revolute joints (7 DOF)
///   - Left leg from pelvis: 3 revolute joints (3 DOF)
///   - Right leg from pelvis: 3 revolute joints (3 DOF)
///   - Total: 6 + 3 + 7 + 7 + 3 + 3 = 29 DOF
inline SkeletonPtr makeG1Humanoid()
{
  auto skel = Skeleton::create("g1_humanoid_29dof");
  constexpr double kMass = 1.5;
  constexpr double kInertia = 0.15;
  const Eigen::Isometry3d offset(Eigen::Translation3d(0.0, 0.0, 0.1));

  // Helper: append a chain of revolute joints to a parent body
  auto addRevoluteChain
      = [&](BodyNode* parent,
            const std::vector<std::string>& jointNames) -> BodyNode* {
    BodyNode* current = parent;
    for (const auto& jname : jointNames) {
      RevoluteJoint::Properties jprops;
      jprops.mName = jname;
      setJointOffset<RevoluteJoint>(jprops, 0);
      auto bprops = makeBodyProps(jname + "_body", kMass, kInertia);
      auto [joint, body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
          current, jprops, bprops);
      joint->setTransformFromParentBodyNode(offset);
      current = body;
    }
    return current;
  };

  // Root: FreeJoint pelvis (6 DOF)
  FreeJoint::Properties pelvisJProps;
  pelvisJProps.mName = "pelvis_joint";
  auto pelvisProps = makeBodyProps("pelvis", kMass, kInertia);
  auto [pelvisJoint, pelvis] = skel->createJointAndBodyNodePair<FreeJoint>(
      nullptr, pelvisJProps, pelvisProps);
  pelvisJoint->setTransformFromParentBodyNode(offset);

  // Torso chain from pelvis (3 DOF)
  BodyNode* torso
      = addRevoluteChain(pelvis, {"waist_yaw", "waist_pitch", "waist_roll"});

  // Left arm from torso (7 DOF)
  addRevoluteChain(
      torso,
      {"l_shoulder_pitch",
       "l_shoulder_roll",
       "l_shoulder_yaw",
       "l_elbow",
       "l_wrist_roll",
       "l_wrist_pitch",
       "l_wrist_yaw"});

  // Right arm from torso (7 DOF)
  addRevoluteChain(
      torso,
      {"r_shoulder_pitch",
       "r_shoulder_roll",
       "r_shoulder_yaw",
       "r_elbow",
       "r_wrist_roll",
       "r_wrist_pitch",
       "r_wrist_yaw"});

  // Left leg from pelvis (3 DOF)
  addRevoluteChain(pelvis, {"l_hip", "l_knee", "l_ankle"});

  // Right leg from pelvis (3 DOF)
  addRevoluteChain(pelvis, {"r_hip", "r_knee", "r_ankle"});

  return skel;
}

/// Randomize all DOF positions and velocities to defeat caching.
inline void randomizeState(const SkeletonPtr& skel)
{
  for (std::size_t i = 0; i < skel->getNumDofs(); ++i) {
    auto* dof = skel->getDof(i);
    dof->setPosition(math::Random::uniform(-0.5, 0.5));
    dof->setVelocity(math::Random::uniform(-1.0, 1.0));
  }
}

/// Force all joints to mark their caches as dirty.
inline void dirtyAllJoints(const SkeletonPtr& skel)
{
  for (std::size_t i = 0; i < skel->getNumJoints(); ++i) {
    skel->getJoint(i)->notifyPositionUpdated();
  }
}

/// Lightweight energy conservation sanity check.
/// Uses zero-gravity world so total KE should be roughly conserved.
/// Returns true if energy drift is within tolerance.
inline bool checkEnergyConservation(
    const SkeletonPtr& skel, int steps = 100, double toleranceFraction = 0.5)
{
  auto world = World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(0.001);
  world->addSkeleton(skel->cloneSkeleton());

  auto s = world->getSkeleton(0);
  // Set some initial velocity
  for (std::size_t i = 0; i < s->getNumDofs(); ++i) {
    s->getDof(i)->setVelocity(0.1);
  }

  const double e0 = s->computeKineticEnergy() + s->computePotentialEnergy();
  for (int i = 0; i < steps; ++i) {
    world->step();
  }
  const double e1 = s->computeKineticEnergy() + s->computePotentialEnergy();

  if (e0 == 0.0) {
    return true;
  }
  return std::abs(e1 - e0) / std::abs(e0) < toleranceFraction;
}

// ============================================================================
// Section B: Per-Operation Benchmark Templates
// ============================================================================

// Macro to define and register a benchmark for a single
// (topology, joint type, operation) combination.
//
// TopologyTag  — name fragment: Serial, BinaryTree, Star
// MakerFunc    — makeSerialChain, makeBinaryTree, makeStar
// JointType    — RevoluteJoint, BallJoint, FreeJoint
// JointTag     — Revolute, Ball, Free
// OpTag        — ForwardKinematics, ForwardDynamics, etc.
// OpBody       — the C++ statement(s) to benchmark

// clang-format off
#define DEFINE_DYNAMICS_BM(TopologyTag, MakerFunc, JointType, JointTag, OpTag, OpBody) \
  static void BM_##TopologyTag##_##JointTag##_##OpTag(                                 \
      benchmark::State& state)                                                          \
  {                                                                                     \
    const int N = static_cast<int>(state.range(0));                                     \
    auto skel = MakerFunc<JointType>(N);                                                \
    randomizeState(skel);                                                               \
                                                                                        \
    for (auto _ : state) {                                                              \
      state.PauseTiming();                                                              \
      dirtyAllJoints(skel);                                                             \
      randomizeState(skel);                                                             \
      state.ResumeTiming();                                                             \
                                                                                        \
      OpBody;                                                                           \
    }                                                                                   \
    state.SetItemsProcessed(                                                            \
        static_cast<int64_t>(state.iterations()));                                      \
    state.counters["bodies"]                                                            \
        = static_cast<double>(skel->getNumBodyNodes());                                 \
    state.counters["dofs"]                                                              \
        = static_cast<double>(skel->getNumDofs());                                      \
  }
// clang-format on

// Scale arguments for body count
// Small: 1,5,10,20  Medium: 50,100  Large: 200,500
#define BM_SCALE_ARGS                                                          \
  Arg(1)->Arg(5)->Arg(10)->Arg(20)->Arg(50)->Arg(100)->Arg(200)

// Full scale including 500 (some combos like Free/500 are very expensive)
#define BM_SCALE_ARGS_FULL                                                     \
  Arg(1)->Arg(5)->Arg(10)->Arg(20)->Arg(50)->Arg(100)->Arg(200)->Arg(500)

// clang-format off
#define BM_OPTS \
  ->MinTime(0.3)
// clang-format on

// Register macro
#define REGISTER_DYNAMICS_BM(TopologyTag, JointTag, OpTag)                     \
  BENCHMARK(BM_##TopologyTag##_##JointTag##_##OpTag)->BM_SCALE_ARGS BM_OPTS

// For operations that scale well (FK, integration), include 500
#define REGISTER_DYNAMICS_BM_FULL(TopologyTag, JointTag, OpTag)                \
  BENCHMARK(BM_##TopologyTag##_##JointTag##_##OpTag)->BM_SCALE_ARGS_FULL BM_OPTS

// ============================================================================
// Section C: Forward Kinematics Benchmarks
// ============================================================================

// --- Serial Chain ---
DEFINE_DYNAMICS_BM(
    Serial,
    makeSerialChain,
    RevoluteJoint,
    Revolute,
    ForwardKinematics,
    skel->computeForwardKinematics(true, true, true))
REGISTER_DYNAMICS_BM_FULL(Serial, Revolute, ForwardKinematics);

DEFINE_DYNAMICS_BM(
    Serial,
    makeSerialChain,
    BallJoint,
    Ball,
    ForwardKinematics,
    skel->computeForwardKinematics(true, true, true))
REGISTER_DYNAMICS_BM_FULL(Serial, Ball, ForwardKinematics);

DEFINE_DYNAMICS_BM(
    Serial,
    makeSerialChain,
    FreeJoint,
    Free,
    ForwardKinematics,
    skel->computeForwardKinematics(true, true, true))
REGISTER_DYNAMICS_BM(Serial, Free, ForwardKinematics);

// --- Binary Tree ---
DEFINE_DYNAMICS_BM(
    BinaryTree,
    makeBinaryTree,
    RevoluteJoint,
    Revolute,
    ForwardKinematics,
    skel->computeForwardKinematics(true, true, true))
REGISTER_DYNAMICS_BM_FULL(BinaryTree, Revolute, ForwardKinematics);

DEFINE_DYNAMICS_BM(
    BinaryTree,
    makeBinaryTree,
    BallJoint,
    Ball,
    ForwardKinematics,
    skel->computeForwardKinematics(true, true, true))
REGISTER_DYNAMICS_BM(BinaryTree, Ball, ForwardKinematics);

DEFINE_DYNAMICS_BM(
    BinaryTree,
    makeBinaryTree,
    FreeJoint,
    Free,
    ForwardKinematics,
    skel->computeForwardKinematics(true, true, true))
REGISTER_DYNAMICS_BM(BinaryTree, Free, ForwardKinematics);

// --- Star ---
DEFINE_DYNAMICS_BM(
    Star,
    makeStar,
    RevoluteJoint,
    Revolute,
    ForwardKinematics,
    skel->computeForwardKinematics(true, true, true))
REGISTER_DYNAMICS_BM_FULL(Star, Revolute, ForwardKinematics);

DEFINE_DYNAMICS_BM(
    Star,
    makeStar,
    BallJoint,
    Ball,
    ForwardKinematics,
    skel->computeForwardKinematics(true, true, true))
REGISTER_DYNAMICS_BM(Star, Ball, ForwardKinematics);

DEFINE_DYNAMICS_BM(
    Star,
    makeStar,
    FreeJoint,
    Free,
    ForwardKinematics,
    skel->computeForwardKinematics(true, true, true))
REGISTER_DYNAMICS_BM(Star, Free, ForwardKinematics);

// ============================================================================
// Section D: Forward Dynamics Benchmarks (ABA)
// ============================================================================

DEFINE_DYNAMICS_BM(
    Serial,
    makeSerialChain,
    RevoluteJoint,
    Revolute,
    ForwardDynamics,
    skel->computeForwardDynamics())
REGISTER_DYNAMICS_BM_FULL(Serial, Revolute, ForwardDynamics);

DEFINE_DYNAMICS_BM(
    Serial,
    makeSerialChain,
    BallJoint,
    Ball,
    ForwardDynamics,
    skel->computeForwardDynamics())
REGISTER_DYNAMICS_BM(Serial, Ball, ForwardDynamics);

DEFINE_DYNAMICS_BM(
    Serial,
    makeSerialChain,
    FreeJoint,
    Free,
    ForwardDynamics,
    skel->computeForwardDynamics())
REGISTER_DYNAMICS_BM(Serial, Free, ForwardDynamics);

DEFINE_DYNAMICS_BM(
    BinaryTree,
    makeBinaryTree,
    RevoluteJoint,
    Revolute,
    ForwardDynamics,
    skel->computeForwardDynamics())
REGISTER_DYNAMICS_BM_FULL(BinaryTree, Revolute, ForwardDynamics);

DEFINE_DYNAMICS_BM(
    BinaryTree,
    makeBinaryTree,
    BallJoint,
    Ball,
    ForwardDynamics,
    skel->computeForwardDynamics())
REGISTER_DYNAMICS_BM(BinaryTree, Ball, ForwardDynamics);

DEFINE_DYNAMICS_BM(
    BinaryTree,
    makeBinaryTree,
    FreeJoint,
    Free,
    ForwardDynamics,
    skel->computeForwardDynamics())
REGISTER_DYNAMICS_BM(BinaryTree, Free, ForwardDynamics);

DEFINE_DYNAMICS_BM(
    Star,
    makeStar,
    RevoluteJoint,
    Revolute,
    ForwardDynamics,
    skel->computeForwardDynamics())
REGISTER_DYNAMICS_BM_FULL(Star, Revolute, ForwardDynamics);

DEFINE_DYNAMICS_BM(
    Star,
    makeStar,
    BallJoint,
    Ball,
    ForwardDynamics,
    skel->computeForwardDynamics())
REGISTER_DYNAMICS_BM(Star, Ball, ForwardDynamics);

DEFINE_DYNAMICS_BM(
    Star,
    makeStar,
    FreeJoint,
    Free,
    ForwardDynamics,
    skel->computeForwardDynamics())
REGISTER_DYNAMICS_BM(Star, Free, ForwardDynamics);

// ============================================================================
// Section E: Inverse Dynamics Benchmarks (RNEA)
// ============================================================================

DEFINE_DYNAMICS_BM(
    Serial,
    makeSerialChain,
    RevoluteJoint,
    Revolute,
    InverseDynamics,
    skel->computeInverseDynamics(false, false, false))
REGISTER_DYNAMICS_BM_FULL(Serial, Revolute, InverseDynamics);

DEFINE_DYNAMICS_BM(
    Serial,
    makeSerialChain,
    BallJoint,
    Ball,
    InverseDynamics,
    skel->computeInverseDynamics(false, false, false))
REGISTER_DYNAMICS_BM(Serial, Ball, InverseDynamics);

DEFINE_DYNAMICS_BM(
    Serial,
    makeSerialChain,
    FreeJoint,
    Free,
    InverseDynamics,
    skel->computeInverseDynamics(false, false, false))
REGISTER_DYNAMICS_BM(Serial, Free, InverseDynamics);

DEFINE_DYNAMICS_BM(
    BinaryTree,
    makeBinaryTree,
    RevoluteJoint,
    Revolute,
    InverseDynamics,
    skel->computeInverseDynamics(false, false, false))
REGISTER_DYNAMICS_BM(BinaryTree, Revolute, InverseDynamics);

DEFINE_DYNAMICS_BM(
    Star,
    makeStar,
    RevoluteJoint,
    Revolute,
    InverseDynamics,
    skel->computeInverseDynamics(false, false, false))
REGISTER_DYNAMICS_BM(Star, Revolute, InverseDynamics);

// ============================================================================
// Section F: Mass Matrix Benchmarks (CRB)
// ============================================================================

DEFINE_DYNAMICS_BM(
    Serial, makeSerialChain, RevoluteJoint, Revolute, MassMatrix, {
      auto m = skel->getMassMatrix();
      benchmark::DoNotOptimize(m);
    })
REGISTER_DYNAMICS_BM_FULL(Serial, Revolute, MassMatrix);

DEFINE_DYNAMICS_BM(Serial, makeSerialChain, BallJoint, Ball, MassMatrix, {
  auto m = skel->getMassMatrix();
  benchmark::DoNotOptimize(m);
})
REGISTER_DYNAMICS_BM(Serial, Ball, MassMatrix);

DEFINE_DYNAMICS_BM(Serial, makeSerialChain, FreeJoint, Free, MassMatrix, {
  auto m = skel->getMassMatrix();
  benchmark::DoNotOptimize(m);
})
REGISTER_DYNAMICS_BM(Serial, Free, MassMatrix);

DEFINE_DYNAMICS_BM(
    BinaryTree, makeBinaryTree, RevoluteJoint, Revolute, MassMatrix, {
      auto m = skel->getMassMatrix();
      benchmark::DoNotOptimize(m);
    })
REGISTER_DYNAMICS_BM(BinaryTree, Revolute, MassMatrix);

DEFINE_DYNAMICS_BM(Star, makeStar, RevoluteJoint, Revolute, MassMatrix, {
  auto m = skel->getMassMatrix();
  benchmark::DoNotOptimize(m);
})
REGISTER_DYNAMICS_BM(Star, Revolute, MassMatrix);

// ============================================================================
// Section G: Coriolis + Gravity Benchmarks
// ============================================================================

DEFINE_DYNAMICS_BM(
    Serial, makeSerialChain, RevoluteJoint, Revolute, CoriolisGravity, {
      auto f = skel->getCoriolisAndGravityForces();
      benchmark::DoNotOptimize(f);
    })
REGISTER_DYNAMICS_BM_FULL(Serial, Revolute, CoriolisGravity);

DEFINE_DYNAMICS_BM(Serial, makeSerialChain, BallJoint, Ball, CoriolisGravity, {
  auto f = skel->getCoriolisAndGravityForces();
  benchmark::DoNotOptimize(f);
})
REGISTER_DYNAMICS_BM(Serial, Ball, CoriolisGravity);

DEFINE_DYNAMICS_BM(
    BinaryTree, makeBinaryTree, RevoluteJoint, Revolute, CoriolisGravity, {
      auto f = skel->getCoriolisAndGravityForces();
      benchmark::DoNotOptimize(f);
    })
REGISTER_DYNAMICS_BM(BinaryTree, Revolute, CoriolisGravity);

// ============================================================================
// Section H: Integration Benchmarks
// ============================================================================

DEFINE_DYNAMICS_BM(
    Serial,
    makeSerialChain,
    RevoluteJoint,
    Revolute,
    Integration,
    skel->integratePositions(0.001);
    skel->integrateVelocities(0.001))
REGISTER_DYNAMICS_BM_FULL(Serial, Revolute, Integration);

DEFINE_DYNAMICS_BM(
    Serial,
    makeSerialChain,
    BallJoint,
    Ball,
    Integration,
    skel->integratePositions(0.001);
    skel->integrateVelocities(0.001))
REGISTER_DYNAMICS_BM_FULL(Serial, Ball, Integration);

DEFINE_DYNAMICS_BM(
    Serial,
    makeSerialChain,
    FreeJoint,
    Free,
    Integration,
    skel->integratePositions(0.001);
    skel->integrateVelocities(0.001))
REGISTER_DYNAMICS_BM(Serial, Free, Integration);

// ============================================================================
// Section I: World::step() Benchmarks (full pipeline)
// ============================================================================

// World::step includes FK + FD + collision + constraints + integration.
// We use a separate function (not the macro) because setup is more complex.

using MakerFn = SkeletonPtr (*)(int, const std::string&);

static void BM_WorldStep_Impl(benchmark::State& state, MakerFn maker)
{
  const int N = static_cast<int>(state.range(0));
  auto skel = maker(N, "ws");
  randomizeState(skel);

  // Energy conservation sanity check (once, outside timed loop)
  assert(checkEnergyConservation(skel, 50, 0.5));

  // Create world for timed measurement (with gravity)
  auto world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);
  world->addSkeleton(skel);

  for (auto _ : state) {
    world->step();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}

static void BM_Serial_Revolute_WorldStep(benchmark::State& state)
{
  BM_WorldStep_Impl(state, &makeSerialChain<RevoluteJoint>);
}
BENCHMARK(BM_Serial_Revolute_WorldStep)->BM_SCALE_ARGS_FULL BM_OPTS;

static void BM_Serial_Ball_WorldStep(benchmark::State& state)
{
  BM_WorldStep_Impl(state, &makeSerialChain<BallJoint>);
}
BENCHMARK(BM_Serial_Ball_WorldStep)->BM_SCALE_ARGS BM_OPTS;

static void BM_Serial_Free_WorldStep(benchmark::State& state)
{
  BM_WorldStep_Impl(state, &makeSerialChain<FreeJoint>);
}
BENCHMARK(BM_Serial_Free_WorldStep)->BM_SCALE_ARGS BM_OPTS;

static void BM_BinaryTree_Revolute_WorldStep(benchmark::State& state)
{
  BM_WorldStep_Impl(state, &makeBinaryTree<RevoluteJoint>);
}
BENCHMARK(BM_BinaryTree_Revolute_WorldStep)->BM_SCALE_ARGS BM_OPTS;

static void BM_Star_Revolute_WorldStep(benchmark::State& state)
{
  BM_WorldStep_Impl(state, &makeStar<RevoluteJoint>);
}
BENCHMARK(BM_Star_Revolute_WorldStep)->BM_SCALE_ARGS BM_OPTS;

// ============================================================================
// Section J: Multi-Skeleton World::step() Benchmarks
// ============================================================================

static void BM_MultiSkeleton_WorldStep(benchmark::State& state)
{
  const int numSkeletons = static_cast<int>(state.range(0));
  auto world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);

  for (int i = 0; i < numSkeletons; ++i) {
    auto skel = makeSerialChain<RevoluteJoint>(20, "ms" + std::to_string(i));
    randomizeState(skel);
    world->addSkeleton(skel);
  }

  for (auto _ : state) {
    world->step();
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * numSkeletons);
  state.counters["skeletons"] = numSkeletons;
  state.counters["bodies_per_skel"] = 20;
  state.counters["total_bodies"] = numSkeletons * 20;
}
BENCHMARK(BM_MultiSkeleton_WorldStep)
    ->Arg(1)
    ->Arg(5)
    ->Arg(10)
    ->Arg(25)
    ->Arg(50)
    ->Arg(100) BM_OPTS;

// ============================================================================
// Section K: Parallel Worlds Benchmark
// ============================================================================

static void BM_ParallelWorlds(benchmark::State& state)
{
  const int numThreads = static_cast<int>(state.range(0));
  const int stepsPerThread = 10;

  // Build independent worlds (one per thread) — outside timed loop
  std::vector<WorldPtr> worlds(numThreads);
  for (int t = 0; t < numThreads; ++t) {
    worlds[t] = World::create("pw_" + std::to_string(t));
    worlds[t]->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    worlds[t]->setTimeStep(0.001);
    auto skel = makeSerialChain<RevoluteJoint>(20, "pw_s" + std::to_string(t));
    randomizeState(skel);
    worlds[t]->addSkeleton(skel);
  }

  for (auto _ : state) {
    std::vector<std::thread> threads;
    threads.reserve(numThreads);
    for (int t = 0; t < numThreads; ++t) {
      threads.emplace_back([&worlds, t, stepsPerThread]() {
        for (int s = 0; s < stepsPerThread; ++s) {
          worlds[t]->step();
        }
      });
    }
    for (auto& th : threads) {
      th.join();
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * numThreads * stepsPerThread);
  state.counters["threads"] = numThreads;
  state.counters["steps_per_thread"] = stepsPerThread;
}
BENCHMARK(BM_ParallelWorlds)->Arg(1)->Arg(2)->Arg(4)->Arg(8) BM_OPTS;

// ============================================================================
// Section L: Unitree G1-like Humanoid Benchmarks (29 DOF)
// ============================================================================

static void BM_G1Humanoid_ForwardKinematics(benchmark::State& state)
{
  auto skel = makeG1Humanoid();
  randomizeState(skel);

  for (auto _ : state) {
    state.PauseTiming();
    dirtyAllJoints(skel);
    randomizeState(skel);
    state.ResumeTiming();

    skel->computeForwardKinematics(true, true, true);
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_G1Humanoid_ForwardKinematics)->MinTime(0.3);

static void BM_G1Humanoid_ForwardDynamics(benchmark::State& state)
{
  auto skel = makeG1Humanoid();
  randomizeState(skel);

  for (auto _ : state) {
    state.PauseTiming();
    dirtyAllJoints(skel);
    randomizeState(skel);
    state.ResumeTiming();

    skel->computeForwardDynamics();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_G1Humanoid_ForwardDynamics)->MinTime(0.3);

static void BM_G1Humanoid_InverseDynamics(benchmark::State& state)
{
  auto skel = makeG1Humanoid();
  randomizeState(skel);

  for (auto _ : state) {
    state.PauseTiming();
    dirtyAllJoints(skel);
    randomizeState(skel);
    state.ResumeTiming();

    skel->computeInverseDynamics(false, false, false);
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_G1Humanoid_InverseDynamics)->MinTime(0.3);

static void BM_G1Humanoid_MassMatrix(benchmark::State& state)
{
  auto skel = makeG1Humanoid();
  randomizeState(skel);

  for (auto _ : state) {
    state.PauseTiming();
    dirtyAllJoints(skel);
    randomizeState(skel);
    state.ResumeTiming();

    auto m = skel->getMassMatrix();
    benchmark::DoNotOptimize(m);
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_G1Humanoid_MassMatrix)->MinTime(0.3);

static void BM_G1Humanoid_WorldStep(benchmark::State& state)
{
  auto skel = makeG1Humanoid();
  randomizeState(skel);
  assert(checkEnergyConservation(skel, 50, 0.5));

  auto world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);
  world->addSkeleton(skel);

  for (auto _ : state) {
    world->step();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_G1Humanoid_WorldStep)->MinTime(0.3);

// ============================================================================
// Section N: Contact/Collision World::step() Benchmarks
// ============================================================================

static SkeletonPtr makeGroundPlane()
{
  auto ground = Skeleton::create("ground");
  auto [joint, body] = ground->createJointAndBodyNodePair<WeldJoint>(nullptr);
  joint->setName("ground_weld");
  body->setName("ground_body");
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(10.0, 10.0, 0.1));
  body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
      shape);
  return ground;
}

static void BM_Contact_StackedBoxes_WorldStep(benchmark::State& state)
{
  const int numBoxes = static_cast<int>(state.range(0));
  auto world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);
  world->addSkeleton(makeGroundPlane());

  for (int i = 0; i < numBoxes; ++i) {
    auto box = Skeleton::create("box_" + std::to_string(i));
    auto [joint, body] = box->createJointAndBodyNodePair<FreeJoint>(nullptr);
    joint->setName("box_joint_" + std::to_string(i));
    body->setName("box_body_" + std::to_string(i));

    auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(0.3, 0.3, 0.3));
    body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
        shape);

    BodyNode::Properties props;
    props.mInertia.setMass(1.0);
    props.mInertia.setMoment(0.1 * Eigen::Matrix3d::Identity());
    body->setInertia(props.mInertia);

    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(0.0, 0.0, 0.2 + i * 0.35);
    joint->setTransformFromParentBodyNode(tf);

    world->addSkeleton(box);
  }

  for (auto _ : state) {
    world->step();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["num_skeletons"]
      = static_cast<double>(world->getNumSkeletons());
  state.counters["total_dofs"] = static_cast<double>(numBoxes * 6);
}
BENCHMARK(BM_Contact_StackedBoxes_WorldStep)
    ->Arg(2)
    ->Arg(5)
    ->Arg(10)
    ->Arg(20)
    ->MinTime(0.1);

static void BM_Contact_ChainOnGround_WorldStep(benchmark::State& state)
{
  const int N = static_cast<int>(state.range(0));
  auto world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);
  world->addSkeleton(makeGroundPlane());

  // Build a serial chain with collision spheres
  auto skel = Skeleton::create("chain");
  BodyNode* parent = nullptr;
  for (int i = 0; i < N; ++i) {
    RevoluteJoint::Properties jprops;
    jprops.mName = "chain_j" + std::to_string(i);
    jprops.mAxis = Eigen::Vector3d::UnitZ();

    BodyNode::Properties bprops;
    bprops.mName = "chain_b" + std::to_string(i);
    bprops.mInertia.setMass(1.0);
    bprops.mInertia.setMoment(0.1 * Eigen::Matrix3d::Identity());

    auto [joint, body] = skel->createJointAndBodyNodePair<RevoluteJoint>(
        parent, jprops, bprops);

    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    if (i == 0) {
      offset.translation() = Eigen::Vector3d(0.0, 0.0, 1.0);
    } else {
      offset.translation() = Eigen::Vector3d(0.1, 0.0, 0.0);
    }
    joint->setTransformFromParentBodyNode(offset);

    auto sphere = std::make_shared<SphereShape>(0.05);
    body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(sphere);

    parent = body;
  }
  randomizeState(skel);
  world->addSkeleton(skel);

  for (auto _ : state) {
    world->step();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_Contact_ChainOnGround_WorldStep)
    ->Arg(5)
    ->Arg(10)
    ->Arg(20)
    ->Arg(50)
    ->MinTime(0.1);

// ============================================================================
// Section O: Skeleton Lifecycle Benchmarks (Create/Clone/Destroy)
// ============================================================================

static void BM_Lifecycle_Create_SerialChain(benchmark::State& state)
{
  const int N = static_cast<int>(state.range(0));
  for (auto _ : state) {
    auto skel = makeSerialChain<RevoluteJoint>(N, "lc_create");
    benchmark::DoNotOptimize(skel.get());
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(N);
}
BENCHMARK(BM_Lifecycle_Create_SerialChain)
    ->Arg(1)
    ->Arg(5)
    ->Arg(10)
    ->Arg(20)
    ->Arg(50)
    ->Arg(100)
    ->Arg(200)
    ->MinTime(0.1);

static void BM_Lifecycle_Clone_SerialChain(benchmark::State& state)
{
  const int N = static_cast<int>(state.range(0));
  auto skel = makeSerialChain<RevoluteJoint>(N, "lc_clone");
  for (auto _ : state) {
    auto clone = skel->cloneSkeleton();
    benchmark::DoNotOptimize(clone.get());
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(N);
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_Lifecycle_Clone_SerialChain)
    ->Arg(1)
    ->Arg(5)
    ->Arg(10)
    ->Arg(20)
    ->Arg(50)
    ->Arg(100)
    ->Arg(200)
    ->MinTime(0.1);

static void BM_Lifecycle_CreateDestroy_Repeated(benchmark::State& state)
{
  const int N = static_cast<int>(state.range(0));
  for (auto _ : state) {
    std::vector<SkeletonPtr> skels;
    skels.reserve(10);
    for (int i = 0; i < 10; ++i) {
      skels.push_back(
          makeSerialChain<RevoluteJoint>(N, "lc_churn_" + std::to_string(i)));
    }
    skels.clear();
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * 10);
  state.counters["total_bodies"] = static_cast<double>(10 * N);
}
BENCHMARK(BM_Lifecycle_CreateDestroy_Repeated)
    ->Arg(5)
    ->Arg(10)
    ->Arg(20)
    ->Arg(50)
    ->Arg(100)
    ->MinTime(0.1);

static void BM_Lifecycle_Clone_BinaryTree(benchmark::State& state)
{
  const int N = static_cast<int>(state.range(0));
  auto skel = makeBinaryTree<RevoluteJoint>(N, "lc_btclone");
  for (auto _ : state) {
    auto clone = skel->cloneSkeleton();
    benchmark::DoNotOptimize(clone.get());
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_Lifecycle_Clone_BinaryTree)
    ->Arg(5)
    ->Arg(10)
    ->Arg(20)
    ->Arg(50)
    ->Arg(100)
    ->MinTime(0.1);

static void BM_Lifecycle_Clone_G1Humanoid(benchmark::State& state)
{
  auto skel = makeG1Humanoid();
  for (auto _ : state) {
    auto clone = skel->cloneSkeleton();
    benchmark::DoNotOptimize(clone.get());
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_Lifecycle_Clone_G1Humanoid)->MinTime(0.1);

// ============================================================================
// Section P: Allocation Metrics (Heap Allocation Counting)
// ============================================================================
// These benchmarks use the global operator new/delete overrides above to count
// heap allocations per operation. The allocation COUNT is what matters -- it
// shows how pool/arena allocation reduces heap pressure.

static void BM_AllocMetrics_WorldStep_Serial20(benchmark::State& state)
{
  auto world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);
  auto skel = makeSerialChain<RevoluteJoint>(20, "alloc_s20");
  randomizeState(skel);
  world->addSkeleton(skel);

  // Warm up
  world->step();

  int64_t total_allocs = 0;
  int64_t total_bytes = 0;
  for (auto _ : state) {
    state.PauseTiming();
    auto start_allocs = g_alloc_count.load(std::memory_order_relaxed);
    auto start_bytes = g_alloc_bytes.load(std::memory_order_relaxed);
    state.ResumeTiming();

    world->step();

    state.PauseTiming();
    total_allocs = g_alloc_count.load(std::memory_order_relaxed) - start_allocs;
    total_bytes = g_alloc_bytes.load(std::memory_order_relaxed) - start_bytes;
    state.ResumeTiming();
  }
  state.counters["allocs_per_step"] = benchmark::Counter(
      static_cast<double>(total_allocs), benchmark::Counter::kAvgIterations);
  state.counters["bytes_per_step"] = benchmark::Counter(
      static_cast<double>(total_bytes), benchmark::Counter::kAvgIterations);
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
}
BENCHMARK(BM_AllocMetrics_WorldStep_Serial20)->MinTime(0.3);

static void BM_AllocMetrics_WorldStep_G1Humanoid(benchmark::State& state)
{
  auto world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);
  auto skel = makeG1Humanoid();
  randomizeState(skel);
  world->addSkeleton(skel);

  world->step();

  int64_t total_allocs = 0;
  int64_t total_bytes = 0;
  for (auto _ : state) {
    state.PauseTiming();
    auto start_allocs = g_alloc_count.load(std::memory_order_relaxed);
    auto start_bytes = g_alloc_bytes.load(std::memory_order_relaxed);
    state.ResumeTiming();

    world->step();

    state.PauseTiming();
    total_allocs
        += g_alloc_count.load(std::memory_order_relaxed) - start_allocs;
    total_bytes += g_alloc_bytes.load(std::memory_order_relaxed) - start_bytes;
    state.ResumeTiming();
  }
  state.counters["allocs_per_step"] = benchmark::Counter(
      static_cast<double>(total_allocs), benchmark::Counter::kAvgIterations);
  state.counters["bytes_per_step"] = benchmark::Counter(
      static_cast<double>(total_bytes), benchmark::Counter::kAvgIterations);
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
}
BENCHMARK(BM_AllocMetrics_WorldStep_G1Humanoid)->MinTime(0.3);

static void BM_AllocMetrics_WorldStep_Contact10(benchmark::State& state)
{
  auto world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);
  world->addSkeleton(makeGroundPlane());

  for (int i = 0; i < 10; ++i) {
    auto box = Skeleton::create("abox_" + std::to_string(i));
    auto [joint, body] = box->createJointAndBodyNodePair<FreeJoint>(nullptr);
    joint->setName("abox_j_" + std::to_string(i));
    body->setName("abox_b_" + std::to_string(i));

    auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(0.3, 0.3, 0.3));
    body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
        shape);
    Inertia inertia;
    inertia.setMass(1.0);
    inertia.setMoment(0.1 * Eigen::Matrix3d::Identity());
    body->setInertia(inertia);

    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(0.0, 0.0, 0.2 + i * 0.35);
    joint->setTransform(tf);
    world->addSkeleton(box);
  }

  // Warm up — let boxes settle
  for (int i = 0; i < 10; ++i) {
    world->step();
  }

  int64_t total_allocs = 0;
  int64_t total_bytes = 0;
  for (auto _ : state) {
    state.PauseTiming();
    auto start_allocs = g_alloc_count.load(std::memory_order_relaxed);
    auto start_bytes = g_alloc_bytes.load(std::memory_order_relaxed);
    state.ResumeTiming();

    world->step();

    state.PauseTiming();
    total_allocs
        += g_alloc_count.load(std::memory_order_relaxed) - start_allocs;
    total_bytes += g_alloc_bytes.load(std::memory_order_relaxed) - start_bytes;
    state.ResumeTiming();
  }
  state.counters["allocs_per_step"] = benchmark::Counter(
      static_cast<double>(total_allocs), benchmark::Counter::kAvgIterations);
  state.counters["bytes_per_step"] = benchmark::Counter(
      static_cast<double>(total_bytes), benchmark::Counter::kAvgIterations);
  state.counters["num_skeletons"]
      = static_cast<double>(world->getNumSkeletons());
}
BENCHMARK(BM_AllocMetrics_WorldStep_Contact10)->MinTime(0.3);

static void BM_AllocMetrics_Create_Serial50(benchmark::State& state)
{
  int64_t total_allocs = 0;
  int64_t total_bytes = 0;
  for (auto _ : state) {
    state.PauseTiming();
    auto start_allocs = g_alloc_count.load(std::memory_order_relaxed);
    auto start_bytes = g_alloc_bytes.load(std::memory_order_relaxed);
    state.ResumeTiming();

    auto skel = makeSerialChain<RevoluteJoint>(50, "alloc_create");
    benchmark::DoNotOptimize(skel.get());

    state.PauseTiming();
    total_allocs
        += g_alloc_count.load(std::memory_order_relaxed) - start_allocs;
    total_bytes += g_alloc_bytes.load(std::memory_order_relaxed) - start_bytes;
    state.ResumeTiming();
  }
  state.counters["allocs_per_create"] = benchmark::Counter(
      static_cast<double>(total_allocs), benchmark::Counter::kAvgIterations);
  state.counters["bytes_per_create"] = benchmark::Counter(
      static_cast<double>(total_bytes), benchmark::Counter::kAvgIterations);
}
BENCHMARK(BM_AllocMetrics_Create_Serial50)->MinTime(0.3);

static void BM_AllocMetrics_Clone_Serial50(benchmark::State& state)
{
  auto skel = makeSerialChain<RevoluteJoint>(50, "alloc_clone");

  int64_t total_allocs = 0;
  int64_t total_bytes = 0;
  for (auto _ : state) {
    state.PauseTiming();
    auto start_allocs = g_alloc_count.load(std::memory_order_relaxed);
    auto start_bytes = g_alloc_bytes.load(std::memory_order_relaxed);
    state.ResumeTiming();

    auto clone = skel->cloneSkeleton();
    benchmark::DoNotOptimize(clone.get());

    state.PauseTiming();
    total_allocs
        += g_alloc_count.load(std::memory_order_relaxed) - start_allocs;
    total_bytes += g_alloc_bytes.load(std::memory_order_relaxed) - start_bytes;
    state.ResumeTiming();
  }
  state.counters["allocs_per_clone"] = benchmark::Counter(
      static_cast<double>(total_allocs), benchmark::Counter::kAvgIterations);
  state.counters["bytes_per_clone"] = benchmark::Counter(
      static_cast<double>(total_bytes), benchmark::Counter::kAvgIterations);
}
BENCHMARK(BM_AllocMetrics_Clone_Serial50)->MinTime(0.3);
