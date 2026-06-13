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

// PLAN-083 CPU scene corpus packet seed. These benchmarks measure reduced
// runtime smoke scenes only; they are not paper-scale bridge, cone-twist,
// codimensional-rod, or mixed-domain performance claims.

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/deformable_body.hpp>
#include <dart/simulation/body/deformable_body_options.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/compute/sequential_executor.hpp>
#include <dart/simulation/compute/world_step_stage.hpp>
#include <dart/simulation/detail/affine_body_dynamics.hpp>
#include <dart/simulation/detail/newton_barrier/mixed_domain_coupling.hpp>
#include <dart/simulation/detail/newton_barrier/projected_newton.hpp>
#include <dart/simulation/detail/newton_barrier/psd_projection.hpp>
#include <dart/simulation/multibody/joint.hpp>
#include <dart/simulation/world.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <benchmark/benchmark.h>

#include <algorithm>
#include <array>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <cmath>

namespace sx = dart::simulation;
namespace sxdetail = dart::simulation::detail;
namespace nb = dart::simulation::detail::newton_barrier;

namespace {

constexpr std::array<double, 4> kBridgeBoardX{-0.45, -0.15, 0.15, 0.45};
constexpr double kMaxReducedEqualityResidual = 1e-8;
constexpr double kPi = 3.141592653589793238462643383279502884;

const Eigen::Vector3d kBridgeBoardHalfExtents(0.10, 0.16, 0.025);
const Eigen::Vector3d kBridgePostHalfExtents(0.05, 0.20, 0.08);
const Eigen::Vector3d kBridgeTravelerHalfExtents(0.07, 0.07, 0.07);
const Eigen::Vector3d kPulleySupportHalfExtents(0.08, 0.04, 0.08);
constexpr double kPulleyWheelRadius = 0.10;
const Eigen::Vector3d kPulleyLoadHalfExtents(0.055, 0.055, 0.055);
const Eigen::Vector3d kUmbrellaMastHalfExtents(0.04, 0.04, 0.28);
const Eigen::Vector3d kUmbrellaHubHalfExtents(0.05, 0.05, 0.04);
const Eigen::Vector3d kUmbrellaRibHalfExtents(0.18, 0.025, 0.025);
const Eigen::Vector3d kLyingFlatGroundHalfExtents(0.28, 0.18, 0.012);
const Eigen::Vector3d kLyingFlatRodHalfExtents(0.14, 0.015, 0.015);
const Eigen::Vector3d kLyingFlatStaticCcdWitnessHalfExtents(
    0.010, 0.020, 0.020);
constexpr double kLyingFlatRingRadius = 0.028;
constexpr std::size_t kLyingFlatGridColumns = 6;
constexpr std::size_t kLyingFlatGridRows = 4;
constexpr double kLyingFlatGridSpacing = 0.055;
const Eigen::Vector3d kCandyShellHalfExtents(0.24, 0.16, 0.025);
constexpr std::size_t kCandyGridColumns = 5;
constexpr std::size_t kCandyGridRows = 5;
constexpr double kCandyGridSpacing = 0.055;
const Eigen::Vector3d kNunchakuHandleHalfExtents(0.18, 0.035, 0.035);
const Eigen::Vector3d kWindmillHubHalfExtents(0.05, 0.05, 0.05);
const Eigen::Vector3d kWindmillBladeHalfExtents(0.045, 0.22, 0.025);
const Eigen::Vector3d kWindmillStrikerHalfExtents(0.09, 0.09, 0.09);
const Eigen::Vector3d kTerrainHalfExtents(0.70, 0.45, 0.025);
const Eigen::Vector3d kTerrainChassisHalfExtents(0.22, 0.12, 0.04);
constexpr double kTerrainWheelRadius = 0.06;
const Eigen::Vector3d kPrecessionGroundHalfExtents(0.55, 0.45, 0.025);
constexpr double kPrecessionWheelRadius = 0.16;
constexpr double kPrecessionWheelHalfHeight = 0.035;
const Eigen::Vector3d kRagdollGroundHalfExtents(0.42, 0.36, 0.025);
const Eigen::Vector3d kRagdollTorsoHalfExtents(0.10, 0.07, 0.14);
const Eigen::Vector3d kRagdollArmHalfExtents(0.12, 0.025, 0.035);
const Eigen::Vector3d kRagdollLegHalfExtents(0.035, 0.04, 0.16);
constexpr double kRagdollHeadRadius = 0.055;
constexpr std::size_t kAbdReducedWreckingBallPairCount = 1;
constexpr std::size_t kAbdReducedCardCount = 8;
constexpr std::size_t kAbdReducedChain8PairCount = 8;
constexpr std::size_t kAbdReducedChain16PairCount = 16;
constexpr std::size_t kAbdReducedChain96PairCount = 96;
constexpr std::size_t kAbdReducedGearsPairCount = 28;
constexpr std::size_t kAbdReducedBulletSmallPairCount = 16;
constexpr std::size_t kAbdReducedBulletMediumPairCount = 48;
constexpr std::size_t kAbdReducedBulletLargePairCount = 96;
constexpr std::size_t kAbdReducedComplexGeometryPairCount = 29;
constexpr std::size_t kAbdReducedFemCouplingPairCount = 27;
const std::array<Eigen::Vector3d, 4> kTerrainWheelOffsets{{
    Eigen::Vector3d(-0.16, -0.14, -0.10),
    Eigen::Vector3d(-0.16, 0.14, -0.10),
    Eigen::Vector3d(0.16, -0.14, -0.10),
    Eigen::Vector3d(0.16, 0.14, -0.10),
}};
const std::array<Eigen::Vector3d, 5> kRagdollPartPositions{{
    Eigen::Vector3d(0.00, 0.0, 0.62),
    Eigen::Vector3d(-0.19, 0.0, 0.45),
    Eigen::Vector3d(0.19, 0.0, 0.45),
    Eigen::Vector3d(-0.07, 0.0, 0.16),
    Eigen::Vector3d(0.07, 0.0, 0.16),
}};

sx::CollisionShape makeOffsetBox(
    const Eigen::Vector3d& halfExtents, const Eigen::Vector3d& localOffset)
{
  auto shape = sx::CollisionShape::makeBox(halfExtents);
  shape.localTransform.translation() = localOffset;
  return shape;
}

std::size_t gridIndex(
    const std::size_t columns, const std::size_t col, const std::size_t row)
{
  return row * columns + col;
}

std::vector<sx::DeformableEdge> makeGridEdges(
    const std::size_t columns,
    const std::size_t rows,
    const std::vector<Eigen::Vector3d>& positions)
{
  std::vector<sx::DeformableEdge> edges;
  const auto add = [&](const std::size_t a, const std::size_t b) {
    edges.push_back({a, b, (positions[a] - positions[b]).norm()});
  };

  for (std::size_t row = 0; row < rows; ++row) {
    for (std::size_t col = 0; col < columns; ++col) {
      const std::size_t here = gridIndex(columns, col, row);
      if (col + 1 < columns) {
        add(here, gridIndex(columns, col + 1, row));
      }
      if (row + 1 < rows) {
        add(here, gridIndex(columns, col, row + 1));
      }
      if (col + 1 < columns && row + 1 < rows) {
        add(here, gridIndex(columns, col + 1, row + 1));
        add(gridIndex(columns, col + 1, row), gridIndex(columns, col, row + 1));
      }
    }
  }
  return edges;
}

std::vector<sx::DeformableSurfaceTriangle> makeGridSurfaceTriangles(
    const std::size_t columns, const std::size_t rows)
{
  std::vector<sx::DeformableSurfaceTriangle> triangles;
  for (std::size_t row = 0; row + 1 < rows; ++row) {
    for (std::size_t col = 0; col + 1 < columns; ++col) {
      const std::size_t a = gridIndex(columns, col, row);
      const std::size_t b = gridIndex(columns, col + 1, row);
      const std::size_t c = gridIndex(columns, col, row + 1);
      const std::size_t d = gridIndex(columns, col + 1, row + 1);
      triangles.push_back({a, b, c});
      triangles.push_back({b, d, c});
    }
  }
  return triangles;
}

sx::DeformableBodyOptions makePlan083SingleNodeBodyOptions(
    const Eigen::Vector3d& position, const Eigen::Vector3d& velocity)
{
  sx::DeformableBodyOptions options;
  options.positions = {position};
  options.velocities = {velocity};
  options.masses = {1.0};
  return options;
}

sx::DeformableBodyOptions makePlan083SurfaceCrossingBodyOptions()
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-1.0, -1.0, 0.0),
         Eigen::Vector3d(1.0, -1.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0),
         Eigen::Vector3d(0.0, 0.0, 1.0)};
  options.velocities
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d(0.0, 0.0, -20.0)};
  options.masses = {1.0, 1.0, 1.0, 1.0};
  options.fixedNodes = {0, 1, 2};
  options.surfaceTriangles = {sx::DeformableSurfaceTriangle{0, 1, 2}};
  return options;
}

sx::DeformableBodyOptions makePlan083InterBodyMovingPointOptions()
{
  auto options = makePlan083SurfaceCrossingBodyOptions();
  options.positions[0].z() = 3.0;
  options.positions[1].z() = 3.0;
  options.positions[2].z() = 3.0;
  return options;
}

sx::DeformableBodyOptions makePlan083StationaryTriangleObstacleOptions()
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-1.0, -1.0, 0.0),
         Eigen::Vector3d(1.0, -1.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0)};
  options.velocities
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero()};
  options.masses = {1.0, 1.0, 1.0};
  options.fixedNodes = {0, 1, 2};
  options.surfaceTriangles = {sx::DeformableSurfaceTriangle{0, 1, 2}};
  return options;
}

sx::RigidBody addPlan083StaticSurfaceCcdBox(
    sx::World& world,
    const std::string& name,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& halfExtents)
{
  sx::RigidBodyOptions options;
  options.isStatic = true;
  options.position = position;
  auto body = world.addRigidBody(name, options);
  body.setCollisionShape(sx::CollisionShape::makeBox(halfExtents));
  body.setDeformableSurfaceCcdObstacle(true);
  return body;
}

sx::RigidBody addPlan083MovingSurfaceCcdBox(
    sx::World& world,
    const std::string& name,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& halfExtents,
    const Eigen::Vector3d& linearVelocity)
{
  sx::RigidBodyOptions options;
  options.isStatic = false;
  options.position = position;
  auto body = world.addRigidBody(name, options);
  body.setCollisionShape(sx::CollisionShape::makeBox(halfExtents));
  body.setDeformableSurfaceCcdObstacle(true);
  body.setLinearVelocity(linearVelocity);
  return body;
}

struct BodySnapshot
{
  sx::RigidBody body;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
};

void snapshotBody(
    std::vector<BodySnapshot>& snapshots, const sx::RigidBody& body)
{
  snapshots.push_back(
      {body,
       body.getTransform(),
       body.getLinearVelocity(),
       body.getAngularVelocity()});
}

struct HangingBridgeFixture
{
  HangingBridgeFixture()
  {
    world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
    world.setTimeStep(0.005);
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

    sx::RigidBodyOptions leftPostOptions;
    leftPostOptions.isStatic = true;
    leftPostOptions.position = Eigen::Vector3d(-0.65, 0.0, 0.56);
    leftPost.emplace(
        world.addRigidBody("plan083_bridge_left_post", leftPostOptions));
    leftPost->setCollisionShape(
        sx::CollisionShape::makeBox(kBridgePostHalfExtents));

    sx::RigidBodyOptions rightPostOptions;
    rightPostOptions.isStatic = true;
    rightPostOptions.position = Eigen::Vector3d(0.65, 0.0, 0.56);
    rightPost.emplace(
        world.addRigidBody("plan083_bridge_right_post", rightPostOptions));
    rightPost->setCollisionShape(
        sx::CollisionShape::makeBox(kBridgePostHalfExtents));

    sx::RigidBody parent = *leftPost;
    boards.reserve(kBridgeBoardX.size());
    for (std::size_t index = 0; index < kBridgeBoardX.size(); ++index) {
      sx::RigidBodyOptions boardOptions;
      boardOptions.mass = 0.25;
      boardOptions.position = Eigen::Vector3d(kBridgeBoardX[index], 0.0, 0.50);
      auto board = world.addRigidBody(
          "plan083_bridge_board_" + std::to_string(index), boardOptions);
      board.setFriction(0.7);
      board.setCollisionShape(
          sx::CollisionShape::makeBox(kBridgeBoardHalfExtents));
      world.addRigidBodyFixedJoint(
          "plan083_bridge_point_connection_" + std::to_string(index),
          parent,
          board);
      boards.push_back(board);
      parent = board;
    }

    sx::RigidBodyOptions travelerOptions;
    travelerOptions.mass = 0.12;
    travelerOptions.position = Eigen::Vector3d(-0.60, 0.0, 0.82);
    travelerOptions.linearVelocity = Eigen::Vector3d(0.35, 0.0, -0.05);
    traveler.emplace(
        world.addRigidBody("plan083_bridge_traveler", travelerOptions));
    traveler->setFriction(0.4);
    traveler->setCollisionShape(
        sx::CollisionShape::makeBox(kBridgeTravelerHalfExtents));

    snapshotBody(snapshots, *leftPost);
    snapshotBody(snapshots, *rightPost);
    for (const auto& board : boards) {
      snapshotBody(snapshots, board);
    }
    snapshotBody(snapshots, *traveler);

    world.enterSimulationMode();
  }

  void reset()
  {
    world.setTime(0.0);
    for (auto& snapshot : snapshots) {
      snapshot.body.setTransform(snapshot.transform);
      snapshot.body.setLinearVelocity(snapshot.linearVelocity);
      snapshot.body.setAngularVelocity(snapshot.angularVelocity);
    }
  }

  double maxBoardSag() const
  {
    double minBoardZ = std::numeric_limits<double>::infinity();
    for (const auto& board : boards) {
      minBoardZ = std::min(minBoardZ, board.getTranslation().z());
    }
    return 0.50 - minBoardZ;
  }

  sx::World world;
  std::optional<sx::RigidBody> leftPost;
  std::optional<sx::RigidBody> rightPost;
  std::vector<sx::RigidBody> boards;
  std::optional<sx::RigidBody> traveler;
  std::vector<BodySnapshot> snapshots;
};

struct NunchakuFixture
{
  NunchakuFixture()
  {
    world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
    world.setTimeStep(0.005);
    world.setGravity(Eigen::Vector3d::Zero());

    sx::RigidBodyOptions anchorOptions;
    anchorOptions.isStatic = true;
    anchorOptions.position = Eigen::Vector3d(0.0, 0.0, 0.75);
    anchor.emplace(
        world.addRigidBody("plan083_nunchaku_anchor_handle", anchorOptions));
    anchor->setCollisionShape(makeOffsetBox(
        kNunchakuHandleHalfExtents,
        Eigen::Vector3d(-kNunchakuHandleHalfExtents.x(), 0.0, 0.0)));

    sx::RigidBodyOptions swingOptions;
    swingOptions.mass = 0.2;
    swingOptions.position = anchorOptions.position;
    swingOptions.angularVelocity = Eigen::Vector3d(0.0, 0.0, 1.5);
    swinging.emplace(
        world.addRigidBody("plan083_nunchaku_swing_handle", swingOptions));
    swinging->setCollisionShape(makeOffsetBox(
        kNunchakuHandleHalfExtents,
        Eigen::Vector3d(kNunchakuHandleHalfExtents.x(), 0.0, 0.0)));

    (void)world.addRigidBodyRevoluteJoint(
        "plan083_nunchaku_hinge", *anchor, *swinging, Eigen::Vector3d::UnitZ());

    snapshotBody(snapshots, *anchor);
    snapshotBody(snapshots, *swinging);

    world.enterSimulationMode();
  }

  void reset()
  {
    world.setTime(0.0);
    for (auto& snapshot : snapshots) {
      snapshot.body.setTransform(snapshot.transform);
      snapshot.body.setLinearVelocity(snapshot.linearVelocity);
      snapshot.body.setAngularVelocity(snapshot.angularVelocity);
    }
  }

  double swingingTipRadius() const
  {
    const Eigen::Vector3d localTip(
        2.0 * kNunchakuHandleHalfExtents.x(), 0.0, 0.0);
    return (swinging->getTransform() * localTip - anchor->getTranslation())
        .norm();
  }

  sx::World world;
  std::optional<sx::RigidBody> anchor;
  std::optional<sx::RigidBody> swinging;
  std::vector<BodySnapshot> snapshots;
};

struct PulleyFixture
{
  PulleyFixture()
  {
    world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
    world.setTimeStep(0.005);
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

    sx::RigidBodyOptions supportOptions;
    supportOptions.isStatic = true;
    supportOptions.position = Eigen::Vector3d(0.0, 0.0, 0.76);
    support.emplace(
        world.addRigidBody("plan083_pulley_support", supportOptions));
    support->setCollisionShape(
        sx::CollisionShape::makeBox(kPulleySupportHalfExtents));

    sx::RigidBodyOptions wheelOptions;
    wheelOptions.mass = 0.16;
    wheelOptions.position = supportOptions.position;
    wheel.emplace(world.addRigidBody("plan083_pulley_wheel", wheelOptions));
    wheel->setCollisionShape(
        sx::CollisionShape::makeSphere(kPulleyWheelRadius));

    sx::RigidBodyOptions leftLoadOptions;
    leftLoadOptions.mass = 0.12;
    leftLoadOptions.position = Eigen::Vector3d(-0.20, 0.0, 0.48);
    leftLoad.emplace(
        world.addRigidBody("plan083_pulley_left_load", leftLoadOptions));
    leftLoad->setCollisionShape(
        sx::CollisionShape::makeBox(kPulleyLoadHalfExtents));

    sx::RigidBodyOptions rightLoadOptions;
    rightLoadOptions.mass = 0.18;
    rightLoadOptions.position = Eigen::Vector3d(0.20, 0.0, 0.42);
    rightLoad.emplace(
        world.addRigidBody("plan083_pulley_right_load", rightLoadOptions));
    rightLoad->setCollisionShape(
        sx::CollisionShape::makeBox(kPulleyLoadHalfExtents));

    (void)world.addRigidBodyRevoluteJoint(
        "plan083_pulley_hinge", *support, *wheel, Eigen::Vector3d::UnitY());
    (void)world.addRigidBodyFixedJoint(
        "plan083_pulley_left_point_connection", *wheel, *leftLoad);
    (void)world.addRigidBodyFixedJoint(
        "plan083_pulley_right_point_connection", *wheel, *rightLoad);

    snapshotBody(snapshots, *support);
    snapshotBody(snapshots, *wheel);
    snapshotBody(snapshots, *leftLoad);
    snapshotBody(snapshots, *rightLoad);

    world.enterSimulationMode();
  }

  void reset()
  {
    world.setTime(0.0);
    for (auto& snapshot : snapshots) {
      snapshot.body.setTransform(snapshot.transform);
      snapshot.body.setLinearVelocity(snapshot.linearVelocity);
      snapshot.body.setAngularVelocity(snapshot.angularVelocity);
    }
  }

  double loadHeightDifference() const
  {
    return leftLoad->getTranslation().z() - rightLoad->getTranslation().z();
  }

  double loadSeparation() const
  {
    return (leftLoad->getTranslation() - rightLoad->getTranslation()).norm();
  }

  sx::World world;
  std::optional<sx::RigidBody> support;
  std::optional<sx::RigidBody> wheel;
  std::optional<sx::RigidBody> leftLoad;
  std::optional<sx::RigidBody> rightLoad;
  std::vector<BodySnapshot> snapshots;
};

struct UmbrellaFixture
{
  UmbrellaFixture()
  {
    world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
    world.setTimeStep(0.005);
    world.setGravity(Eigen::Vector3d::Zero());

    sx::RigidBodyOptions mastOptions;
    mastOptions.isStatic = true;
    mastOptions.position = Eigen::Vector3d(0.0, 0.0, 0.48);
    mast.emplace(world.addRigidBody("plan083_umbrella_mast", mastOptions));
    mast->setCollisionShape(
        sx::CollisionShape::makeBox(kUmbrellaMastHalfExtents));

    sx::RigidBodyOptions hubOptions;
    hubOptions.mass = 0.12;
    hubOptions.position = Eigen::Vector3d(0.0, 0.0, 0.78);
    hubOptions.angularVelocity = Eigen::Vector3d(0.0, 0.9, 0.0);
    hub.emplace(world.addRigidBody("plan083_umbrella_hinged_hub", hubOptions));
    hub->setCollisionShape(
        sx::CollisionShape::makeBox(kUmbrellaHubHalfExtents));

    sx::RigidBodyOptions leftRibOptions;
    leftRibOptions.mass = 0.08;
    leftRibOptions.position = Eigen::Vector3d(-0.18, 0.0, 0.74);
    leftRibOptions.angularVelocity = hubOptions.angularVelocity;
    leftRib.emplace(
        world.addRigidBody("plan083_umbrella_left_rib", leftRibOptions));
    leftRib->setCollisionShape(
        sx::CollisionShape::makeBox(kUmbrellaRibHalfExtents));

    sx::RigidBodyOptions rightRibOptions;
    rightRibOptions.mass = 0.08;
    rightRibOptions.position = Eigen::Vector3d(0.18, 0.0, 0.74);
    rightRibOptions.angularVelocity = hubOptions.angularVelocity;
    rightRib.emplace(
        world.addRigidBody("plan083_umbrella_right_rib", rightRibOptions));
    rightRib->setCollisionShape(
        sx::CollisionShape::makeBox(kUmbrellaRibHalfExtents));

    (void)world.addRigidBodyRevoluteJoint(
        "plan083_umbrella_canopy_hinge", *mast, *hub, Eigen::Vector3d::UnitY());
    (void)world.addRigidBodyFixedJoint(
        "plan083_umbrella_left_rib_point_connection", *hub, *leftRib);
    (void)world.addRigidBodyFixedJoint(
        "plan083_umbrella_right_rib_point_connection", *hub, *rightRib);

    snapshotBody(snapshots, *mast);
    snapshotBody(snapshots, *hub);
    snapshotBody(snapshots, *leftRib);
    snapshotBody(snapshots, *rightRib);

    world.enterSimulationMode();
  }

  void reset()
  {
    world.setTime(0.0);
    for (auto& snapshot : snapshots) {
      snapshot.body.setTransform(snapshot.transform);
      snapshot.body.setLinearVelocity(snapshot.linearVelocity);
      snapshot.body.setAngularVelocity(snapshot.angularVelocity);
    }
  }

  double canopySpan() const
  {
    return (rightRib->getTranslation() - leftRib->getTranslation()).norm();
  }

  sx::World world;
  std::optional<sx::RigidBody> mast;
  std::optional<sx::RigidBody> hub;
  std::optional<sx::RigidBody> leftRib;
  std::optional<sx::RigidBody> rightRib;
  std::vector<BodySnapshot> snapshots;
};

struct LyingFlatFixture
{
  LyingFlatFixture()
  {
    world.setTimeStep(0.004);
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -1.0));
    const double halfWidth = 0.5 * kLyingFlatGridSpacing
                             * static_cast<double>(kLyingFlatGridColumns - 1u);
    const double halfDepth = 0.5 * kLyingFlatGridSpacing
                             * static_cast<double>(kLyingFlatGridRows - 1u);

    addObstacle(
        "plan083_lying_flat_ground",
        Eigen::Vector3d(0.0, 0.0, kLyingFlatGroundHalfExtents.z()),
        sx::CollisionShape::makeBox(kLyingFlatGroundHalfExtents));
    addObstacle(
        "plan083_lying_flat_reduced_rod",
        Eigen::Vector3d(0.0, 0.055, 0.044),
        sx::CollisionShape::makeBox(kLyingFlatRodHalfExtents));
    addObstacle(
        "plan083_lying_flat_left_ring",
        Eigen::Vector3d(-0.09, -0.045, 0.042),
        sx::CollisionShape::makeSphere(kLyingFlatRingRadius));
    addObstacle(
        "plan083_lying_flat_right_ring",
        Eigen::Vector3d(0.09, -0.045, 0.042),
        sx::CollisionShape::makeSphere(kLyingFlatRingRadius));
    addObstacle(
        "plan083_lying_flat_static_ccd_witness",
        Eigen::Vector3d(-0.040, -halfDepth, 0.078),
        sx::CollisionShape::makeBox(kLyingFlatStaticCcdWitnessHalfExtents),
        false);

    sx::DeformableBodyOptions clothOptions;
    for (std::size_t row = 0; row < kLyingFlatGridRows; ++row) {
      for (std::size_t col = 0; col < kLyingFlatGridColumns; ++col) {
        const double x
            = kLyingFlatGridSpacing * static_cast<double>(col) - halfWidth;
        const double y
            = kLyingFlatGridSpacing * static_cast<double>(row) - halfDepth;
        const double stagger = (col + row) % 2u == 0u ? 0.0 : 0.003;
        const double lateral
            = col < kLyingFlatGridColumns / 2u ? 0.025 : -0.025;
        const bool drivesStaticCcdWitness = row == 0u && col == 0u;
        const Eigen::Vector3d velocity
            = drivesStaticCcdWitness ? Eigen::Vector3d(40.0, 0.0, 0.0)
                                     : Eigen::Vector3d(lateral, 0.0, -0.02);
        clothOptions.positions.emplace_back(x, y, 0.078 + stagger);
        clothOptions.velocities.push_back(velocity);
        clothOptions.masses.push_back(0.02);
      }
    }
    clothOptions.edges = makeGridEdges(
        kLyingFlatGridColumns, kLyingFlatGridRows, clothOptions.positions);
    clothOptions.surfaceTriangles
        = makeGridSurfaceTriangles(kLyingFlatGridColumns, kLyingFlatGridRows);
    clothOptions.edgeStiffness = 80.0;
    clothOptions.damping = 1.4;
    clothOptions.material.frictionCoefficient = 0.35;

    initialPositions = clothOptions.positions;
    initialVelocities = clothOptions.velocities;
    cloth.emplace(world.addDeformableBody(
        "plan083_lying_flat_deformable_cloth", clothOptions));

    world.enterSimulationMode();
  }

  sx::RigidBody addObstacle(
      const std::string& name,
      const Eigen::Vector3d& position,
      const sx::CollisionShape& shape,
      const bool barrierOnly = true)
  {
    sx::RigidBodyOptions options;
    options.isStatic = true;
    options.position = position;
    sx::RigidBody body = world.addRigidBody(name, options);
    body.setCollisionShape(shape);
    body.setDeformableSurfaceCcdObstacle(true);
    body.setDeformableObstacleBarrierOnly(barrierOnly);
    obstacles.push_back(body);
    return body;
  }

  void reset()
  {
    world.setTime(0.0);
    for (std::size_t node = 0; node < initialPositions.size(); ++node) {
      cloth->setPosition(node, initialPositions[node]);
      cloth->setVelocity(node, initialVelocities[node]);
    }
  }

  double minClothHeight() const
  {
    double minHeight = std::numeric_limits<double>::infinity();
    for (std::size_t node = 0; node < cloth->getNodeCount(); ++node) {
      minHeight = std::min(minHeight, cloth->getPosition(node).z());
    }
    return minHeight;
  }

  double clothSpanX() const
  {
    double minX = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    for (std::size_t node = 0; node < cloth->getNodeCount(); ++node) {
      const double x = cloth->getPosition(node).x();
      minX = std::min(minX, x);
      maxX = std::max(maxX, x);
    }
    return maxX - minX;
  }

  double clothSpanY() const
  {
    double minY = std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();
    for (std::size_t node = 0; node < cloth->getNodeCount(); ++node) {
      const double y = cloth->getPosition(node).y();
      minY = std::min(minY, y);
      maxY = std::max(maxY, y);
    }
    return maxY - minY;
  }

  sx::World world;
  std::vector<sx::RigidBody> obstacles;
  std::optional<sx::DeformableBody> cloth;
  std::vector<Eigen::Vector3d> initialPositions;
  std::vector<Eigen::Vector3d> initialVelocities;
};

struct CandyFixture
{
  CandyFixture()
  {
    world.setTimeStep(0.004);
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -1.5));

    sx::RigidBodyOptions shellOptions;
    shellOptions.isStatic = true;
    shellOptions.position = Eigen::Vector3d(0.0, 0.0, 0.025);
    shell.emplace(
        world.addRigidBody("plan083_candy_reduced_shell", shellOptions));
    shell->setCollisionShape(
        sx::CollisionShape::makeBox(kCandyShellHalfExtents));
    shell->setDeformableSurfaceCcdObstacle(true);
    shell->setDeformableObstacleBarrierOnly(true);

    sx::DeformableBodyOptions clothOptions;
    const double halfWidth
        = 0.5 * kCandyGridSpacing * static_cast<double>(kCandyGridColumns - 1u);
    const double halfDepth
        = 0.5 * kCandyGridSpacing * static_cast<double>(kCandyGridRows - 1u);
    for (std::size_t row = 0; row < kCandyGridRows; ++row) {
      for (std::size_t col = 0; col < kCandyGridColumns; ++col) {
        const double x
            = kCandyGridSpacing * static_cast<double>(col) - halfWidth;
        const double y
            = kCandyGridSpacing * static_cast<double>(row) - halfDepth;
        const double lateral = row % 2u == 0u ? -0.08 : 0.08;
        clothOptions.positions.emplace_back(x, y, 0.075);
        clothOptions.velocities.emplace_back(lateral, 0.0, -0.05);
        clothOptions.masses.push_back(0.025);
      }
    }
    clothOptions.edges = makeGridEdges(
        kCandyGridColumns, kCandyGridRows, clothOptions.positions);
    clothOptions.surfaceTriangles
        = makeGridSurfaceTriangles(kCandyGridColumns, kCandyGridRows);
    clothOptions.edgeStiffness = 90.0;
    clothOptions.damping = 1.2;
    clothOptions.material.frictionCoefficient = 0.45;

    initialPositions = clothOptions.positions;
    initialVelocities = clothOptions.velocities;
    cloth.emplace(world.addDeformableBody(
        "plan083_candy_deformable_cloth", clothOptions));

    world.enterSimulationMode();
  }

  void reset()
  {
    world.setTime(0.0);
    for (std::size_t node = 0; node < initialPositions.size(); ++node) {
      cloth->setPosition(node, initialPositions[node]);
      cloth->setVelocity(node, initialVelocities[node]);
    }
  }

  double minClothHeight() const
  {
    double minHeight = std::numeric_limits<double>::infinity();
    for (std::size_t node = 0; node < cloth->getNodeCount(); ++node) {
      minHeight = std::min(minHeight, cloth->getPosition(node).z());
    }
    return minHeight;
  }

  double clothSpanX() const
  {
    double minX = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    for (std::size_t node = 0; node < cloth->getNodeCount(); ++node) {
      const double x = cloth->getPosition(node).x();
      minX = std::min(minX, x);
      maxX = std::max(maxX, x);
    }
    return maxX - minX;
  }

  sx::World world;
  std::optional<sx::RigidBody> shell;
  std::optional<sx::DeformableBody> cloth;
  std::vector<Eigen::Vector3d> initialPositions;
  std::vector<Eigen::Vector3d> initialVelocities;
};

struct NunchakuScalingFixture
{
  explicit NunchakuScalingFixture(std::size_t pairCount)
  {
    world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
    world.setTimeStep(0.005);
    world.setGravity(Eigen::Vector3d::Zero());

    const std::size_t columns = static_cast<std::size_t>(
        std::ceil(std::sqrt(static_cast<double>(pairCount))));
    anchors.reserve(pairCount);
    swinging.reserve(pairCount);
    for (std::size_t index = 0; index < pairCount; ++index) {
      const Eigen::Vector3d basePosition(
          1.2 * static_cast<double>(index % columns),
          1.2 * static_cast<double>(index / columns),
          0.75);

      sx::RigidBodyOptions anchorOptions;
      anchorOptions.isStatic = true;
      anchorOptions.position = basePosition;
      auto anchor = world.addRigidBody(
          "plan083_nunchaku_scaling_anchor_" + std::to_string(index),
          anchorOptions);
      anchor.setCollisionShape(makeOffsetBox(
          kNunchakuHandleHalfExtents,
          Eigen::Vector3d(-kNunchakuHandleHalfExtents.x(), 0.0, 0.0)));

      sx::RigidBodyOptions swingOptions;
      swingOptions.mass = 0.2;
      swingOptions.position = basePosition;
      swingOptions.angularVelocity = Eigen::Vector3d(0.0, 0.0, 1.5);
      auto swing = world.addRigidBody(
          "plan083_nunchaku_scaling_swing_" + std::to_string(index),
          swingOptions);
      swing.setCollisionShape(makeOffsetBox(
          kNunchakuHandleHalfExtents,
          Eigen::Vector3d(kNunchakuHandleHalfExtents.x(), 0.0, 0.0)));

      (void)world.addRigidBodyRevoluteJoint(
          "plan083_nunchaku_scaling_hinge_" + std::to_string(index),
          anchor,
          swing,
          Eigen::Vector3d::UnitZ());

      anchors.push_back(anchor);
      swinging.push_back(swing);
    }

    for (const auto& anchor : anchors) {
      snapshotBody(snapshots, anchor);
    }
    for (const auto& swing : swinging) {
      snapshotBody(snapshots, swing);
    }

    world.enterSimulationMode();
  }

  void reset()
  {
    world.setTime(0.0);
    for (auto& snapshot : snapshots) {
      snapshot.body.setTransform(snapshot.transform);
      snapshot.body.setLinearVelocity(snapshot.linearVelocity);
      snapshot.body.setAngularVelocity(snapshot.angularVelocity);
    }
  }

  sx::World world;
  std::vector<sx::RigidBody> anchors;
  std::vector<sx::RigidBody> swinging;
  std::vector<BodySnapshot> snapshots;
};

struct WindmillFixture
{
  WindmillFixture()
  {
    world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
    world.setTimeStep(0.005);
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

    sx::RigidBodyOptions hubOptions;
    hubOptions.isStatic = true;
    hubOptions.position = Eigen::Vector3d(0.0, 0.0, 0.65);
    hub.emplace(world.addRigidBody("plan083_windmill_hub", hubOptions));
    hub->setCollisionShape(
        sx::CollisionShape::makeBox(kWindmillHubHalfExtents));

    sx::RigidBodyOptions bladeOptions;
    bladeOptions.mass = 0.25;
    bladeOptions.position = hubOptions.position;
    bladeOptions.angularVelocity = Eigen::Vector3d(0.0, 1.2, 0.0);
    blade.emplace(world.addRigidBody("plan083_windmill_blade", bladeOptions));
    blade->setFriction(0.6);
    blade->setCollisionShape(makeOffsetBox(
        kWindmillBladeHalfExtents, Eigen::Vector3d(0.0, 0.0, 0.18)));

    sx::RigidBodyOptions strikerOptions;
    strikerOptions.mass = 0.18;
    strikerOptions.position = Eigen::Vector3d(0.0, 0.0, 0.955);
    strikerOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -0.25);
    striker.emplace(
        world.addRigidBody("plan083_windmill_falling_box", strikerOptions));
    striker->setFriction(0.6);
    striker->setCollisionShape(
        sx::CollisionShape::makeBox(kWindmillStrikerHalfExtents));

    (void)world.addRigidBodyRevoluteJoint(
        "plan083_windmill_hinge", *hub, *blade, Eigen::Vector3d::UnitY());

    snapshotBody(snapshots, *hub);
    snapshotBody(snapshots, *blade);
    snapshotBody(snapshots, *striker);

    world.enterSimulationMode();
  }

  void reset()
  {
    world.setTime(0.0);
    for (auto& snapshot : snapshots) {
      snapshot.body.setTransform(snapshot.transform);
      snapshot.body.setLinearVelocity(snapshot.linearVelocity);
      snapshot.body.setAngularVelocity(snapshot.angularVelocity);
    }
  }

  double bladeTipRadius() const
  {
    const Eigen::Vector3d localTip(0.0, 0.0, 0.36);
    return (blade->getTransform() * localTip - hub->getTranslation()).norm();
  }

  double strikerBladeClearance() const
  {
    const double bladeTop
        = (blade->getTransform() * Eigen::Vector3d(0.0, 0.0, 0.18)).z()
          + kWindmillBladeHalfExtents.z();
    const double strikerBottom
        = striker->getTranslation().z() - kWindmillStrikerHalfExtents.z();
    return strikerBottom - bladeTop;
  }

  sx::World world;
  std::optional<sx::RigidBody> hub;
  std::optional<sx::RigidBody> blade;
  std::optional<sx::RigidBody> striker;
  std::vector<BodySnapshot> snapshots;
};

struct TerrainVehicleFixture
{
  TerrainVehicleFixture()
  {
    world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
    world.setTimeStep(0.005);
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

    sx::RigidBodyOptions terrainOptions;
    terrainOptions.isStatic = true;
    terrainOptions.position = Eigen::Vector3d(0.0, 0.0, -0.025);
    terrain.emplace(
        world.addRigidBody("plan083_vehicle_terrain", terrainOptions));
    terrain->setFriction(0.8);
    terrain->setCollisionShape(
        sx::CollisionShape::makeBox(kTerrainHalfExtents));

    sx::RigidBodyOptions chassisOptions;
    chassisOptions.mass = 0.45;
    chassisOptions.position = Eigen::Vector3d(0.0, 0.0, 0.17);
    chassisOptions.linearVelocity = Eigen::Vector3d(0.12, 0.0, 0.0);
    chassis.emplace(
        world.addRigidBody("plan083_vehicle_chassis", chassisOptions));
    chassis->setFriction(0.7);
    chassis->setCollisionShape(
        sx::CollisionShape::makeBox(kTerrainChassisHalfExtents));

    wheels.reserve(kTerrainWheelOffsets.size());
    for (std::size_t index = 0; index < kTerrainWheelOffsets.size(); ++index) {
      sx::RigidBodyOptions wheelOptions;
      wheelOptions.mass = 0.08;
      wheelOptions.position
          = chassisOptions.position + kTerrainWheelOffsets[index];
      wheelOptions.angularVelocity = Eigen::Vector3d(0.0, 3.0, 0.0);
      auto wheel = world.addRigidBody(
          "plan083_vehicle_passive_wheel_" + std::to_string(index),
          wheelOptions);
      wheel.setFriction(0.9);
      wheel.setCollisionShape(
          sx::CollisionShape::makeSphere(kTerrainWheelRadius));
      (void)world.addRigidBodyRevoluteJoint(
          "plan083_vehicle_wheel_hinge_" + std::to_string(index),
          *chassis,
          wheel,
          Eigen::Vector3d::UnitY());
      wheels.push_back(wheel);
    }

    snapshotBody(snapshots, *terrain);
    snapshotBody(snapshots, *chassis);
    for (const auto& wheel : wheels) {
      snapshotBody(snapshots, wheel);
    }

    world.enterSimulationMode();
  }

  void reset()
  {
    world.setTime(0.0);
    for (auto& snapshot : snapshots) {
      snapshot.body.setTransform(snapshot.transform);
      snapshot.body.setLinearVelocity(snapshot.linearVelocity);
      snapshot.body.setAngularVelocity(snapshot.angularVelocity);
    }
  }

  double minWheelGroundClearance() const
  {
    double clearance = std::numeric_limits<double>::infinity();
    for (const auto& wheel : wheels) {
      clearance = std::min(
          clearance, wheel.getTranslation().z() - kTerrainWheelRadius);
    }
    return clearance;
  }

  sx::World world;
  std::optional<sx::RigidBody> terrain;
  std::optional<sx::RigidBody> chassis;
  std::vector<sx::RigidBody> wheels;
  std::vector<BodySnapshot> snapshots;
};

struct PrecessionFixture
{
  PrecessionFixture()
  {
    world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
    world.setTimeStep(0.005);
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.025);
    ground.emplace(
        world.addRigidBody("plan083_precession_ground", groundOptions));
    ground->setFriction(0.8);
    ground->setCollisionShape(
        sx::CollisionShape::makeBox(kPrecessionGroundHalfExtents));

    sx::RigidBodyOptions wheelOptions;
    wheelOptions.mass = 0.22;
    wheelOptions.position = Eigen::Vector3d(-0.18, 0.0, kPrecessionWheelRadius);
    wheelOptions.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitX()));
    wheelOptions.linearVelocity = Eigen::Vector3d(0.28, 0.0, 0.0);
    wheelOptions.angularVelocity = Eigen::Vector3d(0.0, 8.0, 1.2);
    wheel.emplace(world.addRigidBody("plan083_precession_wheel", wheelOptions));
    wheel->setFriction(0.9);
    wheel->setCollisionShape(
        sx::CollisionShape::makeSphere(kPrecessionWheelRadius));

    snapshotBody(snapshots, *ground);
    snapshotBody(snapshots, *wheel);

    world.enterSimulationMode();
  }

  void reset()
  {
    world.setTime(0.0);
    for (auto& snapshot : snapshots) {
      snapshot.body.setTransform(snapshot.transform);
      snapshot.body.setLinearVelocity(snapshot.linearVelocity);
      snapshot.body.setAngularVelocity(snapshot.angularVelocity);
    }
  }

  double wheelGroundClearance() const
  {
    return wheel->getTranslation().z() - kPrecessionWheelRadius;
  }

  sx::World world;
  std::optional<sx::RigidBody> ground;
  std::optional<sx::RigidBody> wheel;
  std::vector<BodySnapshot> snapshots;
};

struct RagdollFixture
{
  RagdollFixture()
  {
    world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
    world.setTimeStep(0.005);
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.025);
    ground.emplace(world.addRigidBody("plan083_ragdoll_ground", groundOptions));
    ground->setFriction(0.8);
    ground->setCollisionShape(
        sx::CollisionShape::makeBox(kRagdollGroundHalfExtents));

    sx::RigidBodyOptions torsoOptions;
    torsoOptions.mass = 0.35;
    torsoOptions.position = Eigen::Vector3d(0.0, 0.0, 0.42);
    torsoOptions.linearVelocity = Eigen::Vector3d(0.04, 0.0, -0.04);
    torsoOptions.angularVelocity = Eigen::Vector3d(0.0, 0.6, 0.25);
    torso.emplace(world.addRigidBody("plan083_ragdoll_torso", torsoOptions));
    torso->setFriction(0.7);
    torso->setCollisionShape(
        sx::CollisionShape::makeBox(kRagdollTorsoHalfExtents));

    const std::array<std::string, 5> names{
        "plan083_ragdoll_head",
        "plan083_ragdoll_left_arm",
        "plan083_ragdoll_right_arm",
        "plan083_ragdoll_left_leg",
        "plan083_ragdoll_right_leg",
    };
    parts.reserve(names.size());
    for (std::size_t index = 0; index < names.size(); ++index) {
      sx::RigidBodyOptions partOptions;
      partOptions.mass = index == 0 ? 0.08 : 0.12;
      partOptions.position = kRagdollPartPositions[index];
      partOptions.linearVelocity = torsoOptions.linearVelocity;
      partOptions.angularVelocity = torsoOptions.angularVelocity;
      auto part = world.addRigidBody(names[index], partOptions);
      part.setFriction(0.8);
      if (index == 0) {
        part.setCollisionShape(
            sx::CollisionShape::makeSphere(kRagdollHeadRadius));
      } else if (index <= 2) {
        part.setCollisionShape(
            sx::CollisionShape::makeBox(kRagdollArmHalfExtents));
      } else {
        part.setCollisionShape(
            sx::CollisionShape::makeBox(kRagdollLegHalfExtents));
      }
      (void)world.addRigidBodyRevoluteJoint(
          "plan083_ragdoll_joint_" + std::to_string(index),
          *torso,
          part,
          index == 0 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY());
      parts.push_back(part);
    }

    snapshotBody(snapshots, *ground);
    snapshotBody(snapshots, *torso);
    for (const auto& part : parts) {
      snapshotBody(snapshots, part);
    }

    world.enterSimulationMode();
  }

  void reset()
  {
    world.setTime(0.0);
    for (auto& snapshot : snapshots) {
      snapshot.body.setTransform(snapshot.transform);
      snapshot.body.setLinearVelocity(snapshot.linearVelocity);
      snapshot.body.setAngularVelocity(snapshot.angularVelocity);
    }
  }

  double minLegGroundClearance() const
  {
    double clearance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 3; index < parts.size(); ++index) {
      clearance = std::min(
          clearance,
          parts[index].getTranslation().z() - kRagdollLegHalfExtents.z());
    }
    return clearance;
  }

  sx::World world;
  std::optional<sx::RigidBody> ground;
  std::optional<sx::RigidBody> torso;
  std::vector<sx::RigidBody> parts;
  std::vector<BodySnapshot> snapshots;
};

struct AbdHouseOfCardsFixture
{
  AbdHouseOfCardsFixture()
  {
    options.solve.barrier.squaredActivationDistance = 0.25;
    options.solve.barrier.stiffness = 0.04;
    options.solve.barrier.projectHessianToPsd = true;
    options.solve.inertialWeight = 1.0;
    options.solve.orthogonalityStiffness = 0.5;
    options.solve.gradientTolerance = 1e-5;
    options.solve.maxIterations = 64;
    options.solve.maxLineSearchIterations = 32;
    options.solve.maxStepNorm = 0.2;
    options.timeStep = 0.03;
    options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);

    triangleBody.dynamic = false;
    triangleA = Eigen::Vector3d(-0.6, -0.4, 0.0);
    triangleB = Eigen::Vector3d(0.6, -0.4, 0.0);
    triangleC = Eigen::Vector3d(-0.6, 0.6, 0.0);
    point = Eigen::Vector3d(0.0, 0.0, 0.0);

    initialCards.reserve(kAbdReducedCardCount);
    for (std::size_t index = 0; index < kAbdReducedCardCount; ++index) {
      sxdetail::AffineBodyState card;
      const double row = static_cast<double>(index / 4u);
      const double col = static_cast<double>(index % 4u);
      card.translation
          = Eigen::Vector3d(-0.24 + 0.16 * col, -0.08 + 0.18 * row, 0.08);
      card.linearMap = Eigen::AngleAxisd(
                           0.04 * static_cast<double>(index + 1u),
                           Eigen::Vector3d::UnitY())
                           .toRotationMatrix();
      card.linearMap(0, 1) += index % 2u == 0u ? 0.015 : -0.015;
      card.linearVelocity = Eigen::Vector3d(0.02 * col, 0.0, -2.0);
      card.affineVelocity(0, 1) = 0.4 + 0.05 * row;
      card.affineVelocity(1, 2) = -0.3 - 0.02 * col;
      card.mass = 0.15;
      initialCards.push_back(card);
    }
  }

  std::vector<sxdetail::AffinePointTriangleRuntimeStepResult> stepAll() const
  {
    std::vector<sxdetail::AffinePointTriangleRuntimeStepResult> results;
    results.reserve(initialCards.size());
    for (const auto& card : initialCards) {
      results.push_back(
          sxdetail::affinePointTriangleRuntimeStep(
              card,
              point,
              triangleBody,
              triangleA,
              triangleB,
              triangleC,
              options));
    }
    return results;
  }

  sxdetail::AffinePointTriangleRuntimeStepOptions options;
  sxdetail::AffineBodyState triangleBody;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d triangleA = Eigen::Vector3d::Zero();
  Eigen::Vector3d triangleB = Eigen::Vector3d::Zero();
  Eigen::Vector3d triangleC = Eigen::Vector3d::Zero();
  std::vector<sxdetail::AffineBodyState> initialCards;
};

struct AbdWreckingBallFixture
{
  AbdWreckingBallFixture()
  {
    options.solve.barrier.squaredActivationDistance = 0.25;
    options.solve.barrier.stiffness = 0.05;
    options.solve.barrier.projectHessianToPsd = true;
    options.solve.inertialWeight = 1.0;
    options.solve.orthogonalityStiffness = 0.5;
    options.solve.gradientTolerance = 1e-5;
    options.solve.maxIterations = 72;
    options.solve.maxLineSearchIterations = 32;
    options.solve.maxStepNorm = 0.18;
    options.timeStep = 0.025;
    options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);

    pointBody.translation = Eigen::Vector3d(0.0, 0.0, 0.10);
    pointBody.linearMap
        = Eigen::AngleAxisd(0.06, Eigen::Vector3d::UnitY()).toRotationMatrix();
    pointBody.linearVelocity = Eigen::Vector3d(0.18, 0.0, -2.2);
    pointBody.affineVelocity(0, 1) = 0.25;
    pointBody.affineVelocity(1, 2) = -0.2;
    pointBody.mass = 0.2;

    triangleBody.translation = Eigen::Vector3d(0.02, -0.01, 0.0);
    triangleBody.linearMap
        = Eigen::AngleAxisd(-0.03, Eigen::Vector3d::UnitX()).toRotationMatrix();
    triangleBody.linearVelocity = Eigen::Vector3d(-0.08, 0.0, 0.15);
    triangleBody.affineVelocity(0, 2) = -0.18;
    triangleBody.affineVelocity(2, 1) = 0.12;
    triangleBody.mass = 0.8;

    point = Eigen::Vector3d(0.0, 0.0, 0.0);
    triangleA = Eigen::Vector3d(-0.55, -0.45, 0.0);
    triangleB = Eigen::Vector3d(0.55, -0.45, 0.0);
    triangleC = Eigen::Vector3d(-0.55, 0.55, 0.0);
  }

  sxdetail::AffinePointTrianglePairRuntimeStepResult step() const
  {
    return sxdetail::affinePointTrianglePairRuntimeStep(
        pointBody,
        point,
        triangleBody,
        triangleA,
        triangleB,
        triangleC,
        options);
  }

  sxdetail::AffinePointTriangleRuntimeStepOptions options;
  sxdetail::AffineBodyState pointBody;
  sxdetail::AffineBodyState triangleBody;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d triangleA = Eigen::Vector3d::Zero();
  Eigen::Vector3d triangleB = Eigen::Vector3d::Zero();
  Eigen::Vector3d triangleC = Eigen::Vector3d::Zero();
};

struct AbdChainNetPair
{
  sxdetail::AffineBodyState pointBody;
  sxdetail::AffineBodyState triangleBody;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d triangleA = Eigen::Vector3d::Zero();
  Eigen::Vector3d triangleB = Eigen::Vector3d::Zero();
  Eigen::Vector3d triangleC = Eigen::Vector3d::Zero();
};

struct AbdChainNetFixture
{
  explicit AbdChainNetFixture(const std::size_t pairCount)
  {
    options.solve.barrier.squaredActivationDistance = 0.25;
    options.solve.barrier.stiffness = 0.045;
    options.solve.barrier.projectHessianToPsd = true;
    options.solve.inertialWeight = 1.0;
    options.solve.orthogonalityStiffness = 0.45;
    options.solve.gradientTolerance = 1e-5;
    options.solve.maxIterations = 72;
    options.solve.maxLineSearchIterations = 32;
    options.solve.maxStepNorm = 0.18;
    options.timeStep = 0.02;
    options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);

    pairs.reserve(pairCount);
    for (std::size_t index = 0; index < pairCount; ++index) {
      const double col = static_cast<double>(index % 8u);
      const double row = static_cast<double>(index / 8u);
      const double phase = static_cast<double>(index % 5u);
      const Eigen::Vector3d anchor(
          -0.28 + 0.08 * col, -0.18 + 0.045 * row, 0.0);

      AbdChainNetPair pair;
      pair.pointBody.translation = anchor + Eigen::Vector3d(0.0, 0.0, 0.10);
      pair.pointBody.linearMap
          = Eigen::AngleAxisd(0.035 + 0.004 * phase, Eigen::Vector3d::UnitY())
                .toRotationMatrix();
      pair.pointBody.linearVelocity
          = Eigen::Vector3d(0.02 + 0.004 * col, 0.0, -1.75 - 0.03 * phase);
      pair.pointBody.affineVelocity(0, 1) = 0.18 + 0.01 * phase;
      pair.pointBody.affineVelocity(1, 2) = -0.16 - 0.005 * col;
      pair.pointBody.mass = 0.12;

      pair.triangleBody.translation = anchor + Eigen::Vector3d(0.015, 0.0, 0.0);
      pair.triangleBody.linearMap
          = Eigen::AngleAxisd(-0.025 - 0.002 * phase, Eigen::Vector3d::UnitX())
                .toRotationMatrix();
      pair.triangleBody.linearVelocity
          = Eigen::Vector3d(-0.02, 0.0, 0.12 + 0.01 * phase);
      pair.triangleBody.affineVelocity(0, 2) = -0.12;
      pair.triangleBody.affineVelocity(2, 1) = 0.08 + 0.002 * col;
      pair.triangleBody.mass = 0.18;

      pair.point = Eigen::Vector3d::Zero();
      pair.triangleA = Eigen::Vector3d(-0.045, -0.035, 0.0);
      pair.triangleB = Eigen::Vector3d(0.055, -0.035, 0.0);
      pair.triangleC = Eigen::Vector3d(-0.045, 0.055, 0.0);
      pairs.push_back(pair);
    }
  }

  std::vector<sxdetail::AffinePointTrianglePairRuntimeStepResult> stepAll()
      const
  {
    std::vector<sxdetail::AffinePointTrianglePairRuntimeStepResult> results;
    results.reserve(pairs.size());
    for (const auto& pair : pairs) {
      results.push_back(
          sxdetail::affinePointTrianglePairRuntimeStep(
              pair.pointBody,
              pair.point,
              pair.triangleBody,
              pair.triangleA,
              pair.triangleB,
              pair.triangleC,
              options));
    }
    return results;
  }

  sxdetail::AffinePointTriangleRuntimeStepOptions options;
  std::vector<AbdChainNetPair> pairs;
};

} // namespace

//==============================================================================
static void BM_Plan083CpuScene_hanging_bridge_reduced_world_step(
    benchmark::State& state)
{
  HangingBridgeFixture fixture;
  sx::compute::SequentialExecutor executor;

  std::size_t failedSteps = 0;
  sx::compute::RigidIpcSolverStats lastStats;
  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    sx::compute::RigidIpcContactStage ipcStage(64);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(ipcStage);
    state.ResumeTiming();

    fixture.world.step(executor, pipeline);
    lastStats = ipcStage.getLastStats();
    const bool residualOk
        = std::isfinite(lastStats.finalEqualityResidualNorm)
          && lastStats.finalEqualityResidualNorm <= kMaxReducedEqualityResidual;
    const bool satisfiedNoOp = residualOk && lastStats.solverIterations == 0u
                               && lastStats.activeArticulationConstraints >= 2u;
    if (!residualOk || (lastStats.failed && !satisfiedNoOp)) {
      ++failedSteps;
    }
    benchmark::DoNotOptimize(fixture.traveler->getTranslation().data());
    benchmark::ClobberMemory();
  }

  state.counters["row_unb_fig_02"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["body_count"] = static_cast<double>(lastStats.bodyCount);
  state.counters["dynamic_body_count"]
      = static_cast<double>(lastStats.dynamicBodyCount);
  state.counters["fixed_joint_count"]
      = static_cast<double>(fixture.boards.size());
  state.counters["active_articulation_constraints"]
      = static_cast<double>(lastStats.activeArticulationConstraints);
  state.counters["solver_iterations"]
      = static_cast<double>(lastStats.solverIterations);
  state.counters["failed_steps"] = static_cast<double>(failedSteps);
  state.counters["final_equality_residual_norm"]
      = lastStats.finalEqualityResidualNorm;
  state.counters["traveler_height_m"] = fixture.traveler->getTranslation().z();
  state.counters["max_board_sag_m"] = fixture.maxBoardSag();
}
BENCHMARK(BM_Plan083CpuScene_hanging_bridge_reduced_world_step)
    ->Unit(benchmark::kMillisecond);

//==============================================================================
static void BM_Plan083CpuScene_pulley_system_reduced_world_step(
    benchmark::State& state)
{
  PulleyFixture fixture;
  sx::compute::SequentialExecutor executor;

  std::size_t failedSteps = 0;
  sx::compute::RigidIpcSolverStats lastStats;
  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    sx::compute::RigidIpcContactStage ipcStage(64);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(ipcStage);
    state.ResumeTiming();

    fixture.world.step(executor, pipeline);
    lastStats = ipcStage.getLastStats();
    const bool residualOk
        = std::isfinite(lastStats.finalEqualityResidualNorm)
          && lastStats.finalEqualityResidualNorm <= kMaxReducedEqualityResidual;
    const bool satisfiedNoOp = residualOk && lastStats.solverIterations == 0u
                               && lastStats.activeArticulationConstraints >= 8u;
    if (!residualOk || (lastStats.failed && !satisfiedNoOp)) {
      ++failedSteps;
    }
    benchmark::DoNotOptimize(fixture.wheel->getTransform().data());
    benchmark::DoNotOptimize(fixture.leftLoad->getTranslation().data());
    benchmark::DoNotOptimize(fixture.rightLoad->getTranslation().data());
    benchmark::ClobberMemory();
  }

  state.counters["row_unb_fig_03"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["body_count"] = static_cast<double>(lastStats.bodyCount);
  state.counters["dynamic_body_count"]
      = static_cast<double>(lastStats.dynamicBodyCount);
  state.counters["fixed_joint_count"] = 2.0;
  state.counters["revolute_joint_count"] = 1.0;
  state.counters["active_articulation_constraints"]
      = static_cast<double>(lastStats.activeArticulationConstraints);
  state.counters["solver_iterations"]
      = static_cast<double>(lastStats.solverIterations);
  state.counters["failed_steps"] = static_cast<double>(failedSteps);
  state.counters["final_equality_residual_norm"]
      = lastStats.finalEqualityResidualNorm;
  state.counters["left_load_height_m"] = fixture.leftLoad->getTranslation().z();
  state.counters["right_load_height_m"]
      = fixture.rightLoad->getTranslation().z();
  state.counters["load_height_difference_m"] = fixture.loadHeightDifference();
  state.counters["load_separation_m"] = fixture.loadSeparation();
  state.counters["wheel_spin_rad_s"] = fixture.wheel->getAngularVelocity().y();
}
BENCHMARK(BM_Plan083CpuScene_pulley_system_reduced_world_step)
    ->Unit(benchmark::kMillisecond);

//==============================================================================
static void BM_Plan083CpuScene_umbrella_reduced_world_step(
    benchmark::State& state)
{
  UmbrellaFixture fixture;
  sx::compute::SequentialExecutor executor;

  std::size_t failedSteps = 0;
  sx::compute::RigidIpcSolverStats lastStats;
  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    sx::compute::RigidIpcContactStage ipcStage(64);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(ipcStage);
    state.ResumeTiming();

    fixture.world.step(executor, pipeline);
    lastStats = ipcStage.getLastStats();
    const bool residualOk
        = std::isfinite(lastStats.finalEqualityResidualNorm)
          && lastStats.finalEqualityResidualNorm <= kMaxReducedEqualityResidual;
    const bool satisfiedNoOp = residualOk && lastStats.solverIterations == 0u
                               && lastStats.activeArticulationConstraints >= 8u;
    if (!residualOk || (lastStats.failed && !satisfiedNoOp)) {
      ++failedSteps;
    }
    benchmark::DoNotOptimize(fixture.hub->getTransform().data());
    benchmark::DoNotOptimize(fixture.leftRib->getTranslation().data());
    benchmark::DoNotOptimize(fixture.rightRib->getTranslation().data());
    benchmark::ClobberMemory();
  }

  state.counters["row_unb_fig_04"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["body_count"] = static_cast<double>(lastStats.bodyCount);
  state.counters["dynamic_body_count"]
      = static_cast<double>(lastStats.dynamicBodyCount);
  state.counters["fixed_joint_count"] = 2.0;
  state.counters["revolute_joint_count"] = 1.0;
  state.counters["active_articulation_constraints"]
      = static_cast<double>(lastStats.activeArticulationConstraints);
  state.counters["solver_iterations"]
      = static_cast<double>(lastStats.solverIterations);
  state.counters["failed_steps"] = static_cast<double>(failedSteps);
  state.counters["final_equality_residual_norm"]
      = lastStats.finalEqualityResidualNorm;
  state.counters["canopy_span_m"] = fixture.canopySpan();
  state.counters["hinge_angular_velocity_rad_s"]
      = fixture.hub->getAngularVelocity().y();
}
BENCHMARK(BM_Plan083CpuScene_umbrella_reduced_world_step)
    ->Unit(benchmark::kMillisecond);

void recordDeformableRuntimeContactCounters(
    benchmark::State& state, const sx::DeformableSolverDiagnostics& diagnostics)
{
  state.counters["line_search_trials"]
      = static_cast<double>(diagnostics.lineSearchTrials);
  state.counters["surface_contact_candidate_builds"]
      = static_cast<double>(diagnostics.surfaceContactCandidateBuilds);
  state.counters["surface_contact_point_triangle_candidates"]
      = static_cast<double>(diagnostics.surfaceContactPointTriangleCandidates);
  state.counters["surface_contact_edge_edge_candidates"]
      = static_cast<double>(diagnostics.surfaceContactEdgeEdgeCandidates);
  state.counters["surface_contact_ccd_point_triangle_checks"]
      = static_cast<double>(diagnostics.surfaceContactCcdPointTriangleChecks);
  state.counters["surface_contact_ccd_edge_edge_checks"]
      = static_cast<double>(diagnostics.surfaceContactCcdEdgeEdgeChecks);
  state.counters["surface_contact_ccd_hits"]
      = static_cast<double>(diagnostics.surfaceContactCcdHits);
  state.counters["surface_contact_ccd_limited_steps"]
      = static_cast<double>(diagnostics.surfaceContactCcdLimitedSteps);
  state.counters["surface_contact_ccd_zero_step_count"]
      = static_cast<double>(diagnostics.surfaceContactCcdZeroStepCount);
  state.counters["inter_body_surface_contact_candidate_builds"]
      = static_cast<double>(diagnostics.interBodySurfaceContactCandidateBuilds);
  state.counters["inter_body_surface_contact_point_triangle_candidates"]
      = static_cast<double>(
          diagnostics.interBodySurfaceContactPointTriangleCandidates);
  state.counters["inter_body_surface_contact_edge_edge_candidates"]
      = static_cast<double>(
          diagnostics.interBodySurfaceContactEdgeEdgeCandidates);
  state.counters["inter_body_surface_contact_ccd_point_triangle_checks"]
      = static_cast<double>(
          diagnostics.interBodySurfaceContactCcdPointTriangleChecks);
  state.counters["inter_body_surface_contact_ccd_edge_edge_checks"]
      = static_cast<double>(
          diagnostics.interBodySurfaceContactCcdEdgeEdgeChecks);
  state.counters["inter_body_surface_contact_ccd_hits"]
      = static_cast<double>(diagnostics.interBodySurfaceContactCcdHits);
  state.counters["inter_body_surface_contact_ccd_limited_steps"]
      = static_cast<double>(diagnostics.interBodySurfaceContactCcdLimitedSteps);
  state.counters["inter_body_surface_contact_ccd_zero_step_count"]
      = static_cast<double>(
          diagnostics.interBodySurfaceContactCcdZeroStepCount);
  state.counters["static_rigid_surface_ccd_snapshot_builds"]
      = static_cast<double>(diagnostics.staticRigidSurfaceCcdSnapshotBuilds);
  state.counters["static_rigid_surface_ccd_box_count"]
      = static_cast<double>(diagnostics.staticRigidSurfaceCcdBoxCount);
  state.counters["static_rigid_surface_ccd_sphere_count"]
      = static_cast<double>(diagnostics.staticRigidSurfaceCcdSphereCount);
  state.counters["static_rigid_surface_ccd_triangle_count"]
      = static_cast<double>(diagnostics.staticRigidSurfaceCcdTriangleCount);
  state.counters["static_rigid_surface_ccd_edge_count"]
      = static_cast<double>(diagnostics.staticRigidSurfaceCcdEdgeCount);
  state.counters["static_rigid_surface_ccd_candidate_builds"]
      = static_cast<double>(diagnostics.staticRigidSurfaceCcdCandidateBuilds);
  state.counters["static_rigid_surface_ccd_point_triangle_candidates"]
      = static_cast<double>(
          diagnostics.staticRigidSurfaceCcdPointTriangleCandidates);
  state.counters["static_rigid_surface_ccd_edge_edge_candidates"]
      = static_cast<double>(
          diagnostics.staticRigidSurfaceCcdEdgeEdgeCandidates);
  state.counters["static_rigid_surface_ccd_point_triangle_checks"]
      = static_cast<double>(
          diagnostics.staticRigidSurfaceCcdPointTriangleChecks);
  state.counters["static_rigid_surface_ccd_edge_edge_checks"]
      = static_cast<double>(diagnostics.staticRigidSurfaceCcdEdgeEdgeChecks);
  state.counters["static_rigid_surface_ccd_hits"]
      = static_cast<double>(diagnostics.staticRigidSurfaceCcdHits);
  state.counters["static_rigid_surface_ccd_limited_steps"]
      = static_cast<double>(diagnostics.staticRigidSurfaceCcdLimitedSteps);
  state.counters["static_rigid_surface_ccd_zero_step_count"]
      = static_cast<double>(diagnostics.staticRigidSurfaceCcdZeroStepCount);
  state.counters["moving_rigid_surface_ccd_snapshot_builds"]
      = static_cast<double>(diagnostics.movingRigidSurfaceCcdSnapshotBuilds);
  state.counters["moving_rigid_surface_ccd_box_count"]
      = static_cast<double>(diagnostics.movingRigidSurfaceCcdBoxCount);
  state.counters["moving_rigid_surface_ccd_sample_count"]
      = static_cast<double>(diagnostics.movingRigidSurfaceCcdSampleCount);
  state.counters["moving_rigid_surface_ccd_inflated_box_count"]
      = static_cast<double>(diagnostics.movingRigidSurfaceCcdInflatedBoxCount);
  state.counters["moving_rigid_surface_ccd_triangle_count"]
      = static_cast<double>(diagnostics.movingRigidSurfaceCcdTriangleCount);
  state.counters["moving_rigid_surface_ccd_edge_count"]
      = static_cast<double>(diagnostics.movingRigidSurfaceCcdEdgeCount);
  state.counters["moving_rigid_surface_ccd_candidate_builds"]
      = static_cast<double>(diagnostics.movingRigidSurfaceCcdCandidateBuilds);
  state.counters["moving_rigid_surface_ccd_point_triangle_candidates"]
      = static_cast<double>(
          diagnostics.movingRigidSurfaceCcdPointTriangleCandidates);
  state.counters["moving_rigid_surface_ccd_edge_edge_candidates"]
      = static_cast<double>(
          diagnostics.movingRigidSurfaceCcdEdgeEdgeCandidates);
  state.counters["moving_rigid_surface_ccd_point_triangle_checks"]
      = static_cast<double>(
          diagnostics.movingRigidSurfaceCcdPointTriangleChecks);
  state.counters["moving_rigid_surface_ccd_edge_edge_checks"]
      = static_cast<double>(diagnostics.movingRigidSurfaceCcdEdgeEdgeChecks);
  state.counters["moving_rigid_surface_ccd_hits"]
      = static_cast<double>(diagnostics.movingRigidSurfaceCcdHits);
  state.counters["moving_rigid_surface_ccd_limited_steps"]
      = static_cast<double>(diagnostics.movingRigidSurfaceCcdLimitedSteps);
  state.counters["moving_rigid_surface_ccd_zero_step_count"]
      = static_cast<double>(diagnostics.movingRigidSurfaceCcdZeroStepCount);
}

template <std::size_t Count, typename Member>
std::size_t sumDiagnosticsMember(
    const std::array<sx::DeformableSolverDiagnostics, Count>& diagnostics,
    Member member)
{
  std::size_t total = 0;
  for (const auto& diagnostic : diagnostics) {
    total += diagnostic.*member;
  }
  return total;
}

template <std::size_t Count>
void recordSummedDeformableRuntimeContactCounters(
    benchmark::State& state,
    const std::array<sx::DeformableSolverDiagnostics, Count>& diagnostics)
{
  state.counters["line_search_trials"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics, &sx::DeformableSolverDiagnostics::lineSearchTrials));
  state.counters["surface_contact_candidate_builds"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::surfaceContactCandidateBuilds));
  state.counters["surface_contact_point_triangle_candidates"] = static_cast<
      double>(sumDiagnosticsMember(
      diagnostics,
      &sx::DeformableSolverDiagnostics::surfaceContactPointTriangleCandidates));
  state.counters["surface_contact_edge_edge_candidates"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::surfaceContactEdgeEdgeCandidates));
  state.counters["surface_contact_ccd_point_triangle_checks"] = static_cast<
      double>(sumDiagnosticsMember(
      diagnostics,
      &sx::DeformableSolverDiagnostics::surfaceContactCcdPointTriangleChecks));
  state.counters["surface_contact_ccd_edge_edge_checks"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::surfaceContactCcdEdgeEdgeChecks));
  state.counters["surface_contact_ccd_hits"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::surfaceContactCcdHits));
  state.counters["surface_contact_ccd_limited_steps"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::surfaceContactCcdLimitedSteps));
  state.counters["surface_contact_ccd_zero_step_count"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::surfaceContactCcdZeroStepCount));
  state.counters["inter_body_surface_contact_candidate_builds"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::
              interBodySurfaceContactCandidateBuilds));
  state.counters["inter_body_surface_contact_point_triangle_candidates"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::
              interBodySurfaceContactPointTriangleCandidates));
  state.counters["inter_body_surface_contact_edge_edge_candidates"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::
              interBodySurfaceContactEdgeEdgeCandidates));
  state.counters["inter_body_surface_contact_ccd_point_triangle_checks"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::
              interBodySurfaceContactCcdPointTriangleChecks));
  state.counters["inter_body_surface_contact_ccd_edge_edge_checks"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::
              interBodySurfaceContactCcdEdgeEdgeChecks));
  state.counters["inter_body_surface_contact_ccd_hits"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::interBodySurfaceContactCcdHits));
  state.counters["inter_body_surface_contact_ccd_limited_steps"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::
              interBodySurfaceContactCcdLimitedSteps));
  state.counters["inter_body_surface_contact_ccd_zero_step_count"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::
              interBodySurfaceContactCcdZeroStepCount));
  state.counters["static_rigid_surface_ccd_snapshot_builds"] = static_cast<
      double>(sumDiagnosticsMember(
      diagnostics,
      &sx::DeformableSolverDiagnostics::staticRigidSurfaceCcdSnapshotBuilds));
  state.counters["static_rigid_surface_ccd_box_count"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::staticRigidSurfaceCcdBoxCount));
  state.counters["static_rigid_surface_ccd_sphere_count"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::staticRigidSurfaceCcdSphereCount));
  state.counters["static_rigid_surface_ccd_triangle_count"] = static_cast<
      double>(sumDiagnosticsMember(
      diagnostics,
      &sx::DeformableSolverDiagnostics::staticRigidSurfaceCcdTriangleCount));
  state.counters["static_rigid_surface_ccd_edge_count"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::staticRigidSurfaceCcdEdgeCount));
  state.counters["static_rigid_surface_ccd_candidate_builds"] = static_cast<
      double>(sumDiagnosticsMember(
      diagnostics,
      &sx::DeformableSolverDiagnostics::staticRigidSurfaceCcdCandidateBuilds));
  state.counters["static_rigid_surface_ccd_point_triangle_candidates"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::
              staticRigidSurfaceCcdPointTriangleCandidates));
  state.counters["static_rigid_surface_ccd_edge_edge_candidates"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::
              staticRigidSurfaceCcdEdgeEdgeCandidates));
  state.counters["static_rigid_surface_ccd_point_triangle_checks"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::
              staticRigidSurfaceCcdPointTriangleChecks));
  state.counters["static_rigid_surface_ccd_edge_edge_checks"] = static_cast<
      double>(sumDiagnosticsMember(
      diagnostics,
      &sx::DeformableSolverDiagnostics::staticRigidSurfaceCcdEdgeEdgeChecks));
  state.counters["static_rigid_surface_ccd_hits"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::staticRigidSurfaceCcdHits));
  state.counters["static_rigid_surface_ccd_limited_steps"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::staticRigidSurfaceCcdLimitedSteps));
  state.counters["static_rigid_surface_ccd_zero_step_count"] = static_cast<
      double>(sumDiagnosticsMember(
      diagnostics,
      &sx::DeformableSolverDiagnostics::staticRigidSurfaceCcdZeroStepCount));
  state.counters["moving_rigid_surface_ccd_snapshot_builds"] = static_cast<
      double>(sumDiagnosticsMember(
      diagnostics,
      &sx::DeformableSolverDiagnostics::movingRigidSurfaceCcdSnapshotBuilds));
  state.counters["moving_rigid_surface_ccd_box_count"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::movingRigidSurfaceCcdBoxCount));
  state.counters["moving_rigid_surface_ccd_sample_count"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::movingRigidSurfaceCcdSampleCount));
  state.counters["moving_rigid_surface_ccd_inflated_box_count"] = static_cast<
      double>(sumDiagnosticsMember(
      diagnostics,
      &sx::DeformableSolverDiagnostics::movingRigidSurfaceCcdInflatedBoxCount));
  state.counters["moving_rigid_surface_ccd_triangle_count"] = static_cast<
      double>(sumDiagnosticsMember(
      diagnostics,
      &sx::DeformableSolverDiagnostics::movingRigidSurfaceCcdTriangleCount));
  state.counters["moving_rigid_surface_ccd_edge_count"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::movingRigidSurfaceCcdEdgeCount));
  state.counters["moving_rigid_surface_ccd_candidate_builds"] = static_cast<
      double>(sumDiagnosticsMember(
      diagnostics,
      &sx::DeformableSolverDiagnostics::movingRigidSurfaceCcdCandidateBuilds));
  state.counters["moving_rigid_surface_ccd_point_triangle_candidates"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::
              movingRigidSurfaceCcdPointTriangleCandidates));
  state.counters["moving_rigid_surface_ccd_edge_edge_candidates"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::
              movingRigidSurfaceCcdEdgeEdgeCandidates));
  state.counters["moving_rigid_surface_ccd_point_triangle_checks"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::
              movingRigidSurfaceCcdPointTriangleChecks));
  state.counters["moving_rigid_surface_ccd_edge_edge_checks"] = static_cast<
      double>(sumDiagnosticsMember(
      diagnostics,
      &sx::DeformableSolverDiagnostics::movingRigidSurfaceCcdEdgeEdgeChecks));
  state.counters["moving_rigid_surface_ccd_hits"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::movingRigidSurfaceCcdHits));
  state.counters["moving_rigid_surface_ccd_limited_steps"]
      = static_cast<double>(sumDiagnosticsMember(
          diagnostics,
          &sx::DeformableSolverDiagnostics::movingRigidSurfaceCcdLimitedSteps));
  state.counters["moving_rigid_surface_ccd_zero_step_count"] = static_cast<
      double>(sumDiagnosticsMember(
      diagnostics,
      &sx::DeformableSolverDiagnostics::movingRigidSurfaceCcdZeroStepCount));
}

struct ExternalSurfaceCcdDiagnostics
{
  sx::DeformableSolverDiagnostics interBody;
  sx::DeformableSolverDiagnostics staticRigid;
  sx::DeformableSolverDiagnostics movingRigid;
  sx::DeformableSolverDiagnostics mixed;
  double interBodyLimitedPointZ = 0.0;
  double staticRigidLimitedPointX = 0.0;
  double movingRigidLimitedPointX = 0.0;
  double mixedInterBodyLimitedPointZ = 0.0;
  double mixedStaticRigidLimitedPointX = 0.0;
  double mixedMovingRigidLimitedPointX = 0.0;
};

ExternalSurfaceCcdDiagnostics runExternalSurfaceCcdDiagnostics()
{
  ExternalSurfaceCcdDiagnostics diagnostics;

  {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.1);
    auto moving = world.addDeformableBody(
        "moving", makePlan083InterBodyMovingPointOptions());
    world.addDeformableBody(
        "obstacle", makePlan083StationaryTriangleObstacleOptions());
    world.step();
    diagnostics.interBody = world.getLastDeformableSolverDiagnostics();
    diagnostics.interBodyLimitedPointZ = moving.getPosition(3).z();
  }

  {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.1);
    addPlan083StaticSurfaceCcdBox(
        world,
        "static_box",
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d(0.05, 1.0, 1.0));
    auto body = world.addDeformableBody(
        "fast_point",
        makePlan083SingleNodeBodyOptions(
            Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(20.0, 0.0, 0.0)));
    world.step();
    diagnostics.staticRigid = world.getLastDeformableSolverDiagnostics();
    diagnostics.staticRigidLimitedPointX = body.getPosition(0).x();
  }

  {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.1);
    addPlan083MovingSurfaceCcdBox(
        world,
        "moving_box",
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.2, 1.0, 1.0),
        Eigen::Vector3d(-1.0, 0.0, 0.0));
    auto body = world.addDeformableBody(
        "approaching_point",
        makePlan083SingleNodeBodyOptions(
            Eigen::Vector3d(-0.5, 0.0, 0.0), Eigen::Vector3d(15.0, 0.0, 0.0)));
    world.step();
    diagnostics.movingRigid = world.getLastDeformableSolverDiagnostics();
    diagnostics.movingRigidLimitedPointX = body.getPosition(0).x();
  }

  {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.1);
    auto interBodyMoving = world.addDeformableBody(
        "mixed_inter_body_moving", makePlan083InterBodyMovingPointOptions());
    world.addDeformableBody(
        "mixed_inter_body_obstacle",
        makePlan083StationaryTriangleObstacleOptions());
    addPlan083StaticSurfaceCcdBox(
        world,
        "mixed_static_box",
        Eigen::Vector3d(0.0, 4.0, 0.0),
        Eigen::Vector3d(0.05, 1.0, 1.0));
    auto staticPoint = world.addDeformableBody(
        "mixed_static_fast_point",
        makePlan083SingleNodeBodyOptions(
            Eigen::Vector3d(-1.0, 4.0, 0.0), Eigen::Vector3d(20.0, 0.0, 0.0)));
    addPlan083MovingSurfaceCcdBox(
        world,
        "mixed_moving_box",
        Eigen::Vector3d(1.0, 8.0, 0.0),
        Eigen::Vector3d(0.2, 1.0, 1.0),
        Eigen::Vector3d(-1.0, 0.0, 0.0));
    auto movingPoint = world.addDeformableBody(
        "mixed_moving_fast_point",
        makePlan083SingleNodeBodyOptions(
            Eigen::Vector3d(-0.5, 8.0, 0.0), Eigen::Vector3d(15.0, 0.0, 0.0)));
    world.step();
    diagnostics.mixed = world.getLastDeformableSolverDiagnostics();
    diagnostics.mixedInterBodyLimitedPointZ
        = interBodyMoving.getPosition(3).z();
    diagnostics.mixedStaticRigidLimitedPointX = staticPoint.getPosition(0).x();
    diagnostics.mixedMovingRigidLimitedPointX = movingPoint.getPosition(0).x();
  }

  return diagnostics;
}

bool hasRequiredExternalSurfaceCcdEvidence(
    const ExternalSurfaceCcdDiagnostics& diagnostics)
{
  const auto& interBody = diagnostics.interBody;
  const auto& staticRigid = diagnostics.staticRigid;
  const auto& movingRigid = diagnostics.movingRigid;
  const auto& mixed = diagnostics.mixed;

  return interBody.interBodySurfaceContactCandidateBuilds > 0u
         && interBody.interBodySurfaceContactPointTriangleCandidates > 0u
         && interBody.interBodySurfaceContactCcdPointTriangleChecks > 0u
         && interBody.interBodySurfaceContactCcdHits > 0u
         && interBody.interBodySurfaceContactCcdLimitedSteps > 0u
         && diagnostics.interBodyLimitedPointZ > 0.0
         && diagnostics.interBodyLimitedPointZ < 1.0
         && staticRigid.staticRigidSurfaceCcdSnapshotBuilds > 0u
         && staticRigid.staticRigidSurfaceCcdBoxCount == 1u
         && staticRigid.staticRigidSurfaceCcdTriangleCount == 12u
         && staticRigid.staticRigidSurfaceCcdEdgeCount == 12u
         && staticRigid.staticRigidSurfaceCcdCandidateBuilds > 0u
         && staticRigid.staticRigidSurfaceCcdPointTriangleCandidates > 0u
         && staticRigid.staticRigidSurfaceCcdPointTriangleChecks > 0u
         && staticRigid.staticRigidSurfaceCcdHits > 0u
         && staticRigid.staticRigidSurfaceCcdLimitedSteps > 0u
         && diagnostics.staticRigidLimitedPointX > -1.0
         && diagnostics.staticRigidLimitedPointX < -0.05
         && movingRigid.movingRigidSurfaceCcdSnapshotBuilds > 0u
         && movingRigid.movingRigidSurfaceCcdBoxCount == 1u
         && movingRigid.movingRigidSurfaceCcdSampleCount >= 2u
         && movingRigid.movingRigidSurfaceCcdTriangleCount > 0u
         && movingRigid.movingRigidSurfaceCcdEdgeCount > 0u
         && movingRigid.movingRigidSurfaceCcdCandidateBuilds > 0u
         && movingRigid.movingRigidSurfaceCcdPointTriangleCandidates > 0u
         && movingRigid.movingRigidSurfaceCcdPointTriangleChecks > 0u
         && movingRigid.movingRigidSurfaceCcdHits > 0u
         && movingRigid.movingRigidSurfaceCcdLimitedSteps > 0u
         && diagnostics.movingRigidLimitedPointX > -0.5
         && diagnostics.movingRigidLimitedPointX < 0.7
         && mixed.interBodySurfaceContactCandidateBuilds > 0u
         && mixed.interBodySurfaceContactPointTriangleCandidates > 0u
         && mixed.interBodySurfaceContactCcdPointTriangleChecks > 0u
         && mixed.interBodySurfaceContactCcdHits > 0u
         && mixed.interBodySurfaceContactCcdLimitedSteps > 0u
         && diagnostics.mixedInterBodyLimitedPointZ > 0.0
         && diagnostics.mixedInterBodyLimitedPointZ < 1.0
         && mixed.staticRigidSurfaceCcdSnapshotBuilds > 0u
         && mixed.staticRigidSurfaceCcdBoxCount == 1u
         && mixed.staticRigidSurfaceCcdCandidateBuilds > 0u
         && mixed.staticRigidSurfaceCcdPointTriangleCandidates > 0u
         && mixed.staticRigidSurfaceCcdPointTriangleChecks > 0u
         && mixed.staticRigidSurfaceCcdHits > 0u
         && mixed.staticRigidSurfaceCcdLimitedSteps > 0u
         && diagnostics.mixedStaticRigidLimitedPointX > -1.0
         && diagnostics.mixedStaticRigidLimitedPointX < -0.05
         && mixed.movingRigidSurfaceCcdSnapshotBuilds > 0u
         && mixed.movingRigidSurfaceCcdBoxCount == 1u
         && mixed.movingRigidSurfaceCcdSampleCount >= 2u
         && mixed.movingRigidSurfaceCcdCandidateBuilds > 0u
         && mixed.movingRigidSurfaceCcdPointTriangleCandidates > 0u
         && mixed.movingRigidSurfaceCcdPointTriangleChecks > 0u
         && mixed.movingRigidSurfaceCcdHits > 0u
         && mixed.movingRigidSurfaceCcdLimitedSteps > 0u
         && diagnostics.mixedMovingRigidLimitedPointX > -0.5
         && diagnostics.mixedMovingRigidLimitedPointX < 0.7;
}

//==============================================================================
static void BM_Plan083CpuScene_external_surface_ccd_diagnostics(
    benchmark::State& state)
{
  std::size_t failedSteps = 0;
  ExternalSurfaceCcdDiagnostics lastDiagnostics;
  for (auto _ : state) {
    lastDiagnostics = runExternalSurfaceCcdDiagnostics();
    if (!hasRequiredExternalSurfaceCcdEvidence(lastDiagnostics)) {
      ++failedSteps;
    }
    benchmark::DoNotOptimize(lastDiagnostics.interBodyLimitedPointZ);
    benchmark::DoNotOptimize(lastDiagnostics.staticRigidLimitedPointX);
    benchmark::DoNotOptimize(lastDiagnostics.movingRigidLimitedPointX);
    benchmark::ClobberMemory();
  }

  const std::array<sx::DeformableSolverDiagnostics, 4> allDiagnostics{
      lastDiagnostics.interBody,
      lastDiagnostics.staticRigid,
      lastDiagnostics.movingRigid,
      lastDiagnostics.mixed};

  state.counters["row_unb_alg_barriers"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["external_surface_ccd_scene_count"] = 4.0;
  state.counters["mixed_external_surface_ccd_scene_count"] = 1.0;
  state.counters["mixed_external_surface_ccd_family_count"] = 3.0;
  state.counters["rigid_obstacle_count"] = 4.0;
  state.counters["static_rigid_obstacle_count"] = 2.0;
  state.counters["moving_rigid_obstacle_count"] = 2.0;
  state.counters["deformable_body_count"]
      = static_cast<double>(sumDiagnosticsMember(
          allDiagnostics, &sx::DeformableSolverDiagnostics::bodyCount));
  state.counters["deformable_node_count"]
      = static_cast<double>(sumDiagnosticsMember(
          allDiagnostics, &sx::DeformableSolverDiagnostics::nodeCount));
  state.counters["deformable_edge_count"]
      = static_cast<double>(sumDiagnosticsMember(
          allDiagnostics, &sx::DeformableSolverDiagnostics::edgeCount));
  state.counters["surface_triangle_count"] = 4.0;
  recordSummedDeformableRuntimeContactCounters(state, allDiagnostics);
  state.counters["mixed_inter_body_surface_contact_candidate_builds"]
      = static_cast<double>(
          lastDiagnostics.mixed.interBodySurfaceContactCandidateBuilds);
  state.counters["mixed_inter_body_surface_contact_point_triangle_candidates"]
      = static_cast<double>(
          lastDiagnostics.mixed.interBodySurfaceContactPointTriangleCandidates);
  state.counters["mixed_inter_body_surface_contact_ccd_point_triangle_checks"]
      = static_cast<double>(
          lastDiagnostics.mixed.interBodySurfaceContactCcdPointTriangleChecks);
  state.counters["mixed_inter_body_surface_contact_ccd_hits"]
      = static_cast<double>(
          lastDiagnostics.mixed.interBodySurfaceContactCcdHits);
  state.counters["mixed_inter_body_surface_contact_ccd_limited_steps"]
      = static_cast<double>(
          lastDiagnostics.mixed.interBodySurfaceContactCcdLimitedSteps);
  state.counters["mixed_static_rigid_surface_ccd_candidate_builds"]
      = static_cast<double>(
          lastDiagnostics.mixed.staticRigidSurfaceCcdCandidateBuilds);
  state.counters["mixed_static_rigid_surface_ccd_point_triangle_candidates"]
      = static_cast<double>(
          lastDiagnostics.mixed.staticRigidSurfaceCcdPointTriangleCandidates);
  state.counters["mixed_static_rigid_surface_ccd_point_triangle_checks"]
      = static_cast<double>(
          lastDiagnostics.mixed.staticRigidSurfaceCcdPointTriangleChecks);
  state.counters["mixed_static_rigid_surface_ccd_hits"]
      = static_cast<double>(lastDiagnostics.mixed.staticRigidSurfaceCcdHits);
  state.counters["mixed_static_rigid_surface_ccd_limited_steps"]
      = static_cast<double>(
          lastDiagnostics.mixed.staticRigidSurfaceCcdLimitedSteps);
  state.counters["mixed_moving_rigid_surface_ccd_candidate_builds"]
      = static_cast<double>(
          lastDiagnostics.mixed.movingRigidSurfaceCcdCandidateBuilds);
  state.counters["mixed_moving_rigid_surface_ccd_point_triangle_candidates"]
      = static_cast<double>(
          lastDiagnostics.mixed.movingRigidSurfaceCcdPointTriangleCandidates);
  state.counters["mixed_moving_rigid_surface_ccd_point_triangle_checks"]
      = static_cast<double>(
          lastDiagnostics.mixed.movingRigidSurfaceCcdPointTriangleChecks);
  state.counters["mixed_moving_rigid_surface_ccd_hits"]
      = static_cast<double>(lastDiagnostics.mixed.movingRigidSurfaceCcdHits);
  state.counters["mixed_moving_rigid_surface_ccd_limited_steps"]
      = static_cast<double>(
          lastDiagnostics.mixed.movingRigidSurfaceCcdLimitedSteps);
  state.counters["failed_steps"] = static_cast<double>(failedSteps);
}
BENCHMARK(BM_Plan083CpuScene_external_surface_ccd_diagnostics)
    ->Unit(benchmark::kMillisecond);

//==============================================================================
static void BM_Plan083CpuScene_lying_flat_reduced_world_step(
    benchmark::State& state)
{
  LyingFlatFixture fixture;
  sx::compute::SequentialExecutor executor;

  std::size_t failedSteps = 0;
  sx::DeformableSolverDiagnostics lastDiagnostics;
  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    state.ResumeTiming();

    fixture.world.step(executor);
    lastDiagnostics = fixture.world.getLastDeformableSolverDiagnostics();
    if (lastDiagnostics.bodyCount != 1u || lastDiagnostics.nodeCount == 0u
        || !std::isfinite(fixture.minClothHeight())) {
      ++failedSteps;
    }
    double firstNodeHeight = fixture.cloth->getPosition(0).z();
    double lastNodeHeight
        = fixture.cloth->getPosition(fixture.cloth->getNodeCount() - 1u).z();
    benchmark::DoNotOptimize(firstNodeHeight);
    benchmark::DoNotOptimize(lastNodeHeight);
    benchmark::ClobberMemory();
  }

  state.counters["row_unb_fig_01"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["rigid_obstacle_count"]
      = static_cast<double>(fixture.obstacles.size());
  state.counters["deformable_body_count"]
      = static_cast<double>(lastDiagnostics.bodyCount);
  state.counters["deformable_node_count"]
      = static_cast<double>(lastDiagnostics.nodeCount);
  state.counters["deformable_edge_count"]
      = static_cast<double>(lastDiagnostics.edgeCount);
  state.counters["surface_triangle_count"]
      = static_cast<double>(fixture.cloth->getSurfaceTriangleCount());
  state.counters["solver_iterations"]
      = static_cast<double>(lastDiagnostics.solverIterations);
  state.counters["active_contact_count"]
      = static_cast<double>(lastDiagnostics.convergedActiveContactCount);
  state.counters["friction_dissipation"] = lastDiagnostics.frictionDissipation;
  state.counters["min_active_contact_distance_m"]
      = lastDiagnostics.minActiveContactDistance;
  recordDeformableRuntimeContactCounters(state, lastDiagnostics);
  state.counters["min_cloth_height_m"] = fixture.minClothHeight();
  state.counters["cloth_span_x_m"] = fixture.clothSpanX();
  state.counters["cloth_span_y_m"] = fixture.clothSpanY();
  state.counters["failed_steps"] = static_cast<double>(failedSteps);
}
BENCHMARK(BM_Plan083CpuScene_lying_flat_reduced_world_step)
    ->Unit(benchmark::kMillisecond);

//==============================================================================
static void BM_Plan083CpuScene_candy_reduced_world_step(benchmark::State& state)
{
  CandyFixture fixture;
  sx::compute::SequentialExecutor executor;

  std::size_t failedSteps = 0;
  sx::DeformableSolverDiagnostics lastDiagnostics;
  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    state.ResumeTiming();

    fixture.world.step(executor);
    lastDiagnostics = fixture.world.getLastDeformableSolverDiagnostics();
    if (lastDiagnostics.bodyCount != 1u || lastDiagnostics.nodeCount == 0u
        || !std::isfinite(fixture.minClothHeight())) {
      ++failedSteps;
    }
    double firstNodeHeight = fixture.cloth->getPosition(0).z();
    double lastNodeHeight
        = fixture.cloth->getPosition(fixture.cloth->getNodeCount() - 1u).z();
    benchmark::DoNotOptimize(firstNodeHeight);
    benchmark::DoNotOptimize(lastNodeHeight);
    benchmark::ClobberMemory();
  }

  state.counters["row_unb_fig_22"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["rigid_obstacle_count"] = 1.0;
  state.counters["deformable_body_count"]
      = static_cast<double>(lastDiagnostics.bodyCount);
  state.counters["deformable_node_count"]
      = static_cast<double>(lastDiagnostics.nodeCount);
  state.counters["deformable_edge_count"]
      = static_cast<double>(lastDiagnostics.edgeCount);
  state.counters["surface_triangle_count"]
      = static_cast<double>(fixture.cloth->getSurfaceTriangleCount());
  state.counters["solver_iterations"]
      = static_cast<double>(lastDiagnostics.solverIterations);
  state.counters["active_contact_count"]
      = static_cast<double>(lastDiagnostics.convergedActiveContactCount);
  state.counters["friction_dissipation"] = lastDiagnostics.frictionDissipation;
  state.counters["min_active_contact_distance_m"]
      = lastDiagnostics.minActiveContactDistance;
  recordDeformableRuntimeContactCounters(state, lastDiagnostics);
  state.counters["min_cloth_height_m"] = fixture.minClothHeight();
  state.counters["cloth_span_x_m"] = fixture.clothSpanX();
  state.counters["failed_steps"] = static_cast<double>(failedSteps);
}
BENCHMARK(BM_Plan083CpuScene_candy_reduced_world_step)
    ->Unit(benchmark::kMillisecond);

//==============================================================================
static void BM_Plan083CpuScene_nunchaku_single_reduced_world_step(
    benchmark::State& state)
{
  NunchakuFixture fixture;
  sx::compute::SequentialExecutor executor;

  std::size_t failedSteps = 0;
  sx::compute::RigidIpcSolverStats lastStats;
  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    sx::compute::RigidIpcContactStage ipcStage(64);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(ipcStage);
    state.ResumeTiming();

    fixture.world.step(executor, pipeline);
    lastStats = ipcStage.getLastStats();
    const bool residualOk
        = std::isfinite(lastStats.finalEqualityResidualNorm)
          && lastStats.finalEqualityResidualNorm <= kMaxReducedEqualityResidual;
    const bool satisfiedNoOp = residualOk && lastStats.solverIterations == 0u
                               && lastStats.activeArticulationConstraints >= 2u;
    if (!residualOk || (lastStats.failed && !satisfiedNoOp)) {
      ++failedSteps;
    }
    benchmark::DoNotOptimize(fixture.swinging->getTransform().data());
    benchmark::ClobberMemory();
  }

  state.counters["row_unb_fig_13"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["body_count"] = static_cast<double>(lastStats.bodyCount);
  state.counters["dynamic_body_count"]
      = static_cast<double>(lastStats.dynamicBodyCount);
  state.counters["revolute_joint_count"]
      = static_cast<double>(fixture.world.getRigidBodyJointCount());
  state.counters["active_articulation_constraints"]
      = static_cast<double>(lastStats.activeArticulationConstraints);
  state.counters["solver_iterations"]
      = static_cast<double>(lastStats.solverIterations);
  state.counters["failed_steps"] = static_cast<double>(failedSteps);
  state.counters["final_equality_residual_norm"]
      = lastStats.finalEqualityResidualNorm;
  state.counters["swinging_tip_radius_m"] = fixture.swingingTipRadius();
  state.counters["free_axis_angular_velocity_rad_s"]
      = fixture.swinging->getAngularVelocity().z();
}
BENCHMARK(BM_Plan083CpuScene_nunchaku_single_reduced_world_step)
    ->Unit(benchmark::kMillisecond);

//==============================================================================
static void BM_Plan083CpuScene_nunchaku_scaling_reduced_world_step(
    benchmark::State& state)
{
  const auto pairCount = static_cast<std::size_t>(state.range(0));
  NunchakuScalingFixture fixture(pairCount);
  sx::compute::SequentialExecutor executor;

  std::size_t failedSteps = 0;
  sx::compute::RigidIpcSolverStats lastStats;
  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    sx::compute::RigidIpcContactStage ipcStage(64);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(ipcStage);
    state.ResumeTiming();

    fixture.world.step(executor, pipeline);
    lastStats = ipcStage.getLastStats();
    const bool residualOk
        = std::isfinite(lastStats.finalEqualityResidualNorm)
          && lastStats.finalEqualityResidualNorm <= kMaxReducedEqualityResidual;
    const bool satisfiedNoOp
        = residualOk && lastStats.solverIterations == 0u
          && lastStats.activeArticulationConstraints >= 2u * pairCount;
    if (!residualOk || (lastStats.failed && !satisfiedNoOp)) {
      ++failedSteps;
    }
    benchmark::DoNotOptimize(fixture.swinging.front().getTransform().data());
    benchmark::DoNotOptimize(fixture.swinging.back().getTransform().data());
    benchmark::ClobberMemory();
  }

  state.counters["row_unb_fig_25"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["nunchaku_pair_count"] = static_cast<double>(pairCount);
  state.counters["body_count"] = static_cast<double>(lastStats.bodyCount);
  state.counters["dynamic_body_count"]
      = static_cast<double>(lastStats.dynamicBodyCount);
  state.counters["revolute_joint_count"]
      = static_cast<double>(fixture.world.getRigidBodyJointCount());
  state.counters["active_articulation_constraints"]
      = static_cast<double>(lastStats.activeArticulationConstraints);
  state.counters["solver_iterations"]
      = static_cast<double>(lastStats.solverIterations);
  state.counters["failed_steps"] = static_cast<double>(failedSteps);
  state.counters["final_equality_residual_norm"]
      = lastStats.finalEqualityResidualNorm;
  state.counters["free_axis_angular_velocity_rad_s"]
      = fixture.swinging.front().getAngularVelocity().z();
}
BENCHMARK(BM_Plan083CpuScene_nunchaku_scaling_reduced_world_step)
    ->Arg(20)
    ->Arg(40)
    ->Arg(60)
    ->Arg(80)
    ->Arg(100)
    ->Unit(benchmark::kMillisecond);

//==============================================================================
static void BM_Plan083CpuScene_windmill_reduced_world_step(
    benchmark::State& state)
{
  WindmillFixture fixture;
  sx::compute::SequentialExecutor executor;

  std::size_t failedSteps = 0;
  sx::compute::RigidIpcSolverStats lastStats;
  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    sx::compute::RigidIpcContactStage ipcStage(64);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(ipcStage);
    state.ResumeTiming();

    fixture.world.step(executor, pipeline);
    lastStats = ipcStage.getLastStats();
    const bool residualOk
        = std::isfinite(lastStats.finalEqualityResidualNorm)
          && lastStats.finalEqualityResidualNorm <= kMaxReducedEqualityResidual;
    const bool satisfiedNoOp = residualOk && lastStats.solverIterations == 0u
                               && lastStats.activeArticulationConstraints >= 2u;
    if (!residualOk || (lastStats.failed && !satisfiedNoOp)) {
      ++failedSteps;
    }
    benchmark::DoNotOptimize(fixture.blade->getTransform().data());
    benchmark::DoNotOptimize(fixture.striker->getTranslation().data());
    benchmark::ClobberMemory();
  }

  state.counters["row_unb_fig_20"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["body_count"] = static_cast<double>(lastStats.bodyCount);
  state.counters["dynamic_body_count"]
      = static_cast<double>(lastStats.dynamicBodyCount);
  state.counters["revolute_joint_count"]
      = static_cast<double>(fixture.world.getRigidBodyJointCount());
  state.counters["active_constraints"]
      = static_cast<double>(lastStats.activeConstraints);
  state.counters["active_friction_constraints"]
      = static_cast<double>(lastStats.activeFrictionConstraints);
  state.counters["active_articulation_constraints"]
      = static_cast<double>(lastStats.activeArticulationConstraints);
  state.counters["solver_iterations"]
      = static_cast<double>(lastStats.solverIterations);
  state.counters["failed_steps"] = static_cast<double>(failedSteps);
  state.counters["final_equality_residual_norm"]
      = lastStats.finalEqualityResidualNorm;
  state.counters["blade_tip_radius_m"] = fixture.bladeTipRadius();
  state.counters["striker_height_m"] = fixture.striker->getTranslation().z();
  state.counters["striker_blade_clearance_m"] = fixture.strikerBladeClearance();
}
BENCHMARK(BM_Plan083CpuScene_windmill_reduced_world_step)
    ->Unit(benchmark::kMillisecond);

//==============================================================================
static void BM_Plan083CpuScene_terrain_vehicle_reduced_world_step(
    benchmark::State& state)
{
  TerrainVehicleFixture fixture;
  sx::compute::SequentialExecutor executor;

  std::size_t failedSteps = 0;
  sx::compute::RigidIpcSolverStats lastStats;
  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    sx::compute::RigidIpcContactStage ipcStage(64);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(ipcStage);
    state.ResumeTiming();

    fixture.world.step(executor, pipeline);
    lastStats = ipcStage.getLastStats();
    const bool residualOk
        = std::isfinite(lastStats.finalEqualityResidualNorm)
          && lastStats.finalEqualityResidualNorm <= kMaxReducedEqualityResidual;
    const bool satisfiedNoOp = residualOk && lastStats.solverIterations == 0u
                               && lastStats.activeArticulationConstraints >= 8u;
    if (!residualOk || (lastStats.failed && !satisfiedNoOp)) {
      ++failedSteps;
    }
    benchmark::DoNotOptimize(fixture.chassis->getTranslation().data());
    benchmark::DoNotOptimize(fixture.wheels.front().getTransform().data());
    benchmark::ClobberMemory();
  }

  state.counters["row_unb_fig_10"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["body_count"] = static_cast<double>(lastStats.bodyCount);
  state.counters["dynamic_body_count"]
      = static_cast<double>(lastStats.dynamicBodyCount);
  state.counters["wheel_count"] = static_cast<double>(fixture.wheels.size());
  state.counters["revolute_joint_count"]
      = static_cast<double>(fixture.world.getRigidBodyJointCount());
  state.counters["active_constraints"]
      = static_cast<double>(lastStats.activeConstraints);
  state.counters["active_friction_constraints"]
      = static_cast<double>(lastStats.activeFrictionConstraints);
  state.counters["active_articulation_constraints"]
      = static_cast<double>(lastStats.activeArticulationConstraints);
  state.counters["solver_iterations"]
      = static_cast<double>(lastStats.solverIterations);
  state.counters["failed_steps"] = static_cast<double>(failedSteps);
  state.counters["final_equality_residual_norm"]
      = lastStats.finalEqualityResidualNorm;
  state.counters["chassis_height_m"] = fixture.chassis->getTranslation().z();
  state.counters["min_wheel_ground_clearance_m"]
      = fixture.minWheelGroundClearance();
}
BENCHMARK(BM_Plan083CpuScene_terrain_vehicle_reduced_world_step)
    ->Unit(benchmark::kMillisecond);

//==============================================================================
static void BM_Plan083CpuScene_precession_reduced_world_step(
    benchmark::State& state)
{
  PrecessionFixture fixture;
  sx::compute::SequentialExecutor executor;

  std::size_t failedSteps = 0;
  sx::compute::RigidIpcSolverStats lastStats;
  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    sx::compute::RigidIpcContactStage ipcStage(64);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(ipcStage);
    state.ResumeTiming();

    fixture.world.step(executor, pipeline);
    lastStats = ipcStage.getLastStats();
    const bool residualOk
        = std::isfinite(lastStats.finalEqualityResidualNorm)
          && lastStats.finalEqualityResidualNorm <= kMaxReducedEqualityResidual;
    const bool satisfiedNoOp = residualOk && lastStats.solverIterations == 0u
                               && lastStats.activeConstraints >= 1u;
    if (!residualOk || (lastStats.failed && !satisfiedNoOp)) {
      ++failedSteps;
    }
    benchmark::DoNotOptimize(fixture.wheel->getTransform().data());
    benchmark::ClobberMemory();
  }

  state.counters["row_unb_fig_23"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["body_count"] = static_cast<double>(lastStats.bodyCount);
  state.counters["dynamic_body_count"]
      = static_cast<double>(lastStats.dynamicBodyCount);
  state.counters["active_constraints"]
      = static_cast<double>(lastStats.activeConstraints);
  state.counters["active_friction_constraints"]
      = static_cast<double>(lastStats.activeFrictionConstraints);
  state.counters["solver_iterations"]
      = static_cast<double>(lastStats.solverIterations);
  state.counters["failed_steps"] = static_cast<double>(failedSteps);
  state.counters["final_equality_residual_norm"]
      = lastStats.finalEqualityResidualNorm;
  state.counters["wheel_height_m"] = fixture.wheel->getTranslation().z();
  state.counters["wheel_ground_clearance_m"] = fixture.wheelGroundClearance();
  state.counters["spin_rate_rad_s"]
      = fixture.wheel->getAngularVelocity().norm();
}
BENCHMARK(BM_Plan083CpuScene_precession_reduced_world_step)
    ->Unit(benchmark::kMillisecond);

//==============================================================================
static void BM_Plan083CpuScene_ragdoll_reduced_world_step(
    benchmark::State& state)
{
  RagdollFixture fixture;
  sx::compute::SequentialExecutor executor;

  std::size_t failedSteps = 0;
  sx::compute::RigidIpcSolverStats lastStats;
  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    sx::compute::RigidIpcContactStage ipcStage(64);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(ipcStage);
    state.ResumeTiming();

    fixture.world.step(executor, pipeline);
    lastStats = ipcStage.getLastStats();
    const bool residualOk
        = std::isfinite(lastStats.finalEqualityResidualNorm)
          && lastStats.finalEqualityResidualNorm <= kMaxReducedEqualityResidual;
    const bool satisfiedNoOp = residualOk && lastStats.solverIterations == 0u
                               && lastStats.activeArticulationConstraints >= 10u
                               && lastStats.activeConstraints >= 1u;
    if (!residualOk || (lastStats.failed && !satisfiedNoOp)) {
      ++failedSteps;
    }
    benchmark::DoNotOptimize(fixture.torso->getTransform().data());
    benchmark::DoNotOptimize(fixture.parts.front().getTransform().data());
    benchmark::ClobberMemory();
  }

  state.counters["row_unb_fig_11"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["body_count"] = static_cast<double>(lastStats.bodyCount);
  state.counters["dynamic_body_count"]
      = static_cast<double>(lastStats.dynamicBodyCount);
  state.counters["ragdoll_body_count"]
      = static_cast<double>(1 + fixture.parts.size());
  state.counters["revolute_joint_count"]
      = static_cast<double>(fixture.world.getRigidBodyJointCount());
  state.counters["active_constraints"]
      = static_cast<double>(lastStats.activeConstraints);
  state.counters["active_friction_constraints"]
      = static_cast<double>(lastStats.activeFrictionConstraints);
  state.counters["active_articulation_constraints"]
      = static_cast<double>(lastStats.activeArticulationConstraints);
  state.counters["solver_iterations"]
      = static_cast<double>(lastStats.solverIterations);
  state.counters["failed_steps"] = static_cast<double>(failedSteps);
  state.counters["final_equality_residual_norm"]
      = lastStats.finalEqualityResidualNorm;
  state.counters["torso_height_m"] = fixture.torso->getTranslation().z();
  state.counters["min_leg_ground_clearance_m"]
      = fixture.minLegGroundClearance();
}
BENCHMARK(BM_Plan083CpuScene_ragdoll_reduced_world_step)
    ->Unit(benchmark::kMillisecond);

//==============================================================================
static void BM_Plan083CpuScene_abd_wrecking_ball_reduced_pair_runtime_step(
    benchmark::State& state)
{
  AbdWreckingBallFixture fixture;

  sxdetail::AffinePointTrianglePairRuntimeStepResult result;
  for (auto _ : state) {
    result = fixture.step();
    benchmark::DoNotOptimize(result.solve.pointState.translation.data());
    benchmark::DoNotOptimize(result.solve.triangleState.translation.data());
    benchmark::DoNotOptimize(result.solve.pointState.linearMap.data());
    benchmark::DoNotOptimize(result.solve.triangleState.linearMap.data());
    benchmark::ClobberMemory();
  }

  const bool stepFailed
      = !result.valid || !result.converged || !result.solve.barrierActive;
  const auto targetBarrier = sxdetail::affinePointTriangleBarrier(
      result.pointInertialTarget,
      fixture.point,
      result.triangleInertialTarget,
      fixture.triangleA,
      fixture.triangleB,
      fixture.triangleC,
      fixture.options.solve.barrier);

  state.counters["row_abd_vs_rigid_wreck"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["affine_body_count"] = 2.0;
  state.counters["dynamic_pair_count"]
      = static_cast<double>(kAbdReducedWreckingBallPairCount);
  state.counters["valid_step_count"] = result.valid ? 1.0 : 0.0;
  state.counters["failed_steps"] = stepFailed ? 1.0 : 0.0;
  state.counters["converged_solve_count"] = result.converged ? 1.0 : 0.0;
  state.counters["barrier_active_count"]
      = result.solve.barrierActive ? 1.0 : 0.0;
  state.counters["solver_iterations"]
      = static_cast<double>(result.solve.iterations);
  state.counters["total_objective_decrease"]
      = result.solve.initialValue - result.solve.finalValue;
  state.counters["max_final_gradient_norm"] = result.solve.finalGradientNorm;
  state.counters["min_target_squared_distance"]
      = targetBarrier.primitive.squaredDistance;
  state.counters["min_final_squared_distance"]
      = result.solve.finalSquaredDistance;
  state.counters["squared_activation_distance"]
      = fixture.options.solve.barrier.squaredActivationDistance;
  state.counters["max_linear_speed_m_s"] = result.maxLinearSpeed;
  state.counters["max_affine_velocity_norm"] = result.maxAffineVelocityNorm;
  state.counters["max_displacement_norm_m"]
      = std::max(result.pointDisplacementNorm, result.triangleDisplacementNorm);
}
BENCHMARK(BM_Plan083CpuScene_abd_wrecking_ball_reduced_pair_runtime_step)
    ->Unit(benchmark::kMillisecond);

//==============================================================================
static void runAbdChainNetReducedPairRuntimeStep(
    benchmark::State& state,
    const std::size_t pairCount,
    const char* rowCounterName)
{
  AbdChainNetFixture fixture(pairCount);

  std::vector<sxdetail::AffinePointTrianglePairRuntimeStepResult> lastResults;
  for (auto _ : state) {
    lastResults = fixture.stepAll();
    for (const auto& result : lastResults) {
      benchmark::DoNotOptimize(result.solve.pointState.translation.data());
      benchmark::DoNotOptimize(result.solve.triangleState.translation.data());
      benchmark::DoNotOptimize(result.solve.pointState.linearMap.data());
      benchmark::DoNotOptimize(result.solve.triangleState.linearMap.data());
    }
    benchmark::ClobberMemory();
  }

  std::size_t convergedSolveCount = 0;
  std::size_t barrierActiveCount = 0;
  std::size_t validStepCount = 0;
  std::size_t failedStepCount = 0;
  std::size_t solverIterations = 0;
  double totalObjectiveDecrease = 0.0;
  double maxFinalGradientNorm = 0.0;
  double minTargetSquaredDistance = std::numeric_limits<double>::infinity();
  double minFinalSquaredDistance = std::numeric_limits<double>::infinity();
  double maxLinearSpeed = 0.0;
  double maxAffineVelocityNorm = 0.0;
  double maxDisplacementNorm = 0.0;

  for (std::size_t index = 0; index < lastResults.size(); ++index) {
    const auto& result = lastResults[index];
    const auto& pair = fixture.pairs[index];
    if (result.valid) {
      ++validStepCount;
    }
    if (result.converged) {
      ++convergedSolveCount;
    }
    if (result.solve.barrierActive) {
      ++barrierActiveCount;
    }
    if (!result.valid || !result.converged || !result.solve.barrierActive) {
      ++failedStepCount;
    }
    solverIterations += static_cast<std::size_t>(result.solve.iterations);
    totalObjectiveDecrease
        += result.solve.initialValue - result.solve.finalValue;
    maxFinalGradientNorm
        = std::max(maxFinalGradientNorm, result.solve.finalGradientNorm);
    maxLinearSpeed = std::max(maxLinearSpeed, result.maxLinearSpeed);
    maxAffineVelocityNorm
        = std::max(maxAffineVelocityNorm, result.maxAffineVelocityNorm);
    maxDisplacementNorm = std::max(
        maxDisplacementNorm,
        std::max(
            result.pointDisplacementNorm, result.triangleDisplacementNorm));
    const auto targetBarrier = sxdetail::affinePointTriangleBarrier(
        result.pointInertialTarget,
        pair.point,
        result.triangleInertialTarget,
        pair.triangleA,
        pair.triangleB,
        pair.triangleC,
        fixture.options.solve.barrier);
    minTargetSquaredDistance = std::min(
        minTargetSquaredDistance, targetBarrier.primitive.squaredDistance);
    minFinalSquaredDistance
        = std::min(minFinalSquaredDistance, result.solve.finalSquaredDistance);
  }

  state.counters[rowCounterName] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["affine_body_count"] = static_cast<double>(2u * pairCount);
  state.counters["dynamic_pair_count"] = static_cast<double>(pairCount);
  state.counters["valid_step_count"] = static_cast<double>(validStepCount);
  state.counters["failed_steps"] = static_cast<double>(failedStepCount);
  state.counters["converged_solve_count"]
      = static_cast<double>(convergedSolveCount);
  state.counters["barrier_active_count"]
      = static_cast<double>(barrierActiveCount);
  state.counters["solver_iterations"] = static_cast<double>(solverIterations);
  state.counters["total_objective_decrease"] = totalObjectiveDecrease;
  state.counters["max_final_gradient_norm"] = maxFinalGradientNorm;
  state.counters["min_target_squared_distance"] = minTargetSquaredDistance;
  state.counters["min_final_squared_distance"] = minFinalSquaredDistance;
  state.counters["squared_activation_distance"]
      = fixture.options.solve.barrier.squaredActivationDistance;
  state.counters["max_linear_speed_m_s"] = maxLinearSpeed;
  state.counters["max_affine_velocity_norm"] = maxAffineVelocityNorm;
  state.counters["max_displacement_norm_m"] = maxDisplacementNorm;
}

static void BM_Plan083CpuScene_abd_chain_8_reduced_pair_runtime_step(
    benchmark::State& state)
{
  runAbdChainNetReducedPairRuntimeStep(
      state, kAbdReducedChain8PairCount, "row_abd_chain_8");
}
BENCHMARK(BM_Plan083CpuScene_abd_chain_8_reduced_pair_runtime_step)
    ->Unit(benchmark::kMillisecond);

static void BM_Plan083CpuScene_abd_chain_16_reduced_pair_runtime_step(
    benchmark::State& state)
{
  runAbdChainNetReducedPairRuntimeStep(
      state, kAbdReducedChain16PairCount, "row_abd_chain_16");
}
BENCHMARK(BM_Plan083CpuScene_abd_chain_16_reduced_pair_runtime_step)
    ->Unit(benchmark::kMillisecond);

static void BM_Plan083CpuScene_abd_chain_96_reduced_pair_runtime_step(
    benchmark::State& state)
{
  runAbdChainNetReducedPairRuntimeStep(
      state, kAbdReducedChain96PairCount, "row_abd_chain_96");
}
BENCHMARK(BM_Plan083CpuScene_abd_chain_96_reduced_pair_runtime_step)
    ->Unit(benchmark::kMillisecond);

static void runAbdComparisonReducedPairRuntimeStep(
    benchmark::State& state,
    const std::size_t pairCount,
    const char* rowCounterName,
    const double paperBodyCount,
    const double paperTriangleCount)
{
  runAbdChainNetReducedPairRuntimeStep(state, pairCount, rowCounterName);
  state.counters["reduced_pair_count"] = static_cast<double>(pairCount);
  state.counters["paper_body_count"] = paperBodyCount;
  state.counters["paper_triangle_count"] = paperTriangleCount;
  state.counters["reference_baseline_measured"] = 0.0;
}

static void BM_Plan083CpuScene_abd_gears_reduced_pair_runtime_step(
    benchmark::State& state)
{
  runAbdComparisonReducedPairRuntimeStep(
      state, kAbdReducedGearsPairCount, "row_abd_gears", 28.0, 2'500'000.0);
}
BENCHMARK(BM_Plan083CpuScene_abd_gears_reduced_pair_runtime_step)
    ->Unit(benchmark::kMillisecond);

static void BM_Plan083CpuScene_abd_bullet_small_reduced_pair_runtime_step(
    benchmark::State& state)
{
  runAbdComparisonReducedPairRuntimeStep(
      state,
      kAbdReducedBulletSmallPairCount,
      "row_abd_bullet_small",
      16.0,
      1'200.0);
}
BENCHMARK(BM_Plan083CpuScene_abd_bullet_small_reduced_pair_runtime_step)
    ->Unit(benchmark::kMillisecond);

static void BM_Plan083CpuScene_abd_bullet_medium_reduced_pair_runtime_step(
    benchmark::State& state)
{
  runAbdComparisonReducedPairRuntimeStep(
      state,
      kAbdReducedBulletMediumPairCount,
      "row_abd_bullet_medium",
      142.0,
      3'500.0);
}
BENCHMARK(BM_Plan083CpuScene_abd_bullet_medium_reduced_pair_runtime_step)
    ->Unit(benchmark::kMillisecond);

static void BM_Plan083CpuScene_abd_bullet_large_reduced_pair_runtime_step(
    benchmark::State& state)
{
  runAbdComparisonReducedPairRuntimeStep(
      state,
      kAbdReducedBulletLargePairCount,
      "row_abd_bullet_large",
      562.0,
      11'000.0);
}
BENCHMARK(BM_Plan083CpuScene_abd_bullet_large_reduced_pair_runtime_step)
    ->Unit(benchmark::kMillisecond);

static void BM_Plan083CpuScene_abd_complex_geometry_reduced_pair_runtime_step(
    benchmark::State& state)
{
  runAbdComparisonReducedPairRuntimeStep(
      state,
      kAbdReducedComplexGeometryPairCount,
      "row_abd_complex_geometry",
      29.0,
      1'200'000.0);
}
BENCHMARK(BM_Plan083CpuScene_abd_complex_geometry_reduced_pair_runtime_step)
    ->Unit(benchmark::kMillisecond);

struct ReducedAffineFemMixedDiagnostics
{
  std::size_t candidateCount = 0;
  std::size_t activeBarrierCount = 0;
  double minSquaredDistance = std::numeric_limits<double>::infinity();
  double barrierValue = 0.0;
  bool finite = false;
};

using ReducedAffineFemVector21d = Eigen::Matrix<double, 21, 1>;
using ReducedAffineFemMatrix21d = Eigen::Matrix<double, 21, 21>;

struct ReducedAffineFemCoupledEvaluation
{
  double value = 0.0;
  ReducedAffineFemVector21d gradient = ReducedAffineFemVector21d::Zero();
  ReducedAffineFemMatrix21d hessian = ReducedAffineFemMatrix21d::Zero();
  double squaredDistance = std::numeric_limits<double>::infinity();
  bool valid = false;
  bool barrierActive = false;
};

struct ReducedAffineFemCoupledSolve
{
  double initialValue = 0.0;
  double finalValue = 0.0;
  double initialGradientNorm = 0.0;
  double finalGradientNorm = 0.0;
  double initialSquaredDistance = std::numeric_limits<double>::infinity();
  double finalSquaredDistance = std::numeric_limits<double>::infinity();
  double affineDisplacementNorm = 0.0;
  double deformableDisplacementNorm = 0.0;
  int iterations = 0;
  bool valid = false;
  bool converged = false;
  bool barrierActive = false;
};

ReducedAffineFemMixedDiagnostics evaluateReducedAffineFemMixedDiagnostics(
    const LyingFlatFixture& deformableFixture)
{
  sxdetail::AffineSurfaceAdapter affineAdapter;
  affineAdapter.restVertices
      = {Eigen::Vector3d(-0.035, -0.025, 0.0),
         Eigen::Vector3d(0.045, -0.025, 0.0),
         Eigen::Vector3d(-0.035, 0.055, 0.0)};
  affineAdapter.triangles = {Eigen::Vector3i(0, 1, 2)};

  sxdetail::AffineBodyState affineState;
  affineState.translation = deformableFixture.cloth->getPosition(0)
                            + Eigen::Vector3d(0.035, 0.025, -0.018);
  affineState.linearMap
      = Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitY()).toRotationMatrix();

  std::vector<Eigen::Vector3d> affineVertices;
  affineVertices.reserve(affineAdapter.restVertices.size());
  for (std::size_t vertex = 0; vertex < affineAdapter.restVertices.size();
       ++vertex) {
    affineVertices.push_back(
        sxdetail::affineSurfaceVertexWorld(affineAdapter, affineState, vertex));
  }
  auto affineSurface = nb::makeMixedDomainSurface(
      nb::MixedDomainType::Affine,
      0,
      std::move(affineVertices),
      affineAdapter.triangles);
  affineSurface.frictionCoefficient = 0.45;

  const std::vector<Eigen::Vector3d> deformableVertices{
      deformableFixture.cloth->getPosition(0),
      deformableFixture.cloth->getPosition(1),
      deformableFixture.cloth->getPosition(kLyingFlatGridColumns)};
  auto deformableSurface = nb::makeMixedDomainSurface(
      nb::MixedDomainType::Deformable,
      0,
      deformableVertices,
      {Eigen::Vector3i(0, 1, 2)});
  deformableSurface.frictionCoefficient = 0.35;

  const std::array<nb::MixedDomainSurface, 2> surfaces{
      affineSurface, deformableSurface};
  nb::MixedDomainCandidateOptions options;
  options.activationDistance = 0.08;
  const auto candidates
      = nb::buildMixedDomainContactCandidates(surfaces, options);
  const auto diagnostics = nb::evaluateMixedDomainBarrierDiagnostics(
      surfaces, candidates, 0.08, 1.0);

  return ReducedAffineFemMixedDiagnostics{
      candidates.candidates.size(),
      diagnostics.activeBarrierCount,
      diagnostics.minSquaredDistance,
      diagnostics.value,
      diagnostics.finite};
}

ReducedAffineFemVector21d packReducedAffineFemState(
    const sxdetail::AffineBodyState& affineState,
    const std::array<Eigen::Vector3d, 3>& deformableVertices)
{
  ReducedAffineFemVector21d vector;
  vector.head<12>() = sxdetail::affineBodyStateToVector(affineState);
  for (int vertex = 0; vertex < 3; ++vertex) {
    vector.segment<3>(12 + 3 * vertex) = deformableVertices[vertex];
  }
  return vector;
}

ReducedAffineFemCoupledEvaluation evaluateReducedAffineFemCoupledSolve(
    const ReducedAffineFemVector21d& vector,
    const ReducedAffineFemVector21d& target,
    const Eigen::Vector3d& affineLocalPoint,
    const double squaredActivationDistance,
    const double barrierStiffness)
{
  constexpr double kAffineInertialWeight = 1.0;
  constexpr double kDeformableInertialWeight = 1.0;
  constexpr double kOrthogonalityStiffness = 0.25;

  ReducedAffineFemCoupledEvaluation evaluation;
  if (!vector.allFinite() || !target.allFinite()) {
    return evaluation;
  }

  const sxdetail::AffineBodyState affineState
      = sxdetail::affineBodyStateFromVector(vector.head<12>());
  const Eigen::Vector3d triangleA = vector.segment<3>(12);
  const Eigen::Vector3d triangleB = vector.segment<3>(15);
  const Eigen::Vector3d triangleC = vector.segment<3>(18);

  const ReducedAffineFemVector21d displacement = vector - target;
  evaluation.value
      = 0.5 * kAffineInertialWeight * displacement.head<12>().squaredNorm()
        + 0.5 * kDeformableInertialWeight
              * displacement.tail<9>().squaredNorm();
  evaluation.gradient.head<12>()
      = kAffineInertialWeight * displacement.head<12>();
  evaluation.gradient.tail<9>()
      = kDeformableInertialWeight * displacement.tail<9>();
  evaluation.hessian.topLeftCorner<12, 12>()
      = kAffineInertialWeight * sxdetail::AffineMatrix12d::Identity();
  evaluation.hessian.bottomRightCorner<9, 9>()
      = kDeformableInertialWeight * Eigen::Matrix<double, 9, 9>::Identity();

  const Eigen::Vector3d affinePoint
      = sxdetail::affineWorldPoint(affineState, affineLocalPoint);
  const auto barrier = nb::pointTriangleBarrier(
      affinePoint,
      triangleA,
      triangleB,
      triangleC,
      squaredActivationDistance,
      barrierStiffness);

  Eigen::Matrix<double, 12, 21> primitiveJacobian
      = Eigen::Matrix<double, 12, 21>::Zero();
  primitiveJacobian.block<3, 12>(0, 0)
      = sxdetail::affinePointJacobian(affineLocalPoint);
  primitiveJacobian.block<3, 3>(3, 12) = Eigen::Matrix3d::Identity();
  primitiveJacobian.block<3, 3>(6, 15) = Eigen::Matrix3d::Identity();
  primitiveJacobian.block<3, 3>(9, 18) = Eigen::Matrix3d::Identity();

  evaluation.value += barrier.value;
  evaluation.gradient += primitiveJacobian.transpose() * barrier.gradient;
  ReducedAffineFemMatrix21d barrierHessian
      = primitiveJacobian.transpose() * barrier.hessian * primitiveJacobian;
  barrierHessian = 0.5
                   * (barrierHessian
                      + ReducedAffineFemMatrix21d(barrierHessian.transpose()));
  evaluation.hessian += nb::projectSymmetricMatrixToPsd<21>(barrierHessian);
  evaluation.squaredDistance = barrier.squaredDistance;
  evaluation.barrierActive = barrier.active;

  const auto orthogonality = sxdetail::affineOrthogonalityEnergy(
      affineState, kOrthogonalityStiffness, true);
  if (orthogonality.active) {
    evaluation.value += orthogonality.value;
    evaluation.gradient.head<12>() += orthogonality.gradient;
    evaluation.hessian.topLeftCorner<12, 12>() += orthogonality.hessian;
  }

  evaluation.hessian
      = 0.5
        * (evaluation.hessian
           + ReducedAffineFemMatrix21d(evaluation.hessian.transpose()));
  evaluation.valid = std::isfinite(evaluation.value)
                     && evaluation.gradient.allFinite()
                     && evaluation.hessian.allFinite()
                     && std::isfinite(evaluation.squaredDistance);
  return evaluation;
}

ReducedAffineFemVector21d makeReducedAffineFemStep(
    const ReducedAffineFemCoupledEvaluation& evaluation,
    const double maxStepNorm)
{
  ReducedAffineFemVector21d step = -evaluation.gradient;
  Eigen::LDLT<ReducedAffineFemMatrix21d> solver(evaluation.hessian);
  if (solver.info() == Eigen::Success && solver.isPositive()) {
    const ReducedAffineFemVector21d newtonStep
        = solver.solve(-evaluation.gradient);
    if (newtonStep.allFinite() && evaluation.gradient.dot(newtonStep) < 0.0) {
      step = newtonStep;
    }
  }

  const double norm = step.norm();
  if (std::isfinite(norm) && norm > maxStepNorm) {
    step *= maxStepNorm / norm;
  }
  return step.allFinite() ? step : ReducedAffineFemVector21d::Zero();
}

ReducedAffineFemCoupledSolve solveReducedAffineFemCoupledContact(
    const LyingFlatFixture& deformableFixture)
{
  constexpr double kActivationDistance = 0.08;
  constexpr double kSquaredActivationDistance
      = kActivationDistance * kActivationDistance;
  constexpr double kBarrierStiffness = 0.015;
  constexpr double kGradientTolerance = 1e-5;
  constexpr double kMaxStepNorm = 0.025;
  constexpr int kMaxIterations = 32;
  constexpr int kMaxLineSearchIterations = 24;

  ReducedAffineFemCoupledSolve result;
  const std::array<Eigen::Vector3d, 3> deformableVertices{
      deformableFixture.cloth->getPosition(0),
      deformableFixture.cloth->getPosition(1),
      deformableFixture.cloth->getPosition(kLyingFlatGridColumns)};
  const Eigen::Vector3d centroid
      = (deformableVertices[0] + deformableVertices[1] + deformableVertices[2])
        / 3.0;

  sxdetail::AffineBodyState affineState;
  affineState.translation = centroid + Eigen::Vector3d(0.016, 0.012, -0.018);
  affineState.linearMap
      = Eigen::AngleAxisd(0.04, Eigen::Vector3d::UnitY()).toRotationMatrix();

  const Eigen::Vector3d affineLocalPoint = Eigen::Vector3d::Zero();
  const ReducedAffineFemVector21d initial
      = packReducedAffineFemState(affineState, deformableVertices);
  ReducedAffineFemVector21d target = initial;
  target.head<3>() += Eigen::Vector3d(0.0, 0.0, -0.010);
  for (int vertex = 0; vertex < 3; ++vertex) {
    target.segment<3>(12 + 3 * vertex) += Eigen::Vector3d(0.0, 0.0, -0.002);
  }

  ReducedAffineFemVector21d vector = initial;
  auto evaluation = evaluateReducedAffineFemCoupledSolve(
      vector,
      target,
      affineLocalPoint,
      kSquaredActivationDistance,
      kBarrierStiffness);
  if (!evaluation.valid) {
    return result;
  }

  result.valid = true;
  result.initialValue = evaluation.value;
  result.initialGradientNorm = evaluation.gradient.norm();
  result.initialSquaredDistance = evaluation.squaredDistance;

  const double tolerance
      = nb::sanitizeProjectedNewtonTolerance(kGradientTolerance);
  for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
    const double gradientNorm = evaluation.gradient.norm();
    if (nb::projectedNewtonResidualConverged(gradientNorm, tolerance)) {
      result.converged = true;
      break;
    }

    const ReducedAffineFemVector21d step
        = makeReducedAffineFemStep(evaluation, kMaxStepNorm);
    const double directionalDerivative = evaluation.gradient.dot(step);
    if (!(step.squaredNorm() > 0.0) || !(directionalDerivative < 0.0)) {
      break;
    }

    bool accepted = false;
    double stepScale = 1.0;
    for (int lineSearch = 0; lineSearch < kMaxLineSearchIterations;
         ++lineSearch) {
      const ReducedAffineFemVector21d candidateVector
          = vector + stepScale * step;
      const auto candidate = evaluateReducedAffineFemCoupledSolve(
          candidateVector,
          target,
          affineLocalPoint,
          kSquaredActivationDistance,
          kBarrierStiffness);
      if (candidate.valid
          && nb::satisfiesSufficientDecrease(
              evaluation.value,
              candidate.value,
              stepScale * directionalDerivative,
              nb::kDefaultSufficientDecreaseFactor)) {
        vector = candidateVector;
        evaluation = candidate;
        accepted = true;
        break;
      }

      stepScale *= nb::kDefaultBacktrackingScale;
    }

    if (!accepted) {
      break;
    }
    result.iterations = iteration + 1;
  }

  result.finalValue = evaluation.value;
  result.finalGradientNorm = evaluation.gradient.norm();
  result.finalSquaredDistance = evaluation.squaredDistance;
  result.barrierActive = evaluation.barrierActive;
  result.converged = result.converged
                     || nb::projectedNewtonResidualConverged(
                         result.finalGradientNorm, tolerance);
  result.affineDisplacementNorm
      = (vector.head<12>() - initial.head<12>()).norm();
  result.deformableDisplacementNorm
      = (vector.tail<9>() - initial.tail<9>()).norm();
  return result;
}

//==============================================================================
static void BM_Plan083CpuScene_abd_fem_coupling_reduced_side_by_side_step(
    benchmark::State& state)
{
  AbdChainNetFixture affineFixture(kAbdReducedFemCouplingPairCount);
  LyingFlatFixture deformableFixture;
  sx::compute::SequentialExecutor executor;

  std::vector<sxdetail::AffinePointTrianglePairRuntimeStepResult> lastResults;
  sx::DeformableSolverDiagnostics lastDiagnostics;
  ReducedAffineFemMixedDiagnostics mixedDiagnostics;
  ReducedAffineFemCoupledSolve coupledSolve;
  std::size_t failedDeformableSteps = 0;

  for (auto _ : state) {
    state.PauseTiming();
    deformableFixture.reset();
    state.ResumeTiming();

    lastResults = affineFixture.stepAll();
    deformableFixture.world.step(executor);
    lastDiagnostics
        = deformableFixture.world.getLastDeformableSolverDiagnostics();
    mixedDiagnostics
        = evaluateReducedAffineFemMixedDiagnostics(deformableFixture);
    coupledSolve = solveReducedAffineFemCoupledContact(deformableFixture);
    if (lastDiagnostics.bodyCount != 1u || lastDiagnostics.nodeCount == 0u
        || !std::isfinite(deformableFixture.minClothHeight())
        || !mixedDiagnostics.finite || mixedDiagnostics.candidateCount == 0u
        || mixedDiagnostics.activeBarrierCount == 0u || !coupledSolve.valid
        || !coupledSolve.converged || !coupledSolve.barrierActive
        || !(coupledSolve.finalValue < coupledSolve.initialValue)
        || !(coupledSolve.affineDisplacementNorm > 0.0)
        || !(coupledSolve.deformableDisplacementNorm > 0.0)) {
      ++failedDeformableSteps;
    }
    for (const auto& result : lastResults) {
      benchmark::DoNotOptimize(result.solve.pointState.translation.data());
      benchmark::DoNotOptimize(result.solve.triangleState.translation.data());
      benchmark::DoNotOptimize(result.solve.pointState.linearMap.data());
      benchmark::DoNotOptimize(result.solve.triangleState.linearMap.data());
    }
    benchmark::DoNotOptimize(deformableFixture.cloth->getPosition(0).data());
    benchmark::ClobberMemory();
  }

  std::size_t convergedSolveCount = 0;
  std::size_t barrierActiveCount = 0;
  std::size_t validStepCount = 0;
  std::size_t failedStepCount = failedDeformableSteps;
  std::size_t solverIterations = 0;
  double totalObjectiveDecrease = 0.0;
  double maxFinalGradientNorm = 0.0;
  double minTargetSquaredDistance = std::numeric_limits<double>::infinity();
  double minFinalSquaredDistance = std::numeric_limits<double>::infinity();
  double maxLinearSpeed = 0.0;
  double maxAffineVelocityNorm = 0.0;
  double maxDisplacementNorm = 0.0;

  for (std::size_t index = 0; index < lastResults.size(); ++index) {
    const auto& result = lastResults[index];
    const auto& pair = affineFixture.pairs[index];
    if (result.valid) {
      ++validStepCount;
    }
    if (result.converged) {
      ++convergedSolveCount;
    }
    if (result.solve.barrierActive) {
      ++barrierActiveCount;
    }
    if (!result.valid || !result.converged || !result.solve.barrierActive) {
      ++failedStepCount;
    }
    solverIterations += static_cast<std::size_t>(result.solve.iterations);
    totalObjectiveDecrease
        += result.solve.initialValue - result.solve.finalValue;
    maxFinalGradientNorm
        = std::max(maxFinalGradientNorm, result.solve.finalGradientNorm);
    maxLinearSpeed = std::max(maxLinearSpeed, result.maxLinearSpeed);
    maxAffineVelocityNorm
        = std::max(maxAffineVelocityNorm, result.maxAffineVelocityNorm);
    maxDisplacementNorm = std::max(
        maxDisplacementNorm,
        std::max(
            result.pointDisplacementNorm, result.triangleDisplacementNorm));
    const auto targetBarrier = sxdetail::affinePointTriangleBarrier(
        result.pointInertialTarget,
        pair.point,
        result.triangleInertialTarget,
        pair.triangleA,
        pair.triangleB,
        pair.triangleC,
        affineFixture.options.solve.barrier);
    minTargetSquaredDistance = std::min(
        minTargetSquaredDistance, targetBarrier.primitive.squaredDistance);
    minFinalSquaredDistance
        = std::min(minFinalSquaredDistance, result.solve.finalSquaredDistance);
  }

  state.counters["row_abd_fem_coupling"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["affine_body_count"]
      = static_cast<double>(2u * kAbdReducedFemCouplingPairCount);
  state.counters["dynamic_pair_count"]
      = static_cast<double>(kAbdReducedFemCouplingPairCount);
  state.counters["reduced_pair_count"]
      = static_cast<double>(kAbdReducedFemCouplingPairCount);
  state.counters["paper_body_count"] = 27.0;
  state.counters["paper_triangle_count"] = 1'100'000.0;
  state.counters["reference_baseline_measured"] = 0.0;
  state.counters["valid_step_count"] = static_cast<double>(validStepCount);
  state.counters["failed_steps"] = static_cast<double>(failedStepCount);
  state.counters["converged_solve_count"]
      = static_cast<double>(convergedSolveCount);
  state.counters["barrier_active_count"]
      = static_cast<double>(barrierActiveCount);
  state.counters["solver_iterations"] = static_cast<double>(solverIterations);
  state.counters["total_objective_decrease"] = totalObjectiveDecrease;
  state.counters["max_final_gradient_norm"] = maxFinalGradientNorm;
  state.counters["min_target_squared_distance"] = minTargetSquaredDistance;
  state.counters["min_final_squared_distance"] = minFinalSquaredDistance;
  state.counters["squared_activation_distance"]
      = affineFixture.options.solve.barrier.squaredActivationDistance;
  state.counters["max_linear_speed_m_s"] = maxLinearSpeed;
  state.counters["max_affine_velocity_norm"] = maxAffineVelocityNorm;
  state.counters["max_displacement_norm_m"] = maxDisplacementNorm;
  state.counters["deformable_body_count"]
      = static_cast<double>(lastDiagnostics.bodyCount);
  state.counters["deformable_node_count"]
      = static_cast<double>(lastDiagnostics.nodeCount);
  state.counters["deformable_edge_count"]
      = static_cast<double>(lastDiagnostics.edgeCount);
  state.counters["surface_triangle_count"]
      = static_cast<double>(deformableFixture.cloth->getSurfaceTriangleCount());
  state.counters["deformable_solver_iterations"]
      = static_cast<double>(lastDiagnostics.solverIterations);
  recordDeformableRuntimeContactCounters(state, lastDiagnostics);
  state.counters["min_cloth_height_m"] = deformableFixture.minClothHeight();
  state.counters["affine_fem_candidate_diagnostics_measured"] = 1.0;
  state.counters["affine_fem_mixed_candidate_count"]
      = static_cast<double>(mixedDiagnostics.candidateCount);
  state.counters["affine_fem_mixed_active_barrier_count"]
      = static_cast<double>(mixedDiagnostics.activeBarrierCount);
  state.counters["affine_fem_mixed_min_squared_distance"]
      = mixedDiagnostics.minSquaredDistance;
  state.counters["affine_fem_mixed_barrier_value"]
      = mixedDiagnostics.barrierValue;
  state.counters["affine_fem_coupled_contact_measured"] = 1.0;
  state.counters["affine_fem_coupled_solve_converged"]
      = coupledSolve.converged ? 1.0 : 0.0;
  state.counters["affine_fem_coupled_solve_iterations"]
      = static_cast<double>(coupledSolve.iterations);
  state.counters["affine_fem_coupled_objective_decrease"]
      = coupledSolve.initialValue - coupledSolve.finalValue;
  state.counters["affine_fem_coupled_initial_gradient_norm"]
      = coupledSolve.initialGradientNorm;
  state.counters["affine_fem_coupled_final_gradient_norm"]
      = coupledSolve.finalGradientNorm;
  state.counters["affine_fem_coupled_initial_squared_distance"]
      = coupledSolve.initialSquaredDistance;
  state.counters["affine_fem_coupled_final_squared_distance"]
      = coupledSolve.finalSquaredDistance;
  state.counters["affine_fem_coupled_affine_displacement_norm"]
      = coupledSolve.affineDisplacementNorm;
  state.counters["affine_fem_coupled_deformable_displacement_norm"]
      = coupledSolve.deformableDisplacementNorm;
}
BENCHMARK(BM_Plan083CpuScene_abd_fem_coupling_reduced_side_by_side_step)
    ->Unit(benchmark::kMillisecond);

//==============================================================================
static void BM_Plan083CpuScene_abd_house_of_cards_reduced_runtime_step(
    benchmark::State& state)
{
  AbdHouseOfCardsFixture fixture;

  std::vector<sxdetail::AffinePointTriangleRuntimeStepResult> lastResults;
  for (auto _ : state) {
    lastResults = fixture.stepAll();
    for (const auto& result : lastResults) {
      benchmark::DoNotOptimize(result.solve.state.translation.data());
      benchmark::DoNotOptimize(result.solve.state.linearMap.data());
    }
    benchmark::ClobberMemory();
  }

  std::size_t convergedSolveCount = 0;
  std::size_t barrierActiveCount = 0;
  std::size_t validStepCount = 0;
  std::size_t failedStepCount = 0;
  std::size_t solverIterations = 0;
  double totalObjectiveDecrease = 0.0;
  double maxFinalGradientNorm = 0.0;
  double minTargetSquaredDistance = std::numeric_limits<double>::infinity();
  double minFinalSquaredDistance = std::numeric_limits<double>::infinity();
  double maxLinearSpeed = 0.0;
  double maxAffineVelocityNorm = 0.0;
  double maxDisplacementNorm = 0.0;

  for (const auto& result : lastResults) {
    if (result.valid) {
      ++validStepCount;
    }
    if (result.converged) {
      ++convergedSolveCount;
    }
    if (result.solve.barrierActive) {
      ++barrierActiveCount;
    }
    if (!result.valid || !result.converged || !result.solve.barrierActive) {
      ++failedStepCount;
    }
    solverIterations += static_cast<std::size_t>(result.solve.iterations);
    totalObjectiveDecrease
        += result.solve.initialValue - result.solve.finalValue;
    maxFinalGradientNorm
        = std::max(maxFinalGradientNorm, result.solve.finalGradientNorm);
    maxLinearSpeed = std::max(maxLinearSpeed, result.linearSpeed);
    maxAffineVelocityNorm
        = std::max(maxAffineVelocityNorm, result.affineVelocityNorm);
    maxDisplacementNorm
        = std::max(maxDisplacementNorm, result.displacementNorm);
    const auto targetBarrier = sxdetail::affinePointTriangleBarrier(
        result.inertialTarget,
        fixture.point,
        fixture.triangleBody,
        fixture.triangleA,
        fixture.triangleB,
        fixture.triangleC,
        fixture.options.solve.barrier);
    minTargetSquaredDistance = std::min(
        minTargetSquaredDistance, targetBarrier.primitive.squaredDistance);
    minFinalSquaredDistance
        = std::min(minFinalSquaredDistance, result.solve.finalSquaredDistance);
  }

  state.counters["row_abd_vs_rigid_cards"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["affine_body_count"]
      = static_cast<double>(fixture.initialCards.size());
  state.counters["static_triangle_body_count"] = 1.0;
  state.counters["point_triangle_pair_count"]
      = static_cast<double>(fixture.initialCards.size());
  state.counters["valid_step_count"] = static_cast<double>(validStepCount);
  state.counters["failed_steps"] = static_cast<double>(failedStepCount);
  state.counters["converged_solve_count"]
      = static_cast<double>(convergedSolveCount);
  state.counters["barrier_active_count"]
      = static_cast<double>(barrierActiveCount);
  state.counters["solver_iterations"] = static_cast<double>(solverIterations);
  state.counters["total_objective_decrease"] = totalObjectiveDecrease;
  state.counters["max_final_gradient_norm"] = maxFinalGradientNorm;
  state.counters["min_target_squared_distance"] = minTargetSquaredDistance;
  state.counters["min_final_squared_distance"] = minFinalSquaredDistance;
  state.counters["squared_activation_distance"]
      = fixture.options.solve.barrier.squaredActivationDistance;
  state.counters["max_linear_speed_m_s"] = maxLinearSpeed;
  state.counters["max_affine_velocity_norm"] = maxAffineVelocityNorm;
  state.counters["max_displacement_norm_m"] = maxDisplacementNorm;
}
BENCHMARK(BM_Plan083CpuScene_abd_house_of_cards_reduced_runtime_step)
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
