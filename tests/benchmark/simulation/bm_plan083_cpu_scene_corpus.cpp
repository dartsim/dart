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
#include <dart/simulation/multibody/joint.hpp>
#include <dart/simulation/world.hpp>

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
    world.setGravity(Eigen::Vector3d::Zero());

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

    sx::DeformableBodyOptions clothOptions;
    const double halfWidth = 0.5 * kLyingFlatGridSpacing
                             * static_cast<double>(kLyingFlatGridColumns - 1u);
    const double halfDepth = 0.5 * kLyingFlatGridSpacing
                             * static_cast<double>(kLyingFlatGridRows - 1u);
    for (std::size_t row = 0; row < kLyingFlatGridRows; ++row) {
      for (std::size_t col = 0; col < kLyingFlatGridColumns; ++col) {
        const double x
            = kLyingFlatGridSpacing * static_cast<double>(col) - halfWidth;
        const double y
            = kLyingFlatGridSpacing * static_cast<double>(row) - halfDepth;
        const double stagger = (col + row) % 2u == 0u ? 0.0 : 0.003;
        const double lateral
            = col < kLyingFlatGridColumns / 2u ? 0.025 : -0.025;
        clothOptions.positions.emplace_back(x, y, 0.078 + stagger);
        clothOptions.velocities.emplace_back(lateral, 0.0, -0.02);
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
      const sx::CollisionShape& shape)
  {
    sx::RigidBodyOptions options;
    options.isStatic = true;
    options.position = position;
    sx::RigidBody body = world.addRigidBody(name, options);
    body.setCollisionShape(shape);
    body.setDeformableSurfaceCcdObstacle(true);
    body.setDeformableObstacleBarrierOnly(true);
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

BENCHMARK_MAIN();
