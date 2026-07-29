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

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/dynamics/fem/DeformableBody.hpp"
#include "dart/dynamics/fem/TetMesh.hpp"
#include "dart/simulation/World.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

#include <cmath>

using namespace dart;
using namespace dart::dynamics;

namespace {

const Eigen::Vector3d kBoxSize(0.4, 0.2, 0.3);
const Eigen::Vector3i kBoxDivisions(3, 2, 4);
constexpr double kTightTolerance = 1e-12;

//==============================================================================
fem::TetMesh createTestMesh()
{
  return fem::TetMesh::createBox(kBoxSize, kBoxDivisions);
}

//==============================================================================
simulation::WorldPtr createTestWorld()
{
  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);
  return world;
}

//==============================================================================
SkeletonPtr createFreeRigidBody(const std::string& name)
{
  auto skeleton = Skeleton::create(name);
  skeleton->createJointAndBodyNodePair<FreeJoint>();
  return skeleton;
}

//==============================================================================
/// Creates an immobile ground plate for scenes that need a rigid body to
/// settle.
SkeletonPtr createFloor()
{
  auto floor = Skeleton::create("floor");
  auto* body = floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;
  body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d(10.0, 10.0, 0.1)));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -0.05);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);
  floor->setMobile(false);
  return floor;
}

//==============================================================================
/// Creates a colliding box that will fall onto the floor and come to rest.
SkeletonPtr createFallingBox(const Eigen::Vector3d& position)
{
  auto skeleton = Skeleton::create("box");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.2)));
  pair.second->setMass(1.0);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  pair.first->setPositions(FreeJoint::convertToPositions(tf));
  return skeleton;
}

//==============================================================================
/// Rebuilds the undeformed shape matrix of one tetrahedron from the mesh.
Eigen::Matrix3d restShapeMatrix(const fem::TetMesh& mesh, std::size_t tetIndex)
{
  const Eigen::Vector4i tet = mesh.getTet(tetIndex);
  const Eigen::Vector3d& x0 = mesh.getRestPosition(tet[0]);

  Eigen::Matrix3d shape;
  shape.col(0) = mesh.getRestPosition(tet[1]) - x0;
  shape.col(1) = mesh.getRestPosition(tet[2]) - x0;
  shape.col(2) = mesh.getRestPosition(tet[3]) - x0;
  return shape;
}

} // namespace

//==============================================================================
TEST(FemDeformableBody, BoxMeshTilesTheBoxWithPositiveVolumes)
{
  const fem::TetMesh mesh = createTestMesh();

  const std::size_t expectedTets = static_cast<std::size_t>(kBoxDivisions[0])
                                   * kBoxDivisions[1] * kBoxDivisions[2] * 6;
  EXPECT_EQ(expectedTets, mesh.getNumTets());
  EXPECT_EQ(
      static_cast<std::size_t>(kBoxDivisions[0] + 1) * (kBoxDivisions[1] + 1)
          * (kBoxDivisions[2] + 1),
      mesh.getNumNodes());

  for (std::size_t t = 0; t < mesh.getNumTets(); ++t) {
    EXPECT_GT(mesh.getRestVolume(t), 0.0) << "tetrahedron " << t;

    // The cached inverse rest shape must actually invert the rest shape.
    const Eigen::Matrix3d shape = restShapeMatrix(mesh, t);
    const Eigen::Matrix3d shouldBeIdentity
        = mesh.getInverseRestShape(t) * shape;
    EXPECT_LT(
        (shouldBeIdentity - Eigen::Matrix3d::Identity()).norm(),
        kTightTolerance)
        << "tetrahedron " << t;
  }

  // The tetrahedra tile the box, so their volumes sum to the box volume.
  const double boxVolume = kBoxSize[0] * kBoxSize[1] * kBoxSize[2];
  EXPECT_NEAR(boxVolume, mesh.getTotalRestVolume(), kTightTolerance);
}

//==============================================================================
TEST(FemDeformableBody, RejectsDegenerateAndInvertedTetrahedra)
{
  std::vector<Eigen::Vector3d> positions;
  positions.emplace_back(0.0, 0.0, 0.0);
  positions.emplace_back(1.0, 0.0, 0.0);
  positions.emplace_back(0.0, 1.0, 0.0);
  positions.emplace_back(0.0, 0.0, 1.0);

  // Positively oriented: accepted.
  {
    common::aligned_vector<Eigen::Vector4i> tets;
    tets.emplace_back(0, 1, 2, 3);
    EXPECT_NO_THROW(fem::TetMesh(positions, tets));
  }

  // Swapping two nodes inverts the tetrahedron: rejected rather than silently
  // reordered, so an inverted input mesh cannot produce negative masses.
  {
    common::aligned_vector<Eigen::Vector4i> tets;
    tets.emplace_back(0, 1, 3, 2);
    EXPECT_THROW(fem::TetMesh(positions, tets), std::invalid_argument);
  }

  // Repeating a node collapses the tetrahedron to zero volume: rejected.
  {
    common::aligned_vector<Eigen::Vector4i> tets;
    tets.emplace_back(0, 1, 2, 2);
    EXPECT_THROW(fem::TetMesh(positions, tets), std::invalid_argument);
  }

  // Out-of-range node index: rejected.
  {
    common::aligned_vector<Eigen::Vector4i> tets;
    tets.emplace_back(0, 1, 2, 7);
    EXPECT_THROW(fem::TetMesh(positions, tets), std::invalid_argument);
  }

  // A sliver keeps a positive volume, so the orientation check alone lets it
  // through, but its inverse rest shape is numerically unusable and would
  // amplify every perturbation once deformation gradients are formed.
  {
    std::vector<Eigen::Vector3d> flattened = positions;
    flattened[3] = Eigen::Vector3d(0.0, 0.0, 1e-14);
    common::aligned_vector<Eigen::Vector4i> tets;
    tets.emplace_back(0, 1, 2, 3);
    EXPECT_THROW(fem::TetMesh(flattened, tets), std::invalid_argument);
  }

  // A merely thin element is still legitimate and must be accepted.
  {
    std::vector<Eigen::Vector3d> thin = positions;
    thin[3] = Eigen::Vector3d(0.0, 0.0, 0.01);
    common::aligned_vector<Eigen::Vector4i> tets;
    tets.emplace_back(0, 1, 2, 3);
    EXPECT_NO_THROW(fem::TetMesh(thin, tets));
  }
}

//==============================================================================
TEST(FemDeformableBody, LumpedMassesSumToDensityTimesRestVolume)
{
  fem::Material material;
  material.mDensity = 1250.0;

  const auto body = fem::DeformableBody::create(createTestMesh(), material);

  const double expectedMass
      = material.mDensity * kBoxSize[0] * kBoxSize[1] * kBoxSize[2];
  EXPECT_NEAR(expectedMass, body->getTotalMass(), kTightTolerance);

  double summed = 0.0;
  for (std::size_t i = 0; i < body->getNumNodes(); ++i) {
    EXPECT_GT(body->getNodeMass(i), 0.0) << "node " << i;
    summed += body->getNodeMass(i);
  }
  EXPECT_NEAR(expectedMass, summed, kTightTolerance);
}

//==============================================================================
TEST(FemDeformableBody, EmbeddedSurfaceFollowsAffineMotionOfTheVolume)
{
  const fem::TetMesh mesh = createTestMesh();
  const auto body = fem::DeformableBody::create(mesh);

  // A mix of box corners, face centers, and the interior, so the embedding is
  // exercised on tetrahedron vertices, faces, and interiors alike.
  std::vector<Eigen::Vector3d> surface;
  const Eigen::Vector3d half = 0.5 * kBoxSize;
  for (int sx = -1; sx <= 1; sx += 2) {
    for (int sy = -1; sy <= 1; sy += 2) {
      for (int sz = -1; sz <= 1; sz += 2) {
        surface.emplace_back(sx * half[0], sy * half[1], sz * half[2]);
      }
    }
  }
  surface.emplace_back(half[0], 0.0, 0.0);
  surface.emplace_back(0.0, -half[1], 0.0);
  surface.emplace_back(0.0, 0.0, half[2]);
  surface.emplace_back(0.0, 0.0, 0.0);
  surface.emplace_back(0.13, -0.04, 0.09);

  body->setEmbeddedSurface(surface);
  ASSERT_EQ(surface.size(), body->getNumSurfaceVertices());

  // At rest the embedding must reproduce the input surface exactly.
  for (std::size_t v = 0; v < surface.size(); ++v) {
    EXPECT_LT(
        (body->getSurfaceVertexPosition(v) - surface[v]).norm(),
        kTightTolerance)
        << "vertex " << v;
  }

  // Barycentric weights sum to one, so an affine motion of every node must move
  // every embedded vertex by exactly the same affine map. This checks the
  // containment search, the weights, and the interpolation at once.
  Eigen::Matrix3d linear;
  linear << 1.3, 0.2, -0.1, 0.0, 0.8, 0.35, -0.25, 0.1, 1.1;
  const Eigen::Vector3d translation(0.7, -1.2, 3.4);

  for (std::size_t i = 0; i < body->getNumNodes(); ++i) {
    body->setNodePosition(i, linear * mesh.getRestPosition(i) + translation);
  }

  for (std::size_t v = 0; v < surface.size(); ++v) {
    const Eigen::Vector3d expected = linear * surface[v] + translation;
    EXPECT_LT(
        (body->getSurfaceVertexPosition(v) - expected).norm(), kTightTolerance)
        << "vertex " << v;
  }
}

//==============================================================================
// The affine gate above cannot see which tetrahedron a vertex binds to: weights
// solved from any tetrahedron reconstruct the vertex at rest and stay affine
// covariant, so that test passes even when every vertex is bound to a
// tetrahedron nowhere near it. These two checks are what actually constrain the
// containment search, which matters because a spatial acceleration structure is
// meant to replace it later.
TEST(FemDeformableBody, EmbeddedSurfaceBindsToTheContainingTetrahedron)
{
  const fem::TetMesh mesh = createTestMesh();
  const auto body = fem::DeformableBody::create(mesh);

  // Strictly interior points, so each one genuinely lies inside a tetrahedron.
  std::vector<Eigen::Vector3d> surface;
  surface.emplace_back(0.0, 0.0, 0.0);
  surface.emplace_back(0.11, -0.03, 0.07);
  surface.emplace_back(-0.09, 0.04, -0.05);
  surface.emplace_back(0.05, 0.05, 0.10);
  body->setEmbeddedSurface(surface);
  ASSERT_EQ(surface.size(), body->getNumSurfaceVertices());

  // Containment: every barycentric weight of an interior vertex lies in [0, 1]
  // exactly when its tetrahedron really contains it.
  for (std::size_t v = 0; v < surface.size(); ++v) {
    const Eigen::Vector4d weights = body->getSurfaceVertexWeights(v);
    EXPECT_NEAR(1.0, weights.sum(), kTightTolerance) << "vertex " << v;
    EXPECT_GE(weights.minCoeff(), -kTightTolerance) << "vertex " << v;
    EXPECT_LE(weights.maxCoeff(), 1.0 + kTightTolerance) << "vertex " << v;
  }

  std::vector<Eigen::Vector3d> before;
  for (std::size_t v = 0; v < surface.size(); ++v) {
    before.push_back(body->getSurfaceVertexPosition(v));
  }

  // A non-affine motion discriminates between tetrahedra: displace exactly one
  // node, and a vertex may move only by that node's own weight. A vertex bound
  // to a tetrahedron that does not reference the node must not move at all.
  // Displace the node carrying the most weight for the first vertex. Weights
  // sum to one, so that corner holds at least a quarter and the vertex is
  // guaranteed to respond; an arbitrary corner could sit at exactly zero weight
  // for a vertex lying on a grid plane.
  const Eigen::Vector4d firstWeights = body->getSurfaceVertexWeights(0);
  int dominantCorner = 0;
  for (int corner = 1; corner < 4; ++corner) {
    if (firstWeights[corner] > firstWeights[dominantCorner]) {
      dominantCorner = corner;
    }
  }
  const auto movedNode = static_cast<std::size_t>(
      mesh.getTet(body->getSurfaceVertexTet(0))[dominantCorner]);
  const Eigen::Vector3d displacement(0.017, -0.023, 0.031);
  body->setNodePosition(
      movedNode, mesh.getRestPosition(movedNode) + displacement);

  double totalMotion = 0.0;
  for (std::size_t v = 0; v < surface.size(); ++v) {
    const Eigen::Vector4i tet = mesh.getTet(body->getSurfaceVertexTet(v));
    const Eigen::Vector4d weights = body->getSurfaceVertexWeights(v);

    double movedNodeWeight = 0.0;
    for (int corner = 0; corner < 4; ++corner) {
      if (tet[corner] == static_cast<int>(movedNode)) {
        movedNodeWeight += weights[corner];
      }
    }

    const Eigen::Vector3d expected = before[v] + movedNodeWeight * displacement;
    EXPECT_LT(
        (body->getSurfaceVertexPosition(v) - expected).norm(), kTightTolerance)
        << "vertex " << v;
    totalMotion += (body->getSurfaceVertexPosition(v) - before[v]).norm();
  }

  EXPECT_GT(totalMotion, 0.0) << "no vertex responded to the displaced node";
}

//==============================================================================
TEST(FemDeformableBody, FreeFallMatchesTheExactDiscreteSolution)
{
  auto world = createTestWorld();
  world->addSkeleton(createFreeRigidBody("rigid"));

  fem::Material material;
  material.mLinearDamping = 0.0;
  const auto body = fem::DeformableBody::create(createTestMesh(), material);
  const Eigen::Vector3d restPosition = body->getNodePosition(0);
  body->attachTo(world);

  const int numSteps = 100;
  for (int i = 0; i < numSteps; ++i) {
    world->step();
  }

  // Semi-implicit Euler from rest under constant gravity has a closed form:
  // v_n = n*g*dt and x_n = x_0 + g*dt^2*n*(n+1)/2. Matching it to 1e-12 proves
  // the body advanced exactly once per world step; a skipped or doubled step
  // would miss by orders of magnitude.
  const double dt = world->getTimeStep();
  const Eigen::Vector3d gravity = world->getGravity();
  const Eigen::Vector3d expectedVelocity = numSteps * dt * gravity;
  const Eigen::Vector3d expectedPosition
      = restPosition + gravity * dt * dt * (0.5 * numSteps * (numSteps + 1));

  EXPECT_LT(
      (body->getNodeVelocity(0) - expectedVelocity).norm(), kTightTolerance);
  EXPECT_LT(
      (body->getNodePosition(0) - expectedPosition).norm(), kTightTolerance);

  // Every node falls together, so the body stays undeformed without elasticity.
  for (std::size_t i = 1; i < body->getNumNodes(); ++i) {
    const Eigen::Vector3d offset
        = body->getNodePosition(i) - body->getNodePosition(0);
    const Eigen::Vector3d restOffset = body->getMesh().getRestPosition(i)
                                       - body->getMesh().getRestPosition(0);
    EXPECT_LT((offset - restOffset).norm(), kTightTolerance) << "node " << i;
  }
}

//==============================================================================
TEST(FemDeformableBody, IntegratesInAWorldWithoutSkeletons)
{
  auto world = createTestWorld();

  const auto body = fem::DeformableBody::create(createTestMesh());
  const Eigen::Vector3d restPosition = body->getNodePosition(0);
  body->attachTo(world);

  const int numSteps = 10;
  for (int i = 0; i < numSteps; ++i) {
    world->step();
  }

  const double dt = world->getTimeStep();
  const Eigen::Vector3d expectedPosition
      = restPosition
        + world->getGravity() * dt * dt * (0.5 * numSteps * (numSteps + 1));
  EXPECT_LT(
      (body->getNodePosition(0) - expectedPosition).norm(), kTightTolerance);
}

//==============================================================================
TEST(FemDeformableBody, LinearDampingDecaysVelocityGeometrically)
{
  auto world = createTestWorld();
  world->setGravity(Eigen::Vector3d::Zero());

  fem::Material material;
  material.mDensity = 1000.0;
  material.mLinearDamping = 3.0;

  const auto body = fem::DeformableBody::create(createTestMesh(), material);
  const Eigen::Vector3d initialVelocity(1.5, -0.5, 0.25);
  for (std::size_t i = 0; i < body->getNumNodes(); ++i) {
    body->setNodeVelocity(i, initialVelocity);
  }
  body->attachTo(world);

  const int numSteps = 25;
  for (int i = 0; i < numSteps; ++i) {
    world->step();
  }

  // Damping is mass proportional and implicit, so without gravity the update is
  // v_{n+1} = v_n / (1 + c*dt) and the velocity decays by an exact geometric
  // factor that does not depend on node mass.
  const double dt = world->getTimeStep();
  const double factor = std::pow(1.0 + material.mLinearDamping * dt, -numSteps);
  const Eigen::Vector3d expectedVelocity = factor * initialVelocity;

  EXPECT_LT(
      (body->getNodeVelocity(0) - expectedVelocity).norm(), kTightTolerance);
  EXPECT_LT(body->getNodeVelocity(0).norm(), initialVelocity.norm());
}

//==============================================================================
// Damping is mass proportional so that every node decays by the same factor.
// Dividing the damping force by node mass instead would give each node its own
// factor, and lumped masses vary several-fold across an ordinary box mesh, so a
// body translating uniformly would damp faster at its corners than in its
// interior and pull itself apart with no elastic force present at all. A large
// damping rate also exercises stability: an explicit factor of (1 - c*dt) would
// diverge here, while the implicit form cannot.
TEST(FemDeformableBody, DampingPreservesShapeAndStaysStable)
{
  auto world = createTestWorld();

  fem::Material material;
  material.mDensity = 1000.0;
  material.mLinearDamping = 500.0; // c*dt = 0.5, far outside explicit stability

  const fem::TetMesh mesh
      = fem::TetMesh::createBox(kBoxSize, Eigen::Vector3i(4, 3, 5));
  const auto body = fem::DeformableBody::create(mesh, material);

  double minMass = std::numeric_limits<double>::max();
  double maxMass = 0.0;
  for (std::size_t i = 0; i < body->getNumNodes(); ++i) {
    minMass = std::min(minMass, body->getNodeMass(i));
    maxMass = std::max(maxMass, body->getNodeMass(i));
  }
  ASSERT_GT(maxMass / minMass, 4.0)
      << "node masses are too uniform for this gate to discriminate";

  const Eigen::Vector3d initialVelocity(0.3, -0.2, 0.15);
  for (std::size_t i = 0; i < body->getNumNodes(); ++i) {
    body->setNodeVelocity(i, initialVelocity);
  }
  body->attachTo(world);

  for (int i = 0; i < 500; ++i) {
    world->step();
  }

  // Uniform motion in, uniform motion out: every node keeps its rest offset and
  // shares one velocity, however much its mass differs from its neighbours'.
  for (std::size_t i = 0; i < body->getNumNodes(); ++i) {
    const Eigen::Vector3d velocity = body->getNodeVelocity(i);
    ASSERT_TRUE(velocity.allFinite()) << "node " << i;
    EXPECT_LT((velocity - body->getNodeVelocity(0)).norm(), kTightTolerance)
        << "node " << i;

    const Eigen::Vector3d offset
        = body->getNodePosition(i) - body->getNodePosition(0);
    const Eigen::Vector3d restOffset
        = mesh.getRestPosition(i) - mesh.getRestPosition(0);
    EXPECT_LT((offset - restOffset).norm(), kTightTolerance) << "node " << i;
  }
}

//==============================================================================
TEST(FemDeformableBody, RejectsNonPhysicalMaterials)
{
  const fem::TetMesh mesh = createTestMesh();

  fem::Material negativeDensity;
  negativeDensity.mDensity = -1000.0;
  EXPECT_THROW(
      fem::DeformableBody::create(mesh, negativeDensity),
      std::invalid_argument);

  fem::Material zeroDensity;
  zeroDensity.mDensity = 0.0;
  EXPECT_THROW(
      fem::DeformableBody::create(mesh, zeroDensity), std::invalid_argument);

  fem::Material negativeDamping;
  negativeDamping.mLinearDamping = -1.0;
  EXPECT_THROW(
      fem::DeformableBody::create(mesh, negativeDamping),
      std::invalid_argument);

  EXPECT_NO_THROW(fem::DeformableBody::create(mesh, fem::Material()));
}

//==============================================================================
TEST(FemDeformableBody, SimulationIsDeterministic)
{
  const auto run = []() {
    auto world = createTestWorld();
    world->addSkeleton(createFreeRigidBody("rigid"));
    const auto body = fem::DeformableBody::create(createTestMesh());
    body->attachTo(world);
    for (int i = 0; i < 50; ++i) {
      world->step();
    }

    double checksum = 0.0;
    for (std::size_t i = 0; i < body->getNumNodes(); ++i) {
      checksum
          += body->getNodePosition(i).sum() + body->getNodeVelocity(i).sum();
    }
    return checksum;
  };

  EXPECT_EQ(run(), run());
}

//==============================================================================
TEST(FemDeformableBody, AttachedBodyLeavesRigidTrajectoryUnchanged)
{
  const auto simulateRigid = [](bool withDeformableBody) {
    auto world = createTestWorld();
    auto skeleton = createFreeRigidBody("rigid");
    world->addSkeleton(skeleton);

    // Keep the body alive for the whole simulation when it is used.
    fem::DeformableBodyPtr deformable;
    if (withDeformableBody) {
      deformable = fem::DeformableBody::create(createTestMesh());
      deformable->attachTo(world);
    }

    for (int i = 0; i < 200; ++i) {
      world->step();
    }

    return skeleton->getPositions().eval();
  };

  const Eigen::VectorXd withoutBody = simulateRigid(false);
  const Eigen::VectorXd withBody = simulateRigid(true);

  // The body never joins the LCP and applies no force to any skeleton, so the
  // rigid trajectory must be bit-identical, not merely close.
  ASSERT_EQ(withoutBody.size(), withBody.size());
  EXPECT_EQ(0.0, (withoutBody - withBody).norm());
}

//==============================================================================
// World::step() returns early without solving at all once automatic
// deactivation decides the rigid scene is resting and no contacts remain, and
// the deformable body advances from inside that solve. A body attached to such
// a world would silently freeze while simulated time kept advancing, which is
// indistinguishable from being detached. Attaching therefore turns the resting
// fast path off, and this gate covers a scene that would otherwise reach it: a
// box falling onto a floor and settling, which is as ordinary as scenes get.
TEST(FemDeformableBody, KeepsIntegratingWhenTheRigidSceneWouldFallAsleep)
{
  auto world = createTestWorld();
  world->addSkeleton(createFloor());
  world->addSkeleton(createFallingBox(Eigen::Vector3d(0.0, 0.0, 0.12)));

  // DART enables deactivation by default; assert it so this gate keeps covering
  // the resting fast path even if that default were ever revisited.
  ASSERT_TRUE(world->getDeactivationOptions().mEnabled);

  const auto body = fem::DeformableBody::create(createTestMesh());
  body->attachTo(world);
  EXPECT_FALSE(world->getDeactivationOptions().mEnabled)
      << "attaching must disable the resting fast path that skips the solve";

  // Let the box land and go quiet. Nothing ever touches the deformable body.
  for (int i = 0; i < 3000; ++i) {
    world->step();
  }
  ASSERT_LT(world->getSkeleton("box")->getVelocities().norm(), 1e-2)
      << "rigid box never settled, so this scene would not sleep";

  const Eigen::Vector3d startPosition = body->getNodePosition(0);
  const Eigen::Vector3d startVelocity = body->getNodeVelocity(0);
  const double startTime = world->getTime();

  const int numSteps = 100;
  for (int i = 0; i < numSteps; ++i) {
    world->step();
  }

  const double dt = world->getTimeStep();
  const Eigen::Vector3d gravity = world->getGravity();
  EXPECT_NEAR(startTime + numSteps * dt, world->getTime(), 1e-9);

  // Same closed form as the free-fall gate, continued from a moving start.
  const Eigen::Vector3d expectedVelocity
      = startVelocity + numSteps * dt * gravity;
  const Eigen::Vector3d expectedPosition
      = startPosition + numSteps * dt * startVelocity
        + gravity * dt * dt * (0.5 * numSteps * (numSteps + 1));

  EXPECT_LT((body->getNodeVelocity(0) - expectedVelocity).norm(), 1e-9);
  EXPECT_LT((body->getNodePosition(0) - expectedPosition).norm(), 1e-9);
}

//==============================================================================
TEST(FemDeformableBody, DetachStopsIntegrationAndReattachResumes)
{
  auto world = createTestWorld();
  const auto body = fem::DeformableBody::create(createTestMesh());

  EXPECT_FALSE(body->isAttached());
  body->attachTo(world);
  EXPECT_TRUE(body->isAttached());

  world->step();
  const Eigen::Vector3d afterOneStep = body->getNodePosition(0);
  EXPECT_GT((afterOneStep - body->getMesh().getRestPosition(0)).norm(), 0.0);

  body->detach();
  EXPECT_FALSE(body->isAttached());

  for (int i = 0; i < 10; ++i) {
    world->step();
  }
  EXPECT_EQ(0.0, (body->getNodePosition(0) - afterOneStep).norm());

  body->attachTo(world);
  EXPECT_TRUE(body->isAttached());
  world->step();
  EXPECT_GT((body->getNodePosition(0) - afterOneStep).norm(), 0.0);
}
