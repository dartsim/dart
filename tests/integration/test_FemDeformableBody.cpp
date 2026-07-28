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
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/fem/DeformableBody.hpp"
#include "dart/dynamics/fem/TetMesh.hpp"
#include "dart/simulation/World.hpp"

#include <gtest/gtest.h>

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

  // Without gravity the damped update is v_{n+1} = v_n * (1 - c*dt/m), so the
  // velocity decays by an exact geometric factor.
  const double dt = world->getTimeStep();
  const double mass = body->getNodeMass(0);
  const double factor
      = std::pow(1.0 - material.mLinearDamping * dt / mass, numSteps);
  const Eigen::Vector3d expectedVelocity = factor * initialVelocity;

  EXPECT_LT(
      (body->getNodeVelocity(0) - expectedVelocity).norm(), kTightTolerance);
  EXPECT_LT(body->getNodeVelocity(0).norm(), initialVelocity.norm());
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
