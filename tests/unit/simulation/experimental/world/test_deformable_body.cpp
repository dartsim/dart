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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/deformable_body.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/compute/sequential_executor.hpp>
#include <dart/simulation/experimental/compute/world_step_stage.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <dart/collision/native/narrow_phase/primitive_ccd.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <sstream>

namespace sx = dart::simulation::experimental;
namespace compute = dart::simulation::experimental::compute;
namespace nc = dart::collision::native;

namespace {

//==============================================================================
void expectVectorNear(
    const Eigen::Vector3d& actual,
    const Eigen::Vector3d& expected,
    double tolerance = 1e-12)
{
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

//==============================================================================
sx::DeformableBodyOptions makeSingleNodeBodyOptions(
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity,
    bool fixed = false)
{
  sx::DeformableBodyOptions options;
  options.positions = {position};
  options.velocities = {velocity};
  options.masses = {1.0};
  if (fixed) {
    options.fixedNodes = {0};
  }
  return options;
}

//==============================================================================
sx::DeformableBodyOptions makeSurfaceCrossingBodyOptions(
    double pointHeight = 1.0, double pointVelocity = -20.0)
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-1.0, -1.0, 0.0),
         Eigen::Vector3d(1.0, -1.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0),
         Eigen::Vector3d(0.0, 0.0, pointHeight)};
  options.velocities
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d(0.0, 0.0, pointVelocity)};
  options.masses = {1.0, 1.0, 1.0, 1.0};
  options.fixedNodes = {0, 1, 2};
  options.surfaceTriangles = {sx::DeformableSurfaceTriangle{0, 1, 2}};
  return options;
}

//==============================================================================
sx::DeformableBodyOptions makeInterBodyMovingPointOptions()
{
  auto options = makeSurfaceCrossingBodyOptions();
  options.positions[0].z() = 3.0;
  options.positions[1].z() = 3.0;
  options.positions[2].z() = 3.0;
  return options;
}

//==============================================================================
sx::DeformableBodyOptions makeStationaryTriangleObstacleOptions(double z = 0.0)
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-1.0, -1.0, z),
         Eigen::Vector3d(1.0, -1.0, z),
         Eigen::Vector3d(0.0, 1.0, z)};
  options.velocities
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero()};
  options.masses = {1.0, 1.0, 1.0};
  options.fixedNodes = {0, 1, 2};
  options.surfaceTriangles = {sx::DeformableSurfaceTriangle{0, 1, 2}};
  return options;
}

//==============================================================================
sx::DeformableBodyOptions makeMovingTriangleOptions()
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-1.0, -1.0, 1.0),
         Eigen::Vector3d(1.0, -1.0, 1.0),
         Eigen::Vector3d(0.0, 1.0, 1.0)};
  options.velocities
      = {Eigen::Vector3d(0.0, 0.0, -20.0),
         Eigen::Vector3d(0.0, 0.0, -20.0),
         Eigen::Vector3d(0.0, 0.0, -20.0)};
  options.masses = {1.0, 1.0, 1.0};
  options.surfaceTriangles = {sx::DeformableSurfaceTriangle{0, 1, 2}};
  return options;
}

//==============================================================================
sx::DeformableBodyOptions makeStationaryPointObstacleOptions()
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-1.0, -1.0, 3.0),
         Eigen::Vector3d(1.0, -1.0, 3.0),
         Eigen::Vector3d(0.0, 1.0, 3.0),
         Eigen::Vector3d(0.0, 0.0, 0.0)};
  options.velocities
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero()};
  options.masses = {1.0, 1.0, 1.0, 1.0};
  options.fixedNodes = {0, 1, 2, 3};
  options.surfaceTriangles = {sx::DeformableSurfaceTriangle{0, 1, 2}};
  return options;
}

//==============================================================================
sx::DeformableBodyOptions makeMovingEdgeOptions()
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(0.0, -1.0, -1.0),
         Eigen::Vector3d(0.0, -1.0, 1.0),
         Eigen::Vector3d(1.0, -1.0, 0.0)};
  options.velocities
      = {Eigen::Vector3d(0.0, 20.0, 0.0),
         Eigen::Vector3d(0.0, 20.0, 0.0),
         Eigen::Vector3d(0.0, 20.0, 0.0)};
  options.masses = {1.0, 1.0, 1.0};
  options.surfaceTriangles = {sx::DeformableSurfaceTriangle{0, 1, 2}};
  return options;
}

//==============================================================================
sx::DeformableBodyOptions makeStationaryEdgeObstacleOptions()
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-1.0, 0.0, 0.0),
         Eigen::Vector3d(1.0, 0.0, 0.0),
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

//==============================================================================
sx::DeformableBodyOptions makeVolumetricInteriorNodeCrossingOptions()
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-1.0, -1.0, 0.0),
         Eigen::Vector3d(1.0, -1.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0),
         Eigen::Vector3d(0.0, 0.0, 2.0),
         Eigen::Vector3d(0.0, 0.0, 0.5)};
  options.velocities
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d(0.0, 0.0, -20.0)};
  options.fixedNodes = {0, 1, 2, 3};
  options.surfaceTriangles
      = {sx::DeformableSurfaceTriangle{0, 1, 2},
         sx::DeformableSurfaceTriangle{0, 3, 1},
         sx::DeformableSurfaceTriangle{1, 3, 2},
         sx::DeformableSurfaceTriangle{2, 3, 0}};
  options.tetrahedra
      = {sx::DeformableTetrahedron{0, 1, 2, 4},
         sx::DeformableTetrahedron{0, 1, 3, 4},
         sx::DeformableTetrahedron{1, 2, 3, 4},
         sx::DeformableTetrahedron{2, 0, 3, 4}};
  return options;
}

//==============================================================================
void expectNoExactPointTriangleCrossing(
    const Eigen::Vector3d& pointStart,
    const Eigen::Vector3d& pointEnd,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  nc::CcdOption option;
  option.tolerance = 1e-8;
  option.maxIterations = 64;
  option.minSeparation = 0.0;
  option.advancement = nc::CcdAdvancement::Conservative;

  nc::CcdPrimitiveResult exact;
  EXPECT_FALSE(
      nc::pointTriangleCcdExact(
          pointStart, pointEnd, a, a, b, b, c, c, option, exact));
}

//==============================================================================
sx::DeformableBodyOptions makeTwoNodeBody()
{
  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX()};
  options.edges = {sx::DeformableEdge{0, 1, -1.0}};
  options.fixedNodes = {0};
  return options;
}

//==============================================================================
sx::DeformableBodyOptions makeSingleTetrahedronBody()
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::UnitX(),
         Eigen::Vector3d::UnitY(),
         Eigen::Vector3d::UnitZ()};
  options.tetrahedra = {sx::DeformableTetrahedron{0, 1, 2, 3}};
  return options;
}

} // namespace

//==============================================================================
TEST(DeformableBody, AddGetAndExposeNodeStateWithoutSolverDetails)
{
  sx::World world;

  auto options = makeTwoNodeBody();
  options.velocities = {Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitY()};
  options.masses = {2.0, 3.0};

  auto body = world.addDeformableBody("strand", options);

  EXPECT_TRUE(body.isValid());
  EXPECT_EQ(body.getName(), "strand");
  EXPECT_TRUE(world.hasDeformableBody("strand"));
  EXPECT_EQ(world.getDeformableBodyCount(), 1u);

  auto restored = world.getDeformableBody("strand");
  ASSERT_TRUE(restored.has_value());
  EXPECT_TRUE(restored->isValid());
  EXPECT_EQ(restored->getNodeCount(), 2u);
  expectVectorNear(restored->getPosition(0), Eigen::Vector3d::Zero());
  expectVectorNear(restored->getVelocity(1), Eigen::Vector3d::UnitY());
  EXPECT_DOUBLE_EQ(restored->getMass(0), 2.0);
  EXPECT_TRUE(restored->isFixedNode(0));
  EXPECT_FALSE(restored->isFixedNode(1));

  ASSERT_EQ(restored->getEdgeCount(), 1u);
  const auto edge = restored->getEdge(0);
  EXPECT_EQ(edge.nodeA, 0u);
  EXPECT_EQ(edge.nodeB, 1u);
  EXPECT_DOUBLE_EQ(edge.restLength, 1.0);
}

//==============================================================================
TEST(DeformableBody, RejectsInvalidModels)
{
  sx::World world;

  sx::DeformableBodyOptions empty;
  EXPECT_THROW(
      world.addDeformableBody("empty", empty), sx::InvalidArgumentException);

  auto mismatchedVelocity = makeTwoNodeBody();
  mismatchedVelocity.velocities = {Eigen::Vector3d::Zero()};
  EXPECT_THROW(
      world.addDeformableBody("bad_velocity", mismatchedVelocity),
      sx::InvalidArgumentException);

  auto nonfinitePosition = makeTwoNodeBody();
  nonfinitePosition.positions[1].x() = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
      world.addDeformableBody("bad_position", nonfinitePosition),
      sx::InvalidArgumentException);

  auto nonfiniteVelocity = makeTwoNodeBody();
  nonfiniteVelocity.velocities
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)};
  EXPECT_THROW(
      world.addDeformableBody("bad_finite_velocity", nonfiniteVelocity),
      sx::InvalidArgumentException);

  auto invalidMass = makeTwoNodeBody();
  invalidMass.masses = {1.0, -1.0};
  EXPECT_THROW(
      world.addDeformableBody("bad_mass", invalidMass),
      sx::InvalidArgumentException);

  auto zeroMass = makeTwoNodeBody();
  zeroMass.masses = {1.0, 0.0};
  EXPECT_THROW(
      world.addDeformableBody("zero_mass", zeroMass),
      sx::InvalidArgumentException);

  auto nonfiniteMass = makeTwoNodeBody();
  nonfiniteMass.masses = {1.0, std::numeric_limits<double>::quiet_NaN()};
  EXPECT_THROW(
      world.addDeformableBody("bad_finite_mass", nonfiniteMass),
      sx::InvalidArgumentException);

  auto duplicateFixed = makeTwoNodeBody();
  duplicateFixed.fixedNodes = {0, 0};
  EXPECT_THROW(
      world.addDeformableBody("duplicate_fixed", duplicateFixed),
      sx::InvalidArgumentException);

  auto invalidFixed = makeTwoNodeBody();
  invalidFixed.fixedNodes = {2};
  EXPECT_THROW(
      world.addDeformableBody("bad_fixed", invalidFixed),
      sx::InvalidArgumentException);

  auto invalidEdge = makeTwoNodeBody();
  invalidEdge.edges = {sx::DeformableEdge{0, 2, 1.0}};
  EXPECT_THROW(
      world.addDeformableBody("bad_edge", invalidEdge),
      sx::InvalidArgumentException);

  auto coincidentEdge = makeTwoNodeBody();
  coincidentEdge.positions[1] = Eigen::Vector3d::Zero();
  EXPECT_THROW(
      world.addDeformableBody("coincident", coincidentEdge),
      sx::InvalidArgumentException);

  auto negativeStiffness = makeTwoNodeBody();
  negativeStiffness.edgeStiffness = -1.0;
  EXPECT_THROW(
      world.addDeformableBody("bad_stiffness", negativeStiffness),
      sx::InvalidArgumentException);

  auto nonfiniteStiffness = makeTwoNodeBody();
  nonfiniteStiffness.edgeStiffness = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
      world.addDeformableBody("bad_finite_stiffness", nonfiniteStiffness),
      sx::InvalidArgumentException);

  auto negativeDamping = makeTwoNodeBody();
  negativeDamping.damping = -1.0;
  EXPECT_THROW(
      world.addDeformableBody("bad_damping", negativeDamping),
      sx::InvalidArgumentException);

  auto nonfiniteDamping = makeTwoNodeBody();
  nonfiniteDamping.damping = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
      world.addDeformableBody("bad_finite_damping", nonfiniteDamping),
      sx::InvalidArgumentException);

  auto badMaterial = makeTwoNodeBody();
  badMaterial.material.density = 0.0;
  EXPECT_THROW(
      world.addDeformableBody("bad_density", badMaterial),
      sx::InvalidArgumentException);

  badMaterial = makeTwoNodeBody();
  badMaterial.material.youngsModulus = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
      world.addDeformableBody("bad_youngs", badMaterial),
      sx::InvalidArgumentException);

  badMaterial = makeTwoNodeBody();
  badMaterial.material.poissonRatio = 0.5;
  EXPECT_THROW(
      world.addDeformableBody("bad_poisson", badMaterial),
      sx::InvalidArgumentException);
}

//==============================================================================
TEST(DeformableBody, MeshTopologyValidationAndSurfaceExtraction)
{
  sx::World world;

  auto options = makeSingleTetrahedronBody();
  auto body = world.addDeformableBody("tetra", options);

  EXPECT_EQ(body.getSurfaceTriangleCount(), 4u);
  EXPECT_EQ(body.getTetrahedronCount(), 1u);
  EXPECT_DOUBLE_EQ(body.getTetrahedronRestVolume(0), 1.0 / 6.0);

  const auto tet = body.getTetrahedron(0);
  EXPECT_EQ(tet.nodeA, 0u);
  EXPECT_EQ(tet.nodeB, 1u);
  EXPECT_EQ(tet.nodeC, 2u);
  EXPECT_EQ(tet.nodeD, 3u);

  const auto face0 = body.getSurfaceTriangle(0);
  EXPECT_EQ(face0.nodeA, 0u);
  EXPECT_EQ(face0.nodeB, 2u);
  EXPECT_EQ(face0.nodeC, 1u);

  sx::World invertedWorld;
  auto inverted = makeSingleTetrahedronBody();
  inverted.tetrahedra = {sx::DeformableTetrahedron{0, 1, 3, 2}};
  auto invertedBody = invertedWorld.addDeformableBody("inverted", inverted);
  const auto canonical = invertedBody.getTetrahedron(0);
  EXPECT_DOUBLE_EQ(invertedBody.getTetrahedronRestVolume(0), 1.0 / 6.0);
  EXPECT_EQ(canonical.nodeC, 2u);
  EXPECT_EQ(canonical.nodeD, 3u);

  sx::World invalidWorld;
  auto invalid = makeSingleTetrahedronBody();
  invalid.tetrahedra = {sx::DeformableTetrahedron{0, 1, 2, 2}};
  EXPECT_THROW(
      invalidWorld.addDeformableBody("repeated_node", invalid),
      sx::InvalidArgumentException);

  invalid = makeSingleTetrahedronBody();
  invalid.positions[3] = Eigen::Vector3d(1.0, 1.0, 0.0);
  EXPECT_THROW(
      invalidWorld.addDeformableBody("flat_tet", invalid),
      sx::InvalidArgumentException);

  invalid = makeSingleTetrahedronBody();
  invalid.tetrahedra.push_back(invalid.tetrahedra.front());
  EXPECT_THROW(
      invalidWorld.addDeformableBody("duplicate_tet", invalid),
      sx::InvalidArgumentException);

  invalid.positions
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::UnitX(),
         Eigen::Vector3d::UnitY()};
  invalid.tetrahedra.clear();
  invalid.surfaceTriangles = {sx::DeformableSurfaceTriangle{0, 1, 2}};
  EXPECT_THROW(
      invalidWorld.addDeformableBody("surface_without_mass", invalid),
      sx::InvalidArgumentException);

  invalid.masses = {1.0, 1.0, 1.0};
  invalid.surfaceTriangles.push_back(invalid.surfaceTriangles.front());
  EXPECT_THROW(
      invalidWorld.addDeformableBody("duplicate_surface", invalid),
      sx::InvalidArgumentException);
}

//==============================================================================
TEST(DeformableBody, DensityMassAssemblyUsesTetrahedraOnly)
{
  sx::World world;

  auto options = makeSingleTetrahedronBody();
  options.material.density = 12.0;
  auto body = world.addDeformableBody("density_mass", options);

  for (std::size_t i = 0; i < body.getNodeCount(); ++i) {
    EXPECT_DOUBLE_EQ(body.getMass(i), 0.5);
  }
  const auto material = body.getMaterialProperties();
  EXPECT_DOUBLE_EQ(material.density, 12.0);
  EXPECT_DOUBLE_EQ(material.youngsModulus, 1.0e5);
  EXPECT_DOUBLE_EQ(material.poissonRatio, 0.3);

  sx::World explicitMassWorld;
  options.masses = {1.0, 2.0, 3.0, 4.0};
  options.material.density = 1200.0;
  auto explicitMassBody
      = explicitMassWorld.addDeformableBody("explicit_mass", options);
  EXPECT_DOUBLE_EQ(explicitMassBody.getMass(0), 1.0);
  EXPECT_DOUBLE_EQ(explicitMassBody.getMass(1), 2.0);
  EXPECT_DOUBLE_EQ(explicitMassBody.getMass(2), 3.0);
  EXPECT_DOUBLE_EQ(explicitMassBody.getMass(3), 4.0);
}

//==============================================================================
TEST(DeformableBody, MeshMetadataDoesNotChangeSpringStepping)
{
  auto makeSpringBody = [](bool withSurface) {
    sx::DeformableBodyOptions options;
    options.positions
        = {Eigen::Vector3d::Zero(),
           Eigen::Vector3d::UnitX(),
           Eigen::Vector3d::UnitY()};
    options.velocities
        = {Eigen::Vector3d::Zero(),
           Eigen::Vector3d(0.1, 0.0, 0.0),
           Eigen::Vector3d(0.0, 0.2, 0.0)};
    options.masses = {1.0, 1.0, 1.0};
    options.edges
        = {sx::DeformableEdge{0, 1, 1.0}, sx::DeformableEdge{0, 2, 1.0}};
    options.fixedNodes = {0};
    options.edgeStiffness = 75.0;
    if (withSurface) {
      options.surfaceTriangles = {sx::DeformableSurfaceTriangle{0, 1, 2}};
    }
    return options;
  };

  sx::World pointWorld;
  sx::World meshWorld;
  pointWorld.setGravity(Eigen::Vector3d(0.0, 0.0, -1.0));
  meshWorld.setGravity(Eigen::Vector3d(0.0, 0.0, -1.0));
  pointWorld.setTimeStep(0.05);
  meshWorld.setTimeStep(0.05);

  auto pointBody = pointWorld.addDeformableBody("point", makeSpringBody(false));
  auto meshBody = meshWorld.addDeformableBody("mesh", makeSpringBody(true));

  pointWorld.step(4);
  meshWorld.step(4);

  for (std::size_t i = 0; i < pointBody.getNodeCount(); ++i) {
    expectVectorNear(meshBody.getPosition(i), pointBody.getPosition(i));
    expectVectorNear(meshBody.getVelocity(i), pointBody.getVelocity(i));
  }
}

//==============================================================================
TEST(DeformableBody, RestLengthSpringIsStationaryWithoutLoads)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  auto body = world.addDeformableBody("rest", makeTwoNodeBody());

  world.step(3);

  expectVectorNear(body.getPosition(0), Eigen::Vector3d::Zero());
  expectVectorNear(body.getPosition(1), Eigen::Vector3d::UnitX());
  expectVectorNear(body.getVelocity(0), Eigen::Vector3d::Zero());
  expectVectorNear(body.getVelocity(1), Eigen::Vector3d::Zero());
}

//==============================================================================
TEST(DeformableBody, FixedNodeStaysFixedWhileStretchedEdgeContracts)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.1);

  auto options = makeTwoNodeBody();
  options.positions[1] = Eigen::Vector3d(2.0, 0.0, 0.0);
  options.edges = {sx::DeformableEdge{0, 1, 1.0}};
  options.edgeStiffness = 100.0;
  auto body = world.addDeformableBody("stretch", options);

  world.step();

  expectVectorNear(body.getPosition(0), Eigen::Vector3d::Zero());
  expectVectorNear(body.getVelocity(0), Eigen::Vector3d::Zero());
  EXPECT_LT(body.getPosition(1).x(), 2.0);
  EXPECT_GT(body.getPosition(1).x(), 0.99);
  EXPECT_LT(body.getPosition(1).z(), 0.0);
}

//==============================================================================
TEST(DeformableBody, FixedSpringMatchesAnalyticImplicitEulerStep)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  auto options = makeTwoNodeBody();
  options.positions[1] = Eigen::Vector3d(2.0, 0.0, 0.0);
  options.edges = {sx::DeformableEdge{0, 1, 1.0}};
  options.edgeStiffness = 100.0;
  options.masses = {1.0, 1.0};
  auto body = world.addDeformableBody("analytic_spring", options);

  world.step();

  constexpr double mass = 1.0;
  constexpr double timeStep = 0.1;
  constexpr double stiffness = 100.0;
  constexpr double initialPosition = 2.0;
  constexpr double restLength = 1.0;
  constexpr double inertialWeight = mass / (timeStep * timeStep);
  constexpr double expectedX
      = (inertialWeight * initialPosition + stiffness * restLength)
        / (inertialWeight + stiffness);

  expectVectorNear(body.getPosition(1), Eigen::Vector3d(expectedX, 0.0, 0.0));
  expectVectorNear(
      body.getVelocity(1),
      Eigen::Vector3d((expectedX - initialPosition) / timeStep, 0.0, 0.0));
}

//==============================================================================
TEST(DeformableBody, FreeParticleMatchesImplicitEulerTargetForDifferentMasses)
{
  const Eigen::Vector3d expectedPosition
      = Eigen::Vector3d(0.5, -0.25, 1.0) + 0.1 * Eigen::Vector3d(1.0, 2.0, 3.0)
        + 0.01 * Eigen::Vector3d(0.0, 0.0, -9.81);
  const Eigen::Vector3d expectedVelocity
      = (expectedPosition - Eigen::Vector3d(0.5, -0.25, 1.0)) / 0.1;

  sx::World lightWorld;
  lightWorld.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  lightWorld.setTimeStep(0.1);
  sx::DeformableBodyOptions lightOptions;
  lightOptions.positions = {Eigen::Vector3d(0.5, -0.25, 1.0)};
  lightOptions.velocities = {Eigen::Vector3d(1.0, 2.0, 3.0)};
  lightOptions.masses = {0.01};
  auto light = lightWorld.addDeformableBody("light", lightOptions);

  sx::World heavyWorld;
  heavyWorld.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  heavyWorld.setTimeStep(0.1);
  sx::DeformableBodyOptions heavyOptions = lightOptions;
  heavyOptions.masses = {10.0};
  auto heavy = heavyWorld.addDeformableBody("heavy", heavyOptions);

  lightWorld.step();
  heavyWorld.step();

  expectVectorNear(light.getPosition(0), expectedPosition, 1e-12);
  expectVectorNear(heavy.getPosition(0), expectedPosition, 1e-12);
  expectVectorNear(light.getVelocity(0), expectedVelocity, 1e-12);
  expectVectorNear(heavy.getVelocity(0), expectedVelocity, 1e-12);
}

//==============================================================================
TEST(DeformableBody, StepCountWithExecutorRunsDefaultDeformableStage)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  auto options = makeTwoNodeBody();
  options.positions[1] = Eigen::Vector3d(2.0, 0.0, 0.0);
  options.edges = {sx::DeformableEdge{0, 1, 1.0}};
  auto body = world.addDeformableBody("executor_step", options);

  compute::SequentialExecutor executor;
  world.step(2, executor);

  expectVectorNear(body.getPosition(0), Eigen::Vector3d::Zero());
  EXPECT_LT(body.getPosition(1).x(), 2.0);
  EXPECT_EQ(world.getFrame(), 2u);
}

//==============================================================================
TEST(DeformableBody, CustomStageStepOverloadsRunDefaultDeformableStage)
{
  auto addStretchedBody = [](sx::World& world) {
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.1);

    auto options = makeTwoNodeBody();
    options.positions[1] = Eigen::Vector3d(2.0, 0.0, 0.0);
    options.edges = {sx::DeformableEdge{0, 1, 1.0}};
    options.edgeStiffness = 100.0;
    return world.addDeformableBody("custom_stage", options);
  };

  compute::SequentialExecutor executor;
  compute::KinematicsStage replacementKinematics;

  sx::World defaultWorld;
  sx::World customWorld;
  auto defaultBody = addStretchedBody(defaultWorld);
  auto customBody = addStretchedBody(customWorld);

  defaultWorld.step(executor);
  customWorld.step(executor, replacementKinematics);

  expectVectorNear(customBody.getPosition(1), defaultBody.getPosition(1));
  expectVectorNear(customBody.getVelocity(1), defaultBody.getVelocity(1));
  EXPECT_LT(customBody.getPosition(1).x(), 2.0);
  EXPECT_DOUBLE_EQ(customWorld.getTime(), defaultWorld.getTime());
  EXPECT_EQ(customWorld.getFrame(), defaultWorld.getFrame());

  sx::World countedDefaultWorld;
  sx::World countedCustomWorld;
  auto countedDefaultBody = addStretchedBody(countedDefaultWorld);
  auto countedCustomBody = addStretchedBody(countedCustomWorld);

  countedDefaultWorld.step(2, executor);
  countedCustomWorld.step(2, executor, replacementKinematics);

  expectVectorNear(
      countedCustomBody.getPosition(1), countedDefaultBody.getPosition(1));
  expectVectorNear(
      countedCustomBody.getVelocity(1), countedDefaultBody.getVelocity(1));
  EXPECT_LT(countedCustomBody.getPosition(1).x(), 2.0);
  EXPECT_DOUBLE_EQ(countedCustomWorld.getTime(), countedDefaultWorld.getTime());
  EXPECT_EQ(countedCustomWorld.getFrame(), countedDefaultWorld.getFrame());
}

//==============================================================================
TEST(DeformableBody, StaticGroundBarrierPreventsCrossing)
{
  sx::World world;
  world.setTimeStep(0.1);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(10.0, 10.0, 0.5)));
  ground.setDeformableGroundBarrier(true);
  EXPECT_TRUE(ground.isDeformableGroundBarrier());

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.0, 0.0, 0.5)};
  options.velocities = {Eigen::Vector3d(0.0, 0.0, -10.0)};
  auto body = world.addDeformableBody("falling_node", options);

  world.step(5);

  EXPECT_LT(body.getPosition(0).z(), 0.5);
  EXPECT_GE(body.getPosition(0).z(), 1e-4 - 1e-12);
}

//==============================================================================
TEST(DeformableBody, ActiveStaticGroundContactAllowsTangentialMotion)
{
  sx::World world;
  world.setTimeStep(0.1);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(10.0, 10.0, 0.5)));
  ground.setDeformableGroundBarrier(true);

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.0, 0.0, 1e-4)};
  options.velocities = {Eigen::Vector3d(1.0, 0.0, 0.0)};
  auto body = world.addDeformableBody("sliding_node", options);

  world.step();

  EXPECT_GT(body.getPosition(0).x(), 0.0);
  EXPECT_GE(body.getPosition(0).z(), 1e-4 - 1e-12);
  EXPECT_GT(body.getVelocity(0).x(), 0.0);
}

//==============================================================================
TEST(DeformableBody, ActiveDirichletNodesDoNotBlockGroundBarrierSolve)
{
  sx::World world;
  world.setTimeStep(0.1);
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(10.0, 10.0, 0.5)));
  ground.setDeformableGroundBarrier(true);

  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(0.0, 0.0, 0.1), Eigen::Vector3d(0.0, 0.0, 0.5)};
  options.edges = {sx::DeformableEdge{0, 1, -1.0}};
  options.edgeStiffness = 100.0;

  sx::DeformableDirichletBoundaryCondition boundary;
  boundary.nodes = {0};
  boundary.linearVelocity = Eigen::Vector3d(0.0, 0.0, -2.0);
  options.dirichletBoundaryConditions.push_back(boundary);

  const auto body = world.addDeformableBody("scripted_anchor", options);
  world.step();

  EXPECT_NEAR(body.getPosition(0).z(), -0.1, 1e-12);
  EXPECT_LT(body.getPosition(1).z(), 0.5);
  EXPECT_GE(body.getPosition(1).z(), 1e-4 - 1e-12);
}

//==============================================================================
TEST(DeformableBody, StaticGroundBarrierUsesFiniteStaticFootprint)
{
  sx::World world;
  world.setTimeStep(0.1);

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.isStatic = true;
  obstacleOptions.position = Eigen::Vector3d(100.0, 0.0, 9.5);
  auto obstacle = world.addRigidBody("distant_obstacle", obstacleOptions);
  obstacle.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(1.0, 1.0, 0.5)));
  obstacle.setDeformableGroundBarrier(true);

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.0, 0.0, 0.5)};
  auto body = world.addDeformableBody("falling_node", options);

  world.step();

  EXPECT_LT(body.getPosition(0).z(), 0.5);
  EXPECT_LT(body.getPosition(0).z(), 1.0);
}

//==============================================================================
TEST(DeformableBody, StaticGroundBarrierUsesOrientedBoxFootprint)
{
  sx::World world;
  world.setTimeStep(0.1);

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.isStatic = true;
  obstacleOptions.orientation = Eigen::AngleAxisd(
      0.25 * 3.14159265358979323846, Eigen::Vector3d::UnitZ());
  auto obstacle = world.addRigidBody("rotated_obstacle", obstacleOptions);
  obstacle.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 1.0)));
  obstacle.setDeformableGroundBarrier(true);

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(2.5, 1.0, 0.25)};
  auto body = world.addDeformableBody("outside_rotated_footprint", options);

  world.step();

  EXPECT_LT(body.getPosition(0).z(), 0.25);
}

//==============================================================================
TEST(DeformableBody, StaticGroundBarrierUsesTiltedBoxSurfaceHeight)
{
  sx::World world;
  world.setTimeStep(0.1);

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.isStatic = true;
  obstacleOptions.orientation = Eigen::AngleAxisd(
      0.25 * 3.14159265358979323846, Eigen::Vector3d::UnitY());
  auto obstacle = world.addRigidBody("tilted_obstacle", obstacleOptions);
  obstacle.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.2)));
  obstacle.setDeformableGroundBarrier(true);

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.0, 0.0, 0.35)};
  auto body = world.addDeformableBody("above_tilted_surface", options);

  world.step();

  EXPECT_LT(body.getPosition(0).z(), 0.35);
  EXPECT_LT(body.getPosition(0).z(), 0.5);
}

//==============================================================================
TEST(DeformableBody, StaticCollisionRequiresGroundBarrierOptIn)
{
  sx::World world;
  world.setTimeStep(0.1);

  sx::RigidBodyOptions ceilingOptions;
  ceilingOptions.isStatic = true;
  ceilingOptions.position = Eigen::Vector3d(0.0, 0.0, 0.5);
  auto ceiling = world.addRigidBody("ordinary_static_box", ceilingOptions);
  ceiling.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(10.0, 10.0, 0.5)));
  ASSERT_FALSE(ceiling.isDeformableGroundBarrier());

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.isStatic = true;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  auto sphere = world.addRigidBody("ordinary_static_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(1.0));
  ASSERT_FALSE(sphere.isDeformableGroundBarrier());

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.0, 0.0, 0.25)};
  options.velocities = {Eigen::Vector3d(0.0, 0.0, -1.0)};
  auto body = world.addDeformableBody("falling_node", options);

  world.step();

  EXPECT_LT(body.getPosition(0).z(), 0.25);
  EXPECT_LT(body.getPosition(0).z(), 1.0);
}

//==============================================================================
TEST(DeformableBody, StaticGroundBarrierCcdLimitsFastFallingNode)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(10.0, 10.0, 0.5)));
  ground.setDeformableGroundBarrier(true);

  auto body = world.addDeformableBody(
      "falling_node",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, -20.0)));

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_GT(body.getPosition(0).z(), 1e-4 - 1e-12);
  EXPECT_LT(body.getPosition(0).z(), 1.0);
  EXPECT_EQ(stats.staticGroundBarrierCount, 1u);
  EXPECT_GT(stats.staticGroundBarrierCcdNodeChecks, 0u);
  EXPECT_GT(stats.staticGroundBarrierCcdSampleChecks, 0u);
  EXPECT_GT(stats.staticGroundBarrierCcdHits, 0u);
  EXPECT_GT(stats.staticGroundBarrierCcdLimitedSteps, 0u);
}

//==============================================================================
TEST(DeformableBody, StaticGroundBarrierCcdCatchesFiniteFootprintFlyThrough)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.05);
  auto ground = world.addRigidBody("finite_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.05)));
  ground.setDeformableGroundBarrier(true);

  auto body = world.addDeformableBody(
      "fast_node",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(-1.0, 0.0, -0.2), Eigen::Vector3d(20.0, 0.0, 0.0)));

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_LT(body.getPosition(0).x(), -0.5);
  EXPECT_NEAR(body.getPosition(0).z(), -0.2, 1e-12);
  EXPECT_GT(stats.staticGroundBarrierCcdHits, 0u);
  EXPECT_GT(stats.staticGroundBarrierCcdLimitedSteps, 0u);
}

//==============================================================================
TEST(DeformableBody, StaticGroundBarrierCcdLimitsSphereTopSurface)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.isStatic = true;
  sphereOptions.position = Eigen::Vector3d::Zero();
  auto sphere = world.addRigidBody("sphere_ground", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  sphere.setDeformableGroundBarrier(true);

  auto body = world.addDeformableBody(
      "falling_node",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, -20.0)));

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_GT(body.getPosition(0).z(), 0.5001 - 1e-12);
  EXPECT_LT(body.getPosition(0).z(), 1.0);
  EXPECT_GT(stats.staticGroundBarrierCcdHits, 0u);
  EXPECT_GT(stats.staticGroundBarrierCcdLimitedSteps, 0u);
}

//==============================================================================
TEST(DeformableBody, StaticGroundBarrierCcdIgnoresOrdinaryStaticShapes)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.isStatic = true;
  obstacleOptions.position = Eigen::Vector3d(0.0, 0.0, -0.05);
  auto obstacle = world.addRigidBody("ordinary_static_box", obstacleOptions);
  obstacle.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.05)));
  ASSERT_FALSE(obstacle.isDeformableGroundBarrier());

  auto body = world.addDeformableBody(
      "fast_node",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(-1.0, 0.0, -0.2), Eigen::Vector3d(20.0, 0.0, 0.0)));

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_GT(body.getPosition(0).x(), 0.5);
  EXPECT_EQ(stats.staticGroundBarrierCount, 0u);
  EXPECT_EQ(stats.staticGroundBarrierCcdNodeChecks, 0u);
  EXPECT_EQ(stats.staticGroundBarrierCcdHits, 0u);
  EXPECT_EQ(stats.staticGroundBarrierCcdLimitedSteps, 0u);
}

//==============================================================================
TEST(DeformableBody, StaticGroundBarrierCcdSkipsFixedNodes)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(10.0, 10.0, 0.5)));
  ground.setDeformableGroundBarrier(true);

  auto body = world.addDeformableBody(
      "fixed_node",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(0.0, 0.0, -0.5),
          Eigen::Vector3d(0.0, 0.0, -20.0),
          true));

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_NEAR(body.getPosition(0).z(), -0.5, 1e-12);
  EXPECT_EQ(stats.staticGroundBarrierCount, 1u);
  EXPECT_EQ(stats.staticGroundBarrierCcdNodeChecks, 0u);
  EXPECT_EQ(stats.staticGroundBarrierCcdHits, 0u);
}

//==============================================================================
TEST(DeformableBody, SurfaceContactCcdPreventsFastPointTriangleCrossing)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  auto body = world.addDeformableBody(
      "surface_crossing", makeSurfaceCrossingBodyOptions());
  const Eigen::Vector3d pointStart = body.getPosition(3);

  world.step();

  EXPECT_GT(body.getPosition(3).z(), 0.0);
  EXPECT_LT(body.getPosition(3).z(), pointStart.z());
  expectNoExactPointTriangleCrossing(
      pointStart,
      body.getPosition(3),
      body.getPosition(0),
      body.getPosition(1),
      body.getPosition(2));
}

//==============================================================================
TEST(DeformableBody, SurfaceContactCcdReportsCustomStageStats)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  auto body = world.addDeformableBody(
      "surface_crossing", makeSurfaceCrossingBodyOptions());

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_GT(body.getPosition(3).z(), 0.0);
  EXPECT_GT(stats.surfaceContactCandidateBuilds, 0u);
  EXPECT_GT(stats.surfaceContactPointTriangleCandidates, 0u);
  EXPECT_GT(stats.surfaceContactCcdPointTriangleChecks, 0u);
  EXPECT_GT(stats.surfaceContactCcdHits, 0u);
  EXPECT_GT(stats.surfaceContactCcdLimitedSteps, 0u);
  EXPECT_EQ(stats.surfaceContactCcdIndeterminateCount, 0u);
  EXPECT_EQ(stats.interBodySurfaceContactPointTriangleCandidates, 0u);
  EXPECT_EQ(stats.interBodySurfaceContactEdgeEdgeCandidates, 0u);
}

//==============================================================================
TEST(DeformableBody, InterBodySurfaceContactCcdLimitsMovingPoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  auto moving
      = world.addDeformableBody("moving", makeInterBodyMovingPointOptions());
  world.addDeformableBody("obstacle", makeStationaryTriangleObstacleOptions());

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_GT(moving.getPosition(3).z(), 0.0);
  EXPECT_LT(moving.getPosition(3).z(), 1.0);
  EXPECT_GT(stats.interBodySurfaceContactCandidateBuilds, 0u);
  EXPECT_GT(stats.interBodySurfaceContactPointTriangleCandidates, 0u);
  EXPECT_GT(stats.interBodySurfaceContactCcdPointTriangleChecks, 0u);
  EXPECT_GT(stats.interBodySurfaceContactCcdHits, 0u);
  EXPECT_GT(stats.interBodySurfaceContactCcdLimitedSteps, 0u);
}

//==============================================================================
TEST(DeformableBody, InterBodySurfaceContactCcdLimitsMovingTriangle)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  auto moving = world.addDeformableBody("moving", makeMovingTriangleOptions());
  world.addDeformableBody("obstacle", makeStationaryPointObstacleOptions());

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_GT(moving.getPosition(0).z(), 0.0);
  EXPECT_LT(moving.getPosition(0).z(), 1.0);
  EXPECT_GT(stats.interBodySurfaceContactPointTriangleCandidates, 0u);
  EXPECT_GT(stats.interBodySurfaceContactCcdPointTriangleChecks, 0u);
  EXPECT_GT(stats.interBodySurfaceContactCcdLimitedSteps, 0u);
}

//==============================================================================
TEST(DeformableBody, InterBodySurfaceContactCcdChecksEdgeEdgePairs)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  auto moving = world.addDeformableBody("moving", makeMovingEdgeOptions());
  world.addDeformableBody("obstacle", makeStationaryEdgeObstacleOptions());

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_LT(moving.getPosition(0).y(), 0.0);
  EXPECT_GT(stats.interBodySurfaceContactEdgeEdgeCandidates, 0u);
  EXPECT_GT(stats.interBodySurfaceContactCcdEdgeEdgeChecks, 0u);
  EXPECT_GT(stats.interBodySurfaceContactCcdHits, 0u);
  EXPECT_GT(stats.interBodySurfaceContactCcdLimitedSteps, 0u);
}

//==============================================================================
TEST(DeformableBody, InterBodySurfaceContactCcdUsesStageStartSnapshot)
{
  const auto runScene = [](bool movingFirst) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.1);

    sx::DeformableBody moving;
    if (movingFirst) {
      moving = world.addDeformableBody(
          "moving", makeInterBodyMovingPointOptions());
      world.addDeformableBody(
          "obstacle", makeStationaryTriangleObstacleOptions());
    } else {
      world.addDeformableBody(
          "obstacle", makeStationaryTriangleObstacleOptions());
      moving = world.addDeformableBody(
          "moving", makeInterBodyMovingPointOptions());
    }

    compute::SequentialExecutor executor;
    compute::DeformableDynamicsStage stage;
    compute::WorldStepPipeline pipeline;
    pipeline.addStage(stage);
    world.step(executor, pipeline);
    return moving.getPosition(3).z();
  };

  const double movingFirstZ = runScene(true);
  const double obstacleFirstZ = runScene(false);
  EXPECT_GT(movingFirstZ, 0.0);
  EXPECT_GT(obstacleFirstZ, 0.0);
  EXPECT_NEAR(movingFirstZ, obstacleFirstZ, 1e-12);
}

//==============================================================================
TEST(DeformableBody, SurfaceContactCcdIgnoresVolumetricInteriorNodes)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  auto body = world.addDeformableBody(
      "volumetric_interior", makeVolumetricInteriorNodeCrossingOptions());

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_LT(body.getPosition(4).z(), 0.0);
  EXPECT_GT(stats.surfaceContactCandidateBuilds, 0u);
  EXPECT_EQ(stats.surfaceContactPointTriangleCandidates, 0u);
  EXPECT_EQ(stats.surfaceContactCcdPointTriangleChecks, 0u);
  EXPECT_EQ(stats.surfaceContactCcdLimitedSteps, 0u);
}

//==============================================================================
TEST(DeformableBody, SurfaceContactCcdRejectsInitialSeparationBand)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  auto body = world.addDeformableBody(
      "surface_initial_band", makeSurfaceCrossingBodyOptions(5e-5, -1.0));
  const Eigen::Vector3d pointStart = body.getPosition(3);

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  expectVectorNear(body.getPosition(3), pointStart, 1e-12);
  EXPECT_GT(stats.surfaceContactCcdZeroStepCount, 0u);
  EXPECT_EQ(stats.acceptedLineSearchSteps, 0u);
  EXPECT_GT(stats.rejectedLineSearchCandidates, 0u);
}

//==============================================================================
TEST(DeformableBody, SurfaceFreeParticlesKeepFastPath)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.25);

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d::Zero()};
  options.velocities = {Eigen::Vector3d(0.0, 0.0, 2.0)};
  auto body = world.addDeformableBody("free_particle", options);

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  expectVectorNear(body.getPosition(0), Eigen::Vector3d(0.0, 0.0, 0.5));
  EXPECT_EQ(stage.getLastStats().surfaceContactCandidateBuilds, 0u);
  EXPECT_EQ(stage.getLastStats().solverIterations, 0u);
}

//==============================================================================
TEST(DeformableBody, SurfaceContactScratchIsPerBody)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  auto surfaced = world.addDeformableBody(
      "surface_crossing", makeSurfaceCrossingBodyOptions());

  sx::DeformableBodyOptions freeOptions;
  freeOptions.positions = {Eigen::Vector3d(3.0, 0.0, 0.0)};
  freeOptions.velocities = {Eigen::Vector3d(0.0, 0.0, 5.0)};
  auto free = world.addDeformableBody("free_particle", freeOptions);

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  EXPECT_GT(surfaced.getPosition(3).z(), 0.0);
  expectVectorNear(free.getPosition(0), Eigen::Vector3d(3.0, 0.0, 0.5));
  EXPECT_EQ(stage.getLastStats().bodyCount, 2u);
  EXPECT_GT(stage.getLastStats().surfaceContactCandidateBuilds, 0u);
}

//==============================================================================
TEST(DeformableBody, StageMetadataUsesDeformableDomain)
{
  compute::DeformableDynamicsStage stage;

  EXPECT_EQ(stage.getName(), "deformable_dynamics");
  const auto metadata = stage.getMetadata();
  EXPECT_EQ(metadata.domain, compute::ComputeStageDomain::DeformableBody);

  bool readsTopology = false;
  for (const auto& resource : metadata.resources) {
    if (resource.resource == "deformable_body.topology") {
      readsTopology = resource.mode == compute::ComputeAccessMode::Read;
    }
  }
  EXPECT_TRUE(readsTopology);
}

//==============================================================================
TEST(DeformableBody, StepIsDeterministic)
{
  auto addBody = [](sx::World& world) {
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -1.0));
    world.setTimeStep(0.05);

    sx::DeformableBodyOptions options;
    options.positions
        = {Eigen::Vector3d::Zero(),
           Eigen::Vector3d(1.5, 0.2, 0.0),
           Eigen::Vector3d(2.0, 0.5, 0.1)};
    options.velocities
        = {Eigen::Vector3d::Zero(),
           Eigen::Vector3d(0.1, 0.0, 0.0),
           Eigen::Vector3d(0.0, -0.1, 0.0)};
    options.edges
        = {sx::DeformableEdge{0, 1, 1.0}, sx::DeformableEdge{1, 2, 0.75}};
    options.fixedNodes = {0};
    options.edgeStiffness = 25.0;
    return world.addDeformableBody("chain", options);
  };

  sx::World worldA;
  sx::World worldB;
  auto bodyA = addBody(worldA);
  auto bodyB = addBody(worldB);

  worldA.step(10);
  worldB.step(10);

  for (std::size_t i = 0; i < bodyA.getNodeCount(); ++i) {
    expectVectorNear(bodyA.getPosition(i), bodyB.getPosition(i));
    expectVectorNear(bodyA.getVelocity(i), bodyB.getVelocity(i));
  }
}

//==============================================================================
TEST(DeformableBody, SerializationPreservesModelAndState)
{
  sx::World world1;
  auto body1 = world1.addDeformableBody("serialized", makeTwoNodeBody());
  body1.setVelocity(1, Eigen::Vector3d(0.25, 0.0, 0.0));

  std::stringstream stream;
  world1.saveBinary(stream);

  sx::World world2;
  world2.loadBinary(stream);

  ASSERT_EQ(world2.getDeformableBodyCount(), 1u);
  auto body2 = world2.getDeformableBody("serialized");
  ASSERT_TRUE(body2.has_value());
  ASSERT_EQ(body2->getNodeCount(), 2u);
  EXPECT_TRUE(body2->isFixedNode(0));
  expectVectorNear(body2->getPosition(1), Eigen::Vector3d::UnitX());
  expectVectorNear(body2->getVelocity(1), Eigen::Vector3d(0.25, 0.0, 0.0));
  ASSERT_EQ(body2->getEdgeCount(), 1u);
  EXPECT_DOUBLE_EQ(body2->getEdge(0).restLength, 1.0);
}

//==============================================================================
TEST(DeformableBody, SerializationPreservesMeshTopologyAndMaterial)
{
  sx::World world1;
  auto options = makeSingleTetrahedronBody();
  options.material.density = 12.0;
  options.material.youngsModulus = 2500.0;
  options.material.poissonRatio = 0.25;
  auto body1 = world1.addDeformableBody("serialized_mesh", options);
  body1.setVelocity(3, Eigen::Vector3d(0.0, 0.0, 0.5));

  std::stringstream stream;
  world1.saveBinary(stream);

  sx::World world2;
  world2.loadBinary(stream);

  auto body2 = world2.getDeformableBody("serialized_mesh");
  ASSERT_TRUE(body2.has_value());
  ASSERT_EQ(body2->getNodeCount(), 4u);
  ASSERT_EQ(body2->getSurfaceTriangleCount(), 4u);
  ASSERT_EQ(body2->getTetrahedronCount(), 1u);
  EXPECT_DOUBLE_EQ(body2->getTetrahedronRestVolume(0), 1.0 / 6.0);
  for (std::size_t i = 0; i < body2->getNodeCount(); ++i) {
    EXPECT_DOUBLE_EQ(body2->getMass(i), 0.5);
  }

  const auto material = body2->getMaterialProperties();
  EXPECT_DOUBLE_EQ(material.density, 12.0);
  EXPECT_DOUBLE_EQ(material.youngsModulus, 2500.0);
  EXPECT_DOUBLE_EQ(material.poissonRatio, 0.25);
  expectVectorNear(body2->getVelocity(3), Eigen::Vector3d(0.0, 0.0, 0.5));

  world2.setTimeStep(0.05);
  world2.step();
  EXPECT_LT(body2->getPosition(3).z(), 1.0 + 0.05 * 0.5);
}
