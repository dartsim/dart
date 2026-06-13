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

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/deformable_body.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/comps/deformable_body.hpp>
#include <dart/simulation/compute/sequential_executor.hpp>
#include <dart/simulation/compute/world_step_stage.hpp>
#include <dart/simulation/io/binary_io.hpp>
#include <dart/simulation/world.hpp>

#include <dart/collision/native/narrow_phase/primitive_ccd.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <limits>
#include <numbers>
#include <sstream>
#include <string>
#include <string_view>

#include <cmath>
#include <cstring>

namespace sx = dart::simulation;
namespace comps = dart::simulation::comps;
namespace compute = dart::simulation::compute;
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

//==============================================================================
sx::RigidBody addStaticSurfaceCcdBox(
    sx::World& world,
    const std::string& name,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& halfExtents,
    const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity())
{
  sx::RigidBodyOptions options;
  options.isStatic = true;
  options.position = position;
  options.orientation = orientation;
  auto body = world.addRigidBody(name, options);
  body.setCollisionShape(sx::CollisionShape::makeBox(halfExtents));
  body.setDeformableSurfaceCcdObstacle(true);
  return body;
}

//==============================================================================
class MoveRigidBodyStage final : public compute::WorldStepStage
{
public:
  MoveRigidBodyStage(sx::RigidBody body, const Eigen::Isometry3d& transform)
    : m_body(body), m_transform(transform)
  {
  }

  [[nodiscard]] std::string_view getName() const noexcept override
  {
    return "move_rigid_body";
  }

  [[nodiscard]] compute::ComputeStageMetadata getMetadata()
      const noexcept override
  {
    return {compute::ComputeStageDomain::Kinematics};
  }

  void execute(
      sx::World& /*world*/, compute::ComputeExecutor& /*executor*/) override
  {
    m_body.setTransform(m_transform);
  }

private:
  sx::RigidBody m_body;
  Eigen::Isometry3d m_transform;
};

//==============================================================================
// Adds a non-static (free) rigid box tagged as a deformable-surface CCD
// obstacle with a prescribed world-frame velocity. The deformable stage
// predicts this body's end-of-step pose from its velocity, so the moving CCD
// limiter constrains the deformable against where the box will be.
sx::RigidBody addMovingSurfaceCcdBox(
    sx::World& world,
    const std::string& name,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& halfExtents,
    const Eigen::Vector3d& linearVelocity,
    const Eigen::Vector3d& angularVelocity = Eigen::Vector3d::Zero(),
    const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity())
{
  sx::RigidBodyOptions options;
  options.isStatic = false;
  options.position = position;
  options.orientation = orientation;
  auto body = world.addRigidBody(name, options);
  body.setCollisionShape(sx::CollisionShape::makeBox(halfExtents));
  body.setDeformableSurfaceCcdObstacle(true);
  body.setLinearVelocity(linearVelocity);
  body.setAngularVelocity(angularVelocity);
  return body;
}

//==============================================================================
// True when `point` lies strictly outside the axis-or-oriented box centered at
// `center` with the given orientation and half extents (i.e. positive
// separation on at least one axis in the box's local frame).
bool isOutsideBox(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& center,
    const Eigen::Quaterniond& orientation,
    const Eigen::Vector3d& halfExtents)
{
  const Eigen::Vector3d local = orientation.conjugate() * (point - center);
  return (local.array().abs() > halfExtents.array()).any();
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
// A single-tetrahedron body that opts in to stable neo-Hookean FEM elasticity
// (no spring edges, so elasticity comes entirely from the FEM term).
sx::DeformableBodyOptions makeFemTetrahedronBody(double youngsModulus = 1.0e5)
{
  sx::DeformableBodyOptions options = makeSingleTetrahedronBody();
  options.material.youngsModulus = youngsModulus;
  options.material.poissonRatio = 0.3;
  options.material.useFiniteElementElasticity = true;
  options.fixedNodes = {0};
  return options;
}

//==============================================================================
// With FEM elasticity opted in, a tetrahedron pinned at one node hangs from
// that node and settles at a bounded deflection under gravity: the stiff
// material resists deformation, so the free nodes do not run away. The same
// body WITHOUT FEM (and without spring edges) has no elastic force, so its free
// nodes free-fall far below. The large gap between the two is the FEM term
// doing work, and confirms it is strictly opt-in.
TEST(DeformableBody, FemTetrahedronResistsGravityWhereSpringlessBodyFreeFalls)
{
  const auto runFreeNodeDrop = [](bool useFem) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);
    sx::DeformableBodyOptions options = makeSingleTetrahedronBody();
    options.fixedNodes = {0};
    options.material.useFiniteElementElasticity = useFem;
    auto body = world.addDeformableBody("tet", options);
    world.step(200);
    // Node 0 is pinned in both cases.
    EXPECT_LT((body.getPosition(0) - Eigen::Vector3d::Zero()).norm(), 1e-9);
    double minZ = 0.0;
    for (int i = 1; i < 4; ++i) {
      minZ = std::min(minZ, body.getPosition(i).z());
      EXPECT_TRUE(body.getPosition(i).allFinite());
    }
    return minZ;
  };

  const double femMinZ = runFreeNodeDrop(/*useFem=*/true);
  const double springlessMinZ = runFreeNodeDrop(/*useFem=*/false);

  // FEM holds the hanging tetrahedron near its rest extent ...
  EXPECT_GT(femMinZ, -3.0);
  // ... while without any elastic force the free nodes have fallen far.
  EXPECT_LT(springlessMinZ, -15.0);
  EXPECT_LT(springlessMinZ, femMinZ - 10.0);
}

//==============================================================================
// At its rest shape with no external load, a FEM tetrahedron stores zero strain
// energy and exerts zero force, so it stays put across many steps.
TEST(DeformableBody, FemTetrahedronIsStationaryAtRest)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.02);
  auto body = world.addDeformableBody("fem_rest", makeFemTetrahedronBody());

  world.step(25);

  expectVectorNear(body.getPosition(0), Eigen::Vector3d::Zero());
  expectVectorNear(body.getPosition(1), Eigen::Vector3d::UnitX());
  expectVectorNear(body.getPosition(2), Eigen::Vector3d::UnitY());
  expectVectorNear(body.getPosition(3), Eigen::Vector3d::UnitZ());
}

//==============================================================================
// World exposes a curated snapshot of the deformable solver's per-step
// diagnostics from the most recent built-in-pipeline step. Before stepping it
// is zero; after a step it reflects the mesh and the projected-Newton solve.
TEST(DeformableBody, ExposesDeformableSolverDiagnostics)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);
  world.addDeformableBody("fem", makeFemTetrahedronBody());

  const auto& before = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(before.bodyCount, 0u);
  EXPECT_EQ(before.nodeCount, 0u);
  EXPECT_EQ(before.solverIterations, 0u);
  EXPECT_EQ(before.projectedNewtonHessianNonZeros, 0u);
  EXPECT_EQ(before.projectedNewtonHessianStorageBytes, 0u);
  EXPECT_EQ(before.projectedNewtonMatrixFreeSolves, 0u);
  EXPECT_EQ(before.surfaceContactCandidateBuilds, 0u);
  EXPECT_EQ(before.surfaceContactCcdHits, 0u);
  EXPECT_EQ(before.surfaceContactCcdLimitedSteps, 0u);
  EXPECT_EQ(before.interBodySurfaceContactCandidateBuilds, 0u);
  EXPECT_EQ(before.staticRigidSurfaceCcdCandidateBuilds, 0u);
  EXPECT_EQ(before.movingRigidSurfaceCcdCandidateBuilds, 0u);

  world.step(5);

  const auto& after = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(after.bodyCount, 1u);
  EXPECT_EQ(after.nodeCount, 4u);
  EXPECT_GE(after.solverIterations, 1u);
  EXPECT_GE(after.objectiveEvaluations, 1u);
  EXPECT_GE(after.projectedNewtonSteps + after.projectedNewtonFallbacks, 1u);
  EXPECT_GT(after.projectedNewtonHessianNonZeros, 0u);
  EXPECT_GT(after.projectedNewtonHessianStorageBytes, 0u);
  // No contacts for a single free-hanging tetrahedron.
  EXPECT_EQ(after.selfContactBarrierActiveContacts, 0u);
  EXPECT_EQ(after.convergedActiveContactCount, 0u);

  world.clear();
  const auto& reset = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(reset.bodyCount, 0u);
  EXPECT_EQ(reset.nodeCount, 0u);
  EXPECT_EQ(reset.solverIterations, 0u);
  EXPECT_EQ(reset.projectedNewtonHessianNonZeros, 0u);
  EXPECT_EQ(reset.projectedNewtonHessianStorageBytes, 0u);
  EXPECT_EQ(reset.surfaceContactCandidateBuilds, 0u);
  EXPECT_EQ(reset.surfaceContactCcdHits, 0u);
  EXPECT_EQ(reset.surfaceContactCcdLimitedSteps, 0u);
  EXPECT_EQ(reset.interBodySurfaceContactCandidateBuilds, 0u);
  EXPECT_EQ(reset.staticRigidSurfaceCcdCandidateBuilds, 0u);
  EXPECT_EQ(reset.movingRigidSurfaceCcdCandidateBuilds, 0u);
}

//==============================================================================
// Replay frames captured by built-in steps include the diagnostics folded from
// that same step, so restoring a recorded post-step frame restores solver state
// rather than the previous frame's diagnostics.
TEST(DeformableBody, ReplayRecordingRestoresPostStepDiagnostics)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);
  world.addDeformableBody("fem", makeFemTetrahedronBody());

  world.setReplayRecordingEnabled(true);
  world.step();

  const auto expected = world.getLastDeformableSolverDiagnostics();
  ASSERT_EQ(expected.bodyCount, 1u);
  ASSERT_EQ(expected.nodeCount, 4u);
  ASSERT_GE(expected.solverIterations, 1u);

  ASSERT_EQ(world.getReplayFrameCount(), 2u);
  world.restoreReplayFrame(0);
  ASSERT_EQ(world.getLastDeformableSolverDiagnostics().bodyCount, 0u);

  world.restoreReplayFrame(1);
  const auto& restored = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(restored.bodyCount, expected.bodyCount);
  EXPECT_EQ(restored.nodeCount, expected.nodeCount);
  EXPECT_EQ(restored.solverIterations, expected.solverIterations);
  EXPECT_EQ(restored.objectiveEvaluations, expected.objectiveEvaluations);
}

//==============================================================================
// The isolated third node contributes only its three inertial diagonal entries
// to the sparse projected-Newton Hessian; the spring-connected pair contributes
// a full 6x6 block that drives the solve.
TEST(DeformableBody, SparseInertiaAssemblyOmitsExplicitZeroEntries)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);

  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::UnitX(),
         Eigen::Vector3d::UnitY()};
  options.velocities
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero()};
  options.masses = {1.0, 2.0, 3.0};
  options.edges = {sx::DeformableEdge{0, 1}};
  world.addDeformableBody("spring_with_isolated_node", options);

  world.step(1);

  const auto& after = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(after.bodyCount, 1u);
  EXPECT_EQ(after.nodeCount, 3u);
  EXPECT_EQ(after.projectedNewtonHessianNonZeros, 39u);
  EXPECT_GT(after.projectedNewtonHessianStorageBytes, 0u);
}

//==============================================================================
// The public solver diagnostics expose which linear-solve path each Newton
// iteration took: the default direct (sparse Cholesky) solve never reports an
// iterative solve, while a body opting in to the iterative
// (incomplete-Cholesky-preconditioned CG) solve surfaces a nonzero solve count,
// CG iteration count, and finite residual estimate. These are the public-API
// mirrors of the internal iterative-solver stats, so Python callers can observe
// and tune the solver path.
TEST(DeformableBody, DiagnosticsExposeIterativeSolveCount)
{
  struct IterativeDiagnostics
  {
    std::size_t solves = 0;
    std::size_t matrixFreeSolves = 0;
    std::size_t iterations = 0;
    std::size_t hessianNonZeros = 0;
    std::size_t hessianStorageBytes = 0;
    double maxError = 0.0;
  };

  const auto run = [](bool iterative) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);
    auto options = makeFemTetrahedronBody();
    options.material.useIterativeLinearSolver = iterative;
    world.addDeformableBody("fem", options);

    IterativeDiagnostics total;
    for (int i = 0; i < 8; ++i) {
      world.step(1);
      const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
      total.solves += diagnostics.projectedNewtonIterativeSolves;
      total.matrixFreeSolves += diagnostics.projectedNewtonMatrixFreeSolves;
      total.iterations += diagnostics.projectedNewtonIterativeIterations;
      total.hessianNonZeros = std::max(
          total.hessianNonZeros, diagnostics.projectedNewtonHessianNonZeros);
      total.hessianStorageBytes = std::max(
          total.hessianStorageBytes,
          diagnostics.projectedNewtonHessianStorageBytes);
      total.maxError = std::max(
          total.maxError, diagnostics.projectedNewtonIterativeMaxError);
    }
    return total;
  };

  const auto direct = run(false);
  const auto iterative = run(true);

  // The default direct solve never takes the iterative path...
  EXPECT_EQ(direct.solves, 0u);
  EXPECT_EQ(direct.matrixFreeSolves, 0u);
  EXPECT_EQ(direct.iterations, 0u);
  EXPECT_GT(direct.hessianNonZeros, 0u);
  EXPECT_GT(direct.hessianStorageBytes, 0u);
  EXPECT_EQ(direct.maxError, 0.0);
  // ...while the opt-in iterative solve surfaces through the public diagnostics
  // with CG effort, sparse-Hessian footprint, and a finite residual estimate.
  EXPECT_GT(iterative.solves, 0u);
  EXPECT_EQ(iterative.matrixFreeSolves, 0u);
  EXPECT_GT(iterative.hessianNonZeros, 0u);
  EXPECT_GT(iterative.hessianStorageBytes, 0u);
  EXPECT_TRUE(std::isfinite(iterative.maxError));
  EXPECT_GE(iterative.maxError, 0.0);
}

//==============================================================================
// A FEM tetrahedron given an initial outward velocity on a free node is pulled
// back toward its rest shape by the elastic restoring force, and the implicit
// solve dissipates the motion so it settles near rest rather than diverging.
// A softer material keeps the transient displacement clearly visible.
TEST(DeformableBody, FemTetrahedronRestoresStretchedNodeTowardRest)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);
  auto options = makeFemTetrahedronBody(/*youngsModulus=*/1.0e3);
  options.fixedNodes = {0, 1, 2};
  options.velocities
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d(0.0, 0.0, 10.0)};
  auto body = world.addDeformableBody("fem_stretch", options);

  // The driven node is clearly displaced from its rest position (z = 1) by the
  // initial velocity before the elastic force arrests it.
  double peakDisplacement = 0.0;
  for (int step = 0; step < 60; ++step) {
    world.step();
    peakDisplacement
        = std::max(peakDisplacement, std::abs(body.getPosition(3).z() - 1.0));
    EXPECT_TRUE(body.getPosition(3).allFinite());
  }
  EXPECT_GT(peakDisplacement, 0.02);

  // After many steps the elastic restoring force + implicit dissipation settle
  // the free node back near its rest position (0, 0, 1); a body with no elastic
  // force would never return.
  world.step(400);
  EXPECT_TRUE(body.getPosition(3).allFinite());
  expectVectorNear(body.getPosition(3), Eigen::Vector3d(0.0, 0.0, 1.0), 5e-2);
}

//==============================================================================
// The fixed-corotational material is selected by an additional opt-in flag on
// top of useFiniteElementElasticity. At its rest shape it stores zero strain
// energy and exerts zero force, so the tetrahedron stays put across many steps,
// exactly like the default stable neo-Hookean kernel.
TEST(DeformableBody, FemFixedCorotationalTetrahedronIsStationaryAtRest)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.02);
  sx::DeformableBodyOptions options = makeFemTetrahedronBody();
  options.material.useFixedCorotationalElasticity = true;
  auto body = world.addDeformableBody("fcr_rest", options);

  world.step(25);

  expectVectorNear(body.getPosition(0), Eigen::Vector3d::Zero());
  expectVectorNear(body.getPosition(1), Eigen::Vector3d::UnitX());
  expectVectorNear(body.getPosition(2), Eigen::Vector3d::UnitY());
  expectVectorNear(body.getPosition(3), Eigen::Vector3d::UnitZ());
}

//==============================================================================
// A fixed-corotational FEM tetrahedron given an outward velocity on a free node
// is pulled back toward its rest shape and settles there, confirming the FCR
// dispatch path is wired through both the energy/gradient objective and the
// projected-Newton Hessian assembly.
TEST(DeformableBody, FemFixedCorotationalRestoresStretchedNodeTowardRest)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);
  auto options = makeFemTetrahedronBody(/*youngsModulus=*/1.0e3);
  options.material.useFixedCorotationalElasticity = true;
  options.fixedNodes = {0, 1, 2};
  options.velocities
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d(0.0, 0.0, 10.0)};
  auto body = world.addDeformableBody("fcr_stretch", options);

  double peakDisplacement = 0.0;
  for (int step = 0; step < 60; ++step) {
    world.step();
    peakDisplacement
        = std::max(peakDisplacement, std::abs(body.getPosition(3).z() - 1.0));
    EXPECT_TRUE(body.getPosition(3).allFinite());
  }
  EXPECT_GT(peakDisplacement, 0.02);

  world.step(400);
  EXPECT_TRUE(body.getPosition(3).allFinite());
  expectVectorNear(body.getPosition(3), Eigen::Vector3d(0.0, 0.0, 1.0), 5e-2);
}

//==============================================================================
// A free FEM cube (one hexahedral cell split into six tetrahedra) opting in to
// stable neo-Hookean elasticity, released above the ground top.
sx::DeformableBodyOptions makeFemCubeBody(
    double size, const Eigen::Vector3d& origin, double youngsModulus)
{
  sx::DeformableBodyOptions options;
  for (int corner = 0; corner < 8; ++corner) {
    options.positions.push_back(
        origin
        + size
              * Eigen::Vector3d(
                  corner & 1, (corner >> 1) & 1, (corner >> 2) & 1));
  }
  // Kuhn six-tetrahedron decomposition of the cell along the 0->7 diagonal.
  const std::array<std::array<std::size_t, 4>, 6> tets = {{
      {0, 1, 3, 7},
      {0, 3, 2, 7},
      {0, 2, 6, 7},
      {0, 6, 4, 7},
      {0, 4, 5, 7},
      {0, 5, 1, 7},
  }};
  for (const auto& tet : tets) {
    options.tetrahedra.push_back(
        sx::DeformableTetrahedron{tet[0], tet[1], tet[2], tet[3]});
  }
  options.material.youngsModulus = youngsModulus;
  options.material.poissonRatio = 0.3;
  options.material.useFiniteElementElasticity = true;
  return options;
}

//==============================================================================
// A volumetric FEM cube dropped onto a static ground barrier settles on the
// barrier surface intersection-free: gravity pulls it down, the IPC clamped-log
// ground barrier catches it (no node crosses the ground top), and the stable
// neo-Hookean elasticity keeps the cube finite as it squashes and rests. This
// exercises FEM elasticity and barrier contact together in one solve.
TEST(DeformableBody, FemCubeSettlesOnGroundBarrierWithoutPenetrating)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.004);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
  ground.setDeformableGroundBarrier(true); // top face at z = 0

  auto body = world.addDeformableBody(
      "fem_cube", makeFemCubeBody(0.2, Eigen::Vector3d(0.0, 0.0, 0.3), 1.5e5));

  const auto minNodeZ = [&]() {
    double minimum = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < body.getNodeCount(); ++i) {
      minimum = std::min(minimum, body.getPosition(i).z());
    }
    return minimum;
  };

  ASSERT_GT(minNodeZ(), 0.25);
  world.step(250);

  // The cube has fallen onto the barrier (well below its release height) ...
  EXPECT_LT(minNodeZ(), 0.1);
  // ... but no node has crossed the ground top, and everything stays finite.
  for (std::size_t i = 0; i < body.getNodeCount(); ++i) {
    EXPECT_TRUE(body.getPosition(i).allFinite());
    EXPECT_GE(body.getPosition(i).z(), -1e-3);
  }
}

//==============================================================================
// A FEM cube dropped onto a static sphere obstacle settles against the curved
// surface intersection-free: the radial clamped-log obstacle barrier (now a
// projected-Newton term) keeps every node outside the sphere while the stable
// neo-Hookean elasticity conforms the cube to the obstacle, all finite. This
// exercises FEM elasticity and the sphere obstacle barrier (energy, gradient,
// and Hessian) together.
TEST(DeformableBody, FemCubeSettlesOnSphereObstacleWithoutPenetrating)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.004);

  const Eigen::Vector3d sphereCenter(0.1, 0.1, 0.0);
  const double sphereRadius = 0.5;
  sx::RigidBodyOptions sphereOptions;
  sphereOptions.isStatic = true;
  sphereOptions.position = sphereCenter;
  auto sphere = world.addRigidBody("obstacle_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(sphereRadius));
  sphere.setDeformableSurfaceCcdObstacle(true);

  // A small FEM cube released just above the sphere's top.
  auto body = world.addDeformableBody(
      "fem_cube",
      makeFemCubeBody(
          0.16, Eigen::Vector3d(0.02, 0.02, 0.62), /*youngsModulus=*/2.0e5));

  const auto minSurfaceDistance = [&]() {
    double minimum = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < body.getNodeCount(); ++i) {
      minimum = std::min(
          minimum, (body.getPosition(i) - sphereCenter).norm() - sphereRadius);
    }
    return minimum;
  };

  ASSERT_GT(minSurfaceDistance(), 0.05);
  world.step(250);

  // The cube has fallen onto the sphere (well below its release height) ...
  double minZ = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < body.getNodeCount(); ++i) {
    minZ = std::min(minZ, body.getPosition(i).z());
  }
  EXPECT_LT(minZ, 0.55);
  // ... but no node penetrates the sphere surface, and all stay finite.
  EXPECT_GT(minSurfaceDistance(), -1e-3);
  for (std::size_t i = 0; i < body.getNodeCount(); ++i) {
    EXPECT_TRUE(body.getPosition(i).allFinite());
  }
}

//==============================================================================
// A static box opted in as a deformable obstacle exerts a clamped-log barrier
// along the outward surface normal: a node resting just outside a box face (in
// the activation band) is pushed straight out along that face normal.
TEST(DeformableBody, BoxObstacleBarrierRepelsNodeAlongFaceNormal)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions boxOptions;
  boxOptions.isStatic = true;
  auto box = world.addRigidBody("obstacle_box", boxOptions);
  box.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  box.setDeformableSurfaceCcdObstacle(true);

  // 0.505 is inside the band off the +x face (surface 0.5, distance 0.005).
  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.505, 0.0, 0.0)};
  options.velocities = {Eigen::Vector3d::Zero()};
  auto body = world.addDeformableBody("node", options);

  world.step(10);

  const auto position = body.getPosition(0);
  EXPECT_GT(position.x(), 0.52);        // repelled past the band edge along +x
  EXPECT_LT(position.x(), 2.0);         // finite (no blow-up)
  EXPECT_NEAR(position.y(), 0.0, 1e-9); // purely along the face normal
  EXPECT_NEAR(position.z(), 0.0, 1e-9);
}

//==============================================================================
// A static capsule (z-axis rod) opted in as a deformable obstacle radially
// repels a nearby node along the outward normal from the capsule axis -- the
// codimensional (rod) analogue of the sphere/box obstacle barriers.
TEST(DeformableBody, CapsuleObstacleBarrierRepelsNodeRadially)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions capsuleOptions;
  capsuleOptions.isStatic = true;
  auto capsule = world.addRigidBody("obstacle_capsule", capsuleOptions);
  // A z-axis capsule: radius 0.3, half-height 0.4 (so the cylindrical side
  // spans z in [-0.4, 0.4]).
  capsule.setCollisionShape(
      sx::CollisionShape::makeCapsule(/*radius=*/0.3, /*halfHeight=*/0.4));
  capsule.setDeformableSurfaceCcdObstacle(true);

  // 0.305 is inside the band off the cylindrical side (surface 0.3, distance
  // 0.005), at z = 0 (mid-segment).
  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.305, 0.0, 0.0)};
  options.velocities = {Eigen::Vector3d::Zero()};
  auto body = world.addDeformableBody("node", options);

  world.step(10);

  const auto position = body.getPosition(0);
  EXPECT_GT(position.x(), 0.32);        // repelled past the band edge along +x
  EXPECT_LT(position.x(), 2.0);         // finite (no blow-up)
  EXPECT_NEAR(position.y(), 0.0, 1e-9); // purely along the radial normal
  EXPECT_NEAR(position.z(), 0.0, 1e-9); // mid-segment: no axial push
}

//==============================================================================
// Lagged Coulomb friction works against the (barrier-only, CCD-free) capsule
// rod obstacle: a node resting on top of a horizontal rod and pushed along its
// axis slides measurably less the larger the friction coefficient, while
// staying on the rod surface. (Mesh-vs-obstacle friction, PLAN-081 M5, is
// unblocked for the capsule because it carries no over-limiting surface CCD.)
TEST(DeformableBody, CapsuleObstacleFrictionDeceleratesSlidingNode)
{
  const auto slideAlongRod = [](double frictionCoefficient) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.004);

    // A horizontal rod: the capsule axis (body z) laid along world y.
    sx::RigidBodyOptions rodOptions;
    rodOptions.isStatic = true;
    rodOptions.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(-std::numbers::pi / 2.0, Eigen::Vector3d::UnitX()));
    auto rod = world.addRigidBody("rod", rodOptions);
    constexpr double radius = 0.2;
    // The rod is long enough that the frictionless node stays on the
    // cylindrical side over the test rather than sliding off an end cap.
    rod.setCollisionShape(
        sx::CollisionShape::makeCapsule(radius, /*halfHeight=*/3.0));
    rod.setDeformableSurfaceCcdObstacle(true);

    // A node resting on top of the rod (inside the radial band), pushed along
    // the rod's axis (+y).
    sx::DeformableBodyOptions options;
    options.positions = {Eigen::Vector3d(0.0, 0.0, radius + 0.012)};
    options.velocities = {Eigen::Vector3d(0.0, 2.0, 0.0)};
    options.material.frictionCoefficient = frictionCoefficient;
    auto body = world.addDeformableBody("slider", options);

    world.step(200);
    return body.getPosition(0);
  };

  const Eigen::Vector3d frictionless = slideAlongRod(0.0);
  const Eigen::Vector3d highFriction = slideAlongRod(0.8);

  // Both stay on the rod surface (radius 0.2, band d_hat = 2e-2) and finite.
  EXPECT_GT(frictionless.z(), 0.2);
  EXPECT_LT(frictionless.z(), 0.24);
  EXPECT_GT(highFriction.z(), 0.2);
  EXPECT_TRUE(highFriction.allFinite());
  // The frictionless node slides far along the axis; friction holds it back.
  EXPECT_GT(frictionless.y(), 1.0);
  EXPECT_LT(highFriction.y(), 0.5 * frictionless.y());
}

//==============================================================================
// A box obstacle opted into barrier-only mode (excluded from the surface CCD
// limiter) lets a node slide tangentially across its top face, so Coulomb
// friction can decelerate the slide -- the CCD-free path to sphere/box obstacle
// friction (PLAN-081 M5). A node shoved across a wide barrier-only box slides
// far frictionless but is held back under friction, staying above the top face.
TEST(DeformableBody, BarrierOnlyBoxObstacleFrictionDeceleratesSlidingNode)
{
  const auto slideAcrossBox = [](double frictionCoefficient) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.004);

    sx::RigidBodyOptions boxOptions;
    boxOptions.isStatic = true;
    auto box = world.addRigidBody("box", boxOptions);
    // A wide plate (top face at z = 0.5) so the node stays on top.
    box.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 1.0, 0.5)));
    box.setDeformableSurfaceCcdObstacle(true);
    box.setDeformableObstacleBarrierOnly(true);
    EXPECT_TRUE(box.isDeformableObstacleBarrierOnly());

    sx::DeformableBodyOptions options;
    options.positions = {Eigen::Vector3d(-0.8, 0.0, 0.512)};
    options.velocities = {Eigen::Vector3d(2.0, 0.0, 0.0)};
    options.material.frictionCoefficient = frictionCoefficient;
    auto body = world.addDeformableBody("slider", options);

    world.step(200);
    return body.getPosition(0);
  };

  const Eigen::Vector3d frictionless = slideAcrossBox(0.0);
  const Eigen::Vector3d highFriction = slideAcrossBox(0.8);

  // Both stay above the top face (z = 0.5, band d_hat = 2e-2) and finite.
  EXPECT_GT(frictionless.z(), 0.5);
  EXPECT_LT(frictionless.z(), 0.54);
  EXPECT_GT(highFriction.z(), 0.5);
  EXPECT_TRUE(highFriction.allFinite());
  // The frictionless node slides far across the face; friction holds it back.
  EXPECT_GT(frictionless.x() + 0.8, 1.0); // started at x = -0.8
  EXPECT_LT(highFriction.x() - (-0.8), 0.5 * (frictionless.x() - (-0.8)));
}

//==============================================================================
// A FEM cube dropped onto a static box obstacle settles on the surface
// intersection-free: the box obstacle barrier (energy, gradient, and Hessian)
// keeps every node outside the box while the FEM elasticity conforms the cube
// to the obstacle, all finite.
TEST(DeformableBody, FemCubeSettlesOnBoxObstacleWithoutPenetrating)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.004);

  const Eigen::Vector3d boxCenter(0.0, 0.0, 0.0);
  const Eigen::Vector3d boxHalf(0.3, 0.3, 0.3); // top face at z = 0.3
  sx::RigidBodyOptions boxOptions;
  boxOptions.isStatic = true;
  boxOptions.position = boxCenter;
  auto box = world.addRigidBody("obstacle_box", boxOptions);
  box.setCollisionShape(sx::CollisionShape::makeBox(boxHalf));
  box.setDeformableSurfaceCcdObstacle(true);

  auto body = world.addDeformableBody(
      "fem_cube",
      makeFemCubeBody(
          0.16, Eigen::Vector3d(-0.06, -0.06, 0.5), /*youngsModulus=*/2.0e5));

  const auto minBoxSurfaceDistance = [&]() {
    double minimum = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < body.getNodeCount(); ++i) {
      const Eigen::Vector3d local = body.getPosition(i) - boxCenter;
      const Eigen::Vector3d clamped
          = local.cwiseMax(-boxHalf).cwiseMin(boxHalf);
      minimum = std::min(minimum, (local - clamped).norm());
    }
    return minimum;
  };

  ASSERT_GT(minBoxSurfaceDistance(), 0.05);
  world.step(250);

  double minZ = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < body.getNodeCount(); ++i) {
    minZ = std::min(minZ, body.getPosition(i).z());
    EXPECT_TRUE(body.getPosition(i).allFinite());
  }
  EXPECT_LT(minZ, 0.45);                     // fell onto the box
  EXPECT_GE(minBoxSurfaceDistance(), -1e-3); // no node inside the box
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
// Opt-in adaptive barrier stiffness scales the ground barrier with the node
// mass: a heavy node under gravity settles in the activation band where the
// barrier force balances gravity. The stiffer adaptive kappa balances at a
// larger distance from the surface than the fixed kappa, so the heavy node
// settles measurably higher (and both stay intersection-free above z = 0).
TEST(DeformableBody, AdaptiveBarrierStiffnessHoldsHeavyNodeFurtherFromGround)
{
  const auto settleHeavyNode = [](bool adaptive) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -0.5));
    world.setTimeStep(0.01);

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
    auto ground = world.addRigidBody("ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(10.0, 10.0, 0.5)));
    ground.setDeformableGroundBarrier(true); // top face at z = 0

    // A single heavy node starting inside the activation band (d_hat = 2e-2).
    // The mass (40) is heavy enough that the adaptive kappa is far above the
    // fixed default, so the two settle at clearly different distances.
    sx::DeformableBodyOptions options;
    options.positions = {Eigen::Vector3d(0.0, 0.0, 0.015)};
    options.masses = {40.0};
    options.material.useAdaptiveBarrierStiffness = adaptive;
    auto body = world.addDeformableBody("heavy_node", options);

    world.step(600);
    return body.getPosition(0).z();
  };

  const double fixedZ = settleHeavyNode(/*adaptive=*/false);
  const double adaptiveZ = settleHeavyNode(/*adaptive=*/true);

  // Both settle intersection-free inside the activation band.
  EXPECT_GT(fixedZ, 0.0);
  EXPECT_LT(fixedZ, 2e-2);
  EXPECT_GT(adaptiveZ, 0.0);
  EXPECT_LT(adaptiveZ, 2e-2);
  EXPECT_TRUE(std::isfinite(adaptiveZ));
  // The stiffer adaptive barrier holds the heavy node measurably higher.
  EXPECT_GT(adaptiveZ, fixedZ + 1e-3);
}

//==============================================================================
// A static sphere opted in as a surface-CCD obstacle exerts a full radial
// barrier force: a node resting inside the activation band (d_hat = 2e-2) at
// the sphere's side is pushed radially outward toward the band edge, not just
// along the vertical that the ground barrier handles. (The barrier is the
// smooth contact force; the surface CCD limiter is the tunnelling guard for
// fast motion.)
TEST(DeformableBody, SphereObstacleBarrierRepelsNodeRadially)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.isStatic = true;
  auto sphere = world.addRigidBody("obstacle_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  sphere.setDeformableSurfaceCcdObstacle(true);

  // 0.505 is inside the band (surface 0.5, distance 0.005 < d_hat 0.02).
  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.505, 0.0, 0.0)};
  options.velocities = {Eigen::Vector3d::Zero()};
  auto body = world.addDeformableBody("node", options);

  world.step(10);

  // The radial barrier drives the node out of the activation band along +x. A
  // vertical-only (ground-style) barrier could never produce this purely-radial
  // push, which is the point of the obstacle barrier.
  const auto position = body.getPosition(0);
  EXPECT_GT(position.x(), 0.52); // repelled past the band edge (radially)
  EXPECT_LT(position.x(), 2.0);  // finite (no blow-up)
  EXPECT_NEAR(position.y(), 0.0, 1e-9); // purely radial (+x), no deflection
  EXPECT_NEAR(position.z(), 0.0, 1e-9);
}

//==============================================================================
// Without the opt-in the same sphere is inert: a force-free node in the band
// does not move (no barrier force).
TEST(DeformableBody, SphereObstacleBarrierRequiresOptIn)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.isStatic = true;
  auto sphere = world.addRigidBody("ordinary_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  ASSERT_FALSE(sphere.isDeformableSurfaceCcdObstacle());

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.505, 0.0, 0.0)};
  options.velocities = {Eigen::Vector3d::Zero()};
  auto body = world.addDeformableBody("node", options);

  world.step(30);

  // No barrier force, so the static node stays put.
  EXPECT_NEAR(body.getPosition(0).x(), 0.505, 1e-6);
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
TEST(DeformableBody, StaticGroundBarrierCcdCatchesNarrowOffsetFootprint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.022, 0.0, -0.05);
  auto ground = world.addRigidBody("narrow_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.005, 0.5, 0.05)));
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
  EXPECT_LT(body.getPosition(0).x(), 0.017);
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
TEST(DeformableBody, SurfaceContactCcdReportsBuiltInWorldDiagnostics)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  auto body = world.addDeformableBody(
      "surface_crossing", makeSurfaceCrossingBodyOptions());

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_GT(body.getPosition(3).z(), 0.0);
  EXPECT_GT(diagnostics.surfaceContactCandidateBuilds, 0u);
  EXPECT_GT(diagnostics.surfaceContactPointTriangleCandidates, 0u);
  EXPECT_GT(diagnostics.surfaceContactCcdPointTriangleChecks, 0u);
  EXPECT_GT(diagnostics.surfaceContactCcdHits, 0u);
  EXPECT_GT(diagnostics.surfaceContactCcdLimitedSteps, 0u);
}

//==============================================================================
TEST(DeformableBody, InterBodySurfaceContactCcdReportsBuiltInWorldDiagnostics)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  auto moving
      = world.addDeformableBody("moving", makeInterBodyMovingPointOptions());
  world.addDeformableBody("obstacle", makeStationaryTriangleObstacleOptions());

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_GT(moving.getPosition(3).z(), 0.0);
  EXPECT_LT(moving.getPosition(3).z(), 1.0);
  EXPECT_GT(diagnostics.interBodySurfaceContactCandidateBuilds, 0u);
  EXPECT_GT(diagnostics.interBodySurfaceContactPointTriangleCandidates, 0u);
  EXPECT_GT(diagnostics.interBodySurfaceContactCcdPointTriangleChecks, 0u);
  EXPECT_GT(diagnostics.interBodySurfaceContactCcdHits, 0u);
  EXPECT_GT(diagnostics.interBodySurfaceContactCcdLimitedSteps, 0u);
  EXPECT_EQ(diagnostics.surfaceContactCcdLimitedSteps, 0u);
  EXPECT_EQ(diagnostics.staticRigidSurfaceCcdLimitedSteps, 0u);
  EXPECT_EQ(diagnostics.movingRigidSurfaceCcdLimitedSteps, 0u);
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
TEST(DeformableBody, StaticRigidSurfaceCcdLimitsPointOnlySideCrossing)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  addStaticSurfaceCcdBox(
      world,
      "static_box",
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.05, 1.0, 1.0));

  auto body = world.addDeformableBody(
      "fast_point",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(20.0, 0.0, 0.0)));

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_LT(body.getPosition(0).x(), -0.05);
  EXPECT_GT(body.getPosition(0).x(), -1.0);
  EXPECT_EQ(stats.staticRigidSurfaceCcdBoxCount, 1u);
  EXPECT_EQ(stats.staticRigidSurfaceCcdTriangleCount, 12u);
  EXPECT_EQ(stats.staticRigidSurfaceCcdEdgeCount, 12u);
  EXPECT_GT(stats.staticRigidSurfaceCcdCandidateBuilds, 0u);
  EXPECT_GT(stats.staticRigidSurfaceCcdPointTriangleCandidates, 0u);
  EXPECT_GT(stats.staticRigidSurfaceCcdPointTriangleChecks, 0u);
  EXPECT_GT(stats.staticRigidSurfaceCcdHits, 0u);
  EXPECT_GT(stats.staticRigidSurfaceCcdLimitedSteps, 0u);
  EXPECT_EQ(stats.surfaceContactCcdLimitedSteps, 0u);
  EXPECT_EQ(stats.interBodySurfaceContactCcdLimitedSteps, 0u);
}

//==============================================================================
TEST(DeformableBody, StaticRigidSurfaceCcdReportsBuiltInWorldDiagnostics)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  addStaticSurfaceCcdBox(
      world,
      "static_box",
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.05, 1.0, 1.0));

  auto body = world.addDeformableBody(
      "fast_point",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(20.0, 0.0, 0.0)));

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_LT(body.getPosition(0).x(), -0.05);
  EXPECT_GT(body.getPosition(0).x(), -1.0);
  EXPECT_GT(diagnostics.staticRigidSurfaceCcdSnapshotBuilds, 0u);
  EXPECT_EQ(diagnostics.staticRigidSurfaceCcdBoxCount, 1u);
  EXPECT_EQ(diagnostics.staticRigidSurfaceCcdTriangleCount, 12u);
  EXPECT_EQ(diagnostics.staticRigidSurfaceCcdEdgeCount, 12u);
  EXPECT_GT(diagnostics.staticRigidSurfaceCcdCandidateBuilds, 0u);
  EXPECT_GT(diagnostics.staticRigidSurfaceCcdPointTriangleCandidates, 0u);
  EXPECT_GT(diagnostics.staticRigidSurfaceCcdPointTriangleChecks, 0u);
  EXPECT_GT(diagnostics.staticRigidSurfaceCcdHits, 0u);
  EXPECT_GT(diagnostics.staticRigidSurfaceCcdLimitedSteps, 0u);
  EXPECT_EQ(diagnostics.surfaceContactCcdLimitedSteps, 0u);
  EXPECT_EQ(diagnostics.interBodySurfaceContactCcdLimitedSteps, 0u);
  EXPECT_EQ(diagnostics.movingRigidSurfaceCcdBoxCount, 0u);
}

//==============================================================================
TEST(DeformableBody, StaticRigidSurfaceCcdRequiresOptIn)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions boxOptions;
  boxOptions.isStatic = true;
  auto box = world.addRigidBody("ordinary_static_box", boxOptions);
  box.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.05, 1.0, 1.0)));
  ASSERT_FALSE(box.isDeformableSurfaceCcdObstacle());

  auto body = world.addDeformableBody(
      "fast_point",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(20.0, 0.0, 0.0)));

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_GT(body.getPosition(0).x(), 0.5);
  EXPECT_EQ(stats.staticRigidSurfaceCcdBoxCount, 0u);
  EXPECT_EQ(stats.staticRigidSurfaceCcdCandidateBuilds, 0u);
  EXPECT_EQ(stats.staticRigidSurfaceCcdHits, 0u);
}

//==============================================================================
// A static sphere opted in as a surface-CCD obstacle limits a fast deformable
// node before it can cross the sphere's surface (non-box rigid surface
// coverage). The sphere is tessellated into a conservatively circumscribed
// triangle mesh that reuses the same point-triangle / edge-edge CCD limiter.
TEST(DeformableBody, StaticRigidSurfaceCcdLimitsAgainstSphereObstacle)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.isStatic = true;
  auto sphere = world.addRigidBody("static_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  sphere.setDeformableSurfaceCcdObstacle(true);

  auto body = world.addDeformableBody(
      "fast_point",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(20.0, 0.0, 0.0)));

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  // Stopped at/just before the sphere's near surface (x = -0.5), never pushed
  // through, and still advanced from its start (x = -1.0).
  EXPECT_LT(body.getPosition(0).x(), -0.5);
  EXPECT_GT(body.getPosition(0).x(), -1.0);
  EXPECT_EQ(stats.staticRigidSurfaceCcdSphereCount, 1u);
  EXPECT_EQ(stats.staticRigidSurfaceCcdBoxCount, 0u);
  EXPECT_GT(stats.staticRigidSurfaceCcdTriangleCount, 0u);
  EXPECT_GT(stats.staticRigidSurfaceCcdEdgeCount, 0u);
  EXPECT_GT(stats.staticRigidSurfaceCcdHits, 0u);
  EXPECT_GT(stats.staticRigidSurfaceCcdLimitedSteps, 0u);
}

//==============================================================================
// An untagged static sphere is not a surface-CCD obstacle, so a fast node
// passes through it (mirrors StaticRigidSurfaceCcdRequiresOptIn for spheres).
TEST(DeformableBody, StaticRigidSurfaceCcdSphereRequiresOptIn)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.isStatic = true;
  auto sphere = world.addRigidBody("ordinary_static_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  ASSERT_FALSE(sphere.isDeformableSurfaceCcdObstacle());

  auto body = world.addDeformableBody(
      "fast_point",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(20.0, 0.0, 0.0)));

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_GT(body.getPosition(0).x(), 0.5);
  EXPECT_EQ(stats.staticRigidSurfaceCcdSphereCount, 0u);
  EXPECT_EQ(stats.staticRigidSurfaceCcdHits, 0u);
}

//==============================================================================
TEST(DeformableBody, StaticRigidSurfaceCcdChecksPhysicalBoxEdges)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  addStaticSurfaceCcdBox(
      world,
      "thin_box",
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.5, 0.05, 0.5));

  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-0.5, -1.0, -0.5),
         Eigen::Vector3d(-0.5, -1.0, 0.5),
         Eigen::Vector3d(-0.7, -1.0, 0.0)};
  options.velocities
      = {Eigen::Vector3d(0.0, 20.0, 0.0),
         Eigen::Vector3d(0.0, 20.0, 0.0),
         Eigen::Vector3d(0.0, 20.0, 0.0)};
  options.masses = {1.0, 1.0, 1.0};
  options.surfaceTriangles = {sx::DeformableSurfaceTriangle{0, 1, 2}};
  auto body = world.addDeformableBody("moving_edge", options);

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_LT(body.getPosition(0).y(), -0.05);
  EXPECT_GT(stats.staticRigidSurfaceCcdEdgeEdgeCandidates, 0u);
  EXPECT_GT(stats.staticRigidSurfaceCcdEdgeEdgeChecks, 0u);
  EXPECT_GT(stats.staticRigidSurfaceCcdHits, 0u);
  EXPECT_GT(stats.staticRigidSurfaceCcdLimitedSteps, 0u);
}

//==============================================================================
TEST(DeformableBody, StaticRigidSurfaceCcdUsesStageStartTransform)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  auto box = addStaticSurfaceCcdBox(
      world,
      "moved_box",
      Eigen::Vector3d(10.0, 0.0, 0.0),
      Eigen::Vector3d(0.05, 1.0, 1.0));

  auto body = world.addDeformableBody(
      "fast_point",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(20.0, 0.0, 0.0)));

  Eigen::Isometry3d movedTransform = Eigen::Isometry3d::Identity();
  movedTransform.translation() = Eigen::Vector3d::Zero();
  MoveRigidBodyStage moveBox(box, movedTransform);
  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(moveBox);
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_LT(body.getPosition(0).x(), -0.05);
  EXPECT_GT(stats.staticRigidSurfaceCcdHits, 0u);
  EXPECT_GT(stats.staticRigidSurfaceCcdLimitedSteps, 0u);
}

//==============================================================================
TEST(DeformableBody, StaticRigidSurfaceCcdHandlesRotatedBoxSide)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  addStaticSurfaceCcdBox(
      world,
      "rotated_box",
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.05, 1.0, 1.0),
      Eigen::Quaterniond(
          Eigen::AngleAxisd(
              0.25 * 3.14159265358979323846, Eigen::Vector3d::UnitZ())));

  auto body = world.addDeformableBody(
      "fast_point",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(-1.0, -1.0, 0.0), Eigen::Vector3d(20.0, 20.0, 0.0)));

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_LT(body.getPosition(0).x(), 0.0);
  EXPECT_LT(body.getPosition(0).y(), 0.0);
  EXPECT_GT(body.getPosition(0).x(), -1.0);
  EXPECT_GT(body.getPosition(0).y(), -1.0);
  EXPECT_GT(stats.staticRigidSurfaceCcdHits, 0u);
  EXPECT_GT(stats.staticRigidSurfaceCcdLimitedSteps, 0u);
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

//==============================================================================
TEST(DeformableBody, SerializationLoadsLegacyV8Material)
{
  sx::World world1;
  auto options = makeSingleTetrahedronBody();
  options.material.density = 12.0;
  options.material.youngsModulus = 2500.0;
  options.material.poissonRatio = 0.25;
  options.material.frictionCoefficient = 0.5;
  options.material.useFiniteElementElasticity = true;
  options.material.useFixedCorotationalElasticity = true;
  options.material.useAdaptiveBarrierStiffness = true;
  options.material.useIterativeLinearSolver = true;
  options.material.useMatrixFreeLinearSolver = true;
  world1.addDeformableBody("legacy_v8_material", options);

  std::stringstream currentStream;
  world1.saveBinary(currentStream);
  std::string legacyBytes = currentStream.str();

  const std::uint32_t legacyVersion = 8u;
  ASSERT_GE(legacyBytes.size(), 2u * sizeof(std::uint32_t));
  std::memcpy(
      legacyBytes.data() + sizeof(std::uint32_t),
      &legacyVersion,
      sizeof(legacyVersion));

  const std::string materialTypeName(comps::DeformableMaterial::getTypeName());
  const auto materialTypeOffset = legacyBytes.find(materialTypeName);
  ASSERT_NE(materialTypeOffset, std::string::npos);

  const auto materialDataOffset = materialTypeOffset + materialTypeName.size();
  const auto matrixFreeFlagOffset
      = materialDataOffset + 4u * sizeof(double) + 4u * sizeof(bool);
  ASSERT_LT(matrixFreeFlagOffset, legacyBytes.size());
  legacyBytes.erase(matrixFreeFlagOffset, sizeof(bool));

  std::stringstream legacyStream(legacyBytes);
  sx::World world2;
  ASSERT_NO_THROW(world2.loadBinary(legacyStream));

  auto body = world2.getDeformableBody("legacy_v8_material");
  ASSERT_TRUE(body.has_value());
  const auto material = body->getMaterialProperties();
  EXPECT_DOUBLE_EQ(material.density, 12.0);
  EXPECT_DOUBLE_EQ(material.youngsModulus, 2500.0);
  EXPECT_DOUBLE_EQ(material.poissonRatio, 0.25);
  EXPECT_DOUBLE_EQ(material.frictionCoefficient, 0.5);
  EXPECT_TRUE(material.useFiniteElementElasticity);
  EXPECT_TRUE(material.useFixedCorotationalElasticity);
  EXPECT_TRUE(material.useAdaptiveBarrierStiffness);
  EXPECT_TRUE(material.useIterativeLinearSolver);
  EXPECT_FALSE(material.useMatrixFreeLinearSolver);
}

//==============================================================================
TEST(DeformableBody, SerializationLoadsLegacyV9MaterialWithoutMatrixFreeFlag)
{
  sx::World world1;
  auto options = makeSingleTetrahedronBody();
  options.material.density = 12.0;
  options.material.youngsModulus = 2500.0;
  options.material.poissonRatio = 0.25;
  options.material.frictionCoefficient = 0.5;
  options.material.useFiniteElementElasticity = true;
  options.material.useFixedCorotationalElasticity = true;
  options.material.useAdaptiveBarrierStiffness = true;
  options.material.useIterativeLinearSolver = true;
  options.material.useMatrixFreeLinearSolver = true;
  world1.addDeformableBody("legacy_v9_material", options);

  std::stringstream currentStream;
  world1.saveBinary(currentStream);
  std::string legacyBytes = currentStream.str();

  const std::uint32_t legacyVersion = 9u;
  ASSERT_GE(legacyBytes.size(), 2u * sizeof(std::uint32_t));
  std::memcpy(
      legacyBytes.data() + sizeof(std::uint32_t),
      &legacyVersion,
      sizeof(legacyVersion));

  const std::string materialTypeName(comps::DeformableMaterial::getTypeName());
  const auto materialTypeOffset = legacyBytes.find(materialTypeName);
  ASSERT_NE(materialTypeOffset, std::string::npos);

  const auto materialDataOffset = materialTypeOffset + materialTypeName.size();
  const auto matrixFreeFlagOffset
      = materialDataOffset + 4u * sizeof(double) + 4u * sizeof(bool);
  ASSERT_LT(matrixFreeFlagOffset, legacyBytes.size());
  legacyBytes.erase(matrixFreeFlagOffset, sizeof(bool));

  std::stringstream legacyStream(legacyBytes);
  sx::World world2;
  ASSERT_NO_THROW(world2.loadBinary(legacyStream));

  auto body = world2.getDeformableBody("legacy_v9_material");
  ASSERT_TRUE(body.has_value());
  const auto material = body->getMaterialProperties();
  EXPECT_DOUBLE_EQ(material.density, 12.0);
  EXPECT_DOUBLE_EQ(material.youngsModulus, 2500.0);
  EXPECT_DOUBLE_EQ(material.poissonRatio, 0.25);
  EXPECT_DOUBLE_EQ(material.frictionCoefficient, 0.5);
  EXPECT_TRUE(material.useFiniteElementElasticity);
  EXPECT_TRUE(material.useFixedCorotationalElasticity);
  EXPECT_TRUE(material.useAdaptiveBarrierStiffness);
  EXPECT_TRUE(material.useIterativeLinearSolver);
  EXPECT_FALSE(material.useMatrixFreeLinearSolver);
}

//==============================================================================
// A deformable node advancing toward a box that is itself moving toward the
// node is conservatively limited so it does not enter the box's predicted
// end-of-step pose. The custom pipeline runs only the deformable stage; the
// limiter still predicts the box end pose from its velocity.
TEST(DeformableBody, MovingRigidSurfaceCcdLimitsAgainstPredictedPose)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  const Eigen::Vector3d halfExtents(0.05, 1.0, 1.0);
  addMovingSurfaceCcdBox(
      world,
      "moving_box",
      Eigen::Vector3d(2.0, 0.0, 0.0),
      halfExtents,
      Eigen::Vector3d(-20.0, 0.0, 0.0));

  auto body = world.addDeformableBody(
      "approaching_point",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(20.0, 0.0, 0.0)));

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_EQ(stats.movingRigidSurfaceCcdBoxCount, 1u);
  // The swept motion is tiled by overlapping static pose samples (>= 2), each
  // contributing the box's 12 triangles / 12 edges.
  EXPECT_GE(stats.movingRigidSurfaceCcdSampleCount, 2u);
  EXPECT_EQ(
      stats.movingRigidSurfaceCcdTriangleCount,
      12u * stats.movingRigidSurfaceCcdSampleCount);
  EXPECT_EQ(
      stats.movingRigidSurfaceCcdEdgeCount,
      12u * stats.movingRigidSurfaceCcdSampleCount);
  EXPECT_GT(stats.movingRigidSurfaceCcdCandidateBuilds, 0u);
  EXPECT_GT(stats.movingRigidSurfaceCcdPointTriangleCandidates, 0u);
  EXPECT_GT(stats.movingRigidSurfaceCcdPointTriangleChecks, 0u);
  EXPECT_GT(stats.movingRigidSurfaceCcdHits, 0u);
  EXPECT_GT(stats.movingRigidSurfaceCcdLimitedSteps, 0u);
  // The moving obstacle path must not be confused with the static one.
  EXPECT_EQ(stats.staticRigidSurfaceCcdBoxCount, 0u);

  // The node moved toward its free target but was limited short of it.
  const Eigen::Vector3d finalPosition = body.getPosition(0);
  EXPECT_GT(finalPosition.x(), -1.0);
  EXPECT_LT(finalPosition.x(), 1.0);

  // Conservative guarantee: the node stays outside the box's predicted end pose
  // (center moves by velocity * dt = -20 * 0.1 = -2.0 to the origin).
  const Eigen::Vector3d predictedEndCenter(0.0, 0.0, 0.0);
  EXPECT_TRUE(isOutsideBox(
      finalPosition,
      predictedEndCenter,
      Eigen::Quaterniond::Identity(),
      halfExtents));
}

//==============================================================================
// Isolates the ordering fix: with the box STATIC at its start pose, the node's
// own trajectory never reaches it, so the static limiter does not engage and
// the node reaches its free target. With the SAME geometry but the box moving
// onto the node's path, the moving limiter predicts the end pose and limits the
// node. Same node motion, opposite outcome -> the prediction is what matters.
TEST(DeformableBody, MovingRigidSurfaceCcdConstrainsWhereStaticSnapshotWouldNot)
{
  const Eigen::Vector3d boxStart(2.0, 0.0, 0.0);
  const Eigen::Vector3d halfExtents(0.05, 1.0, 1.0);
  const Eigen::Vector3d nodeStart(-1.0, 0.0, 0.0);
  const Eigen::Vector3d nodeVelocity(20.0, 0.0, 0.0);

  // Baseline: a static box at the start pose is too far from the node's
  // trajectory to limit it.
  double staticFinalX = 0.0;
  {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.1);
    addStaticSurfaceCcdBox(world, "static_box", boxStart, halfExtents);
    auto body = world.addDeformableBody(
        "approaching_point",
        makeSingleNodeBodyOptions(nodeStart, nodeVelocity));

    compute::SequentialExecutor executor;
    compute::DeformableDynamicsStage stage;
    compute::WorldStepPipeline pipeline;
    pipeline.addStage(stage);
    world.step(executor, pipeline);

    const auto& stats = stage.getLastStats();
    EXPECT_EQ(stats.movingRigidSurfaceCcdBoxCount, 0u);
    EXPECT_EQ(stats.staticRigidSurfaceCcdLimitedSteps, 0u);
    staticFinalX = body.getPosition(0).x();
    EXPECT_GT(staticFinalX, 0.9); // reached free target ~ +1.0
  }

  // Moving: the same box sweeps onto the node's path and is predicted, so the
  // node is limited well short of its free target.
  double movingFinalX = 0.0;
  {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.1);
    addMovingSurfaceCcdBox(
        world,
        "moving_box",
        boxStart,
        halfExtents,
        Eigen::Vector3d(-20.0, 0.0, 0.0));
    auto body = world.addDeformableBody(
        "approaching_point",
        makeSingleNodeBodyOptions(nodeStart, nodeVelocity));

    compute::SequentialExecutor executor;
    compute::DeformableDynamicsStage stage;
    compute::WorldStepPipeline pipeline;
    pipeline.addStage(stage);
    world.step(executor, pipeline);

    const auto& stats = stage.getLastStats();
    EXPECT_EQ(stats.staticRigidSurfaceCcdBoxCount, 0u);
    EXPECT_GT(stats.movingRigidSurfaceCcdLimitedSteps, 0u);
    movingFinalX = body.getPosition(0).x();
  }

  // The moving prediction constrains the node where the static snapshot did
  // not.
  EXPECT_LT(movingFinalX, staticFinalX);
  EXPECT_LT(movingFinalX, 0.9);
}

//==============================================================================
// An obstacle receding from the deformable must not spuriously limit it.
TEST(DeformableBody, MovingRigidSurfaceCcdDoesNotLimitRecedingObstacle)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  addMovingSurfaceCcdBox(
      world,
      "receding_box",
      Eigen::Vector3d(2.0, 0.0, 0.0),
      Eigen::Vector3d(0.05, 1.0, 1.0),
      Eigen::Vector3d(20.0, 0.0, 0.0)); // moves away in +x

  auto body = world.addDeformableBody(
      "approaching_point",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(20.0, 0.0, 0.0)));

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_EQ(stats.movingRigidSurfaceCcdBoxCount, 1u);
  EXPECT_EQ(stats.movingRigidSurfaceCcdHits, 0u);
  EXPECT_EQ(stats.movingRigidSurfaceCcdLimitedSteps, 0u);
  // The node reaches its free target unimpeded.
  EXPECT_GT(body.getPosition(0).x(), 0.9);
}

//==============================================================================
// A fast-rotating obstacle is handled conservatively: its swept motion is tiled
// by sampled rotated poses, so a node advancing toward the spinning slab is
// limited and stays outside the slab's rotated end pose, deterministically.
TEST(DeformableBody, MovingRigidSurfaceCcdHandlesRotatingObstacle)
{
  const Eigen::Vector3d boxCenter(0.0, 0.0, 0.0);
  const Eigen::Vector3d halfExtents(1.0, 0.1, 1.0);
  const Eigen::Vector3d angularVelocity(0.0, 0.0, 10.0); // 10 rad/s about z
  const double timeStep = 0.1;                           // theta = 1.0 rad

  std::size_t limitedSteps = 0;
  std::size_t boxCount = 0;
  const auto run = [&]() {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(timeStep);
    addMovingSurfaceCcdBox(
        world,
        "spinning_slab",
        boxCenter,
        halfExtents,
        Eigen::Vector3d::Zero(),
        angularVelocity);
    // Node starts outside the slab's swept disc on the +x side and drives in
    // toward the center, so it must be limited by the rotating obstacle.
    auto body = world.addDeformableBody(
        "approaching_point",
        makeSingleNodeBodyOptions(
            Eigen::Vector3d(1.5, 0.0, 0.0), Eigen::Vector3d(-10.0, 0.0, 0.0)));

    compute::SequentialExecutor executor;
    compute::DeformableDynamicsStage stage;
    compute::WorldStepPipeline pipeline;
    pipeline.addStage(stage);
    world.step(executor, pipeline);
    limitedSteps = stage.getLastStats().movingRigidSurfaceCcdLimitedSteps;
    boxCount = stage.getLastStats().movingRigidSurfaceCcdBoxCount;
    return body.getPosition(0);
  };

  const Eigen::Vector3d finalPosition = run();

  EXPECT_EQ(boxCount, 1u);
  EXPECT_GT(limitedSteps, 0u);

  // Predicted end orientation: rotation by theta = |angular| * dt = 1.0 rad
  // about z. The node must remain outside the obstacle's rotated end pose.
  const Eigen::Quaterniond endOrientation(
      Eigen::AngleAxisd(10.0 * timeStep, Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(
      isOutsideBox(finalPosition, boxCenter, endOrientation, halfExtents));

  // Determinism: identical inputs produce identical output.
  const Eigen::Vector3d repeat = run();
  EXPECT_DOUBLE_EQ(finalPosition.x(), repeat.x());
  EXPECT_DOUBLE_EQ(finalPosition.y(), repeat.y());
  EXPECT_DOUBLE_EQ(finalPosition.z(), repeat.z());
}

//==============================================================================
// The moving and static rigid obstacle collectors are disjoint: a static CCD
// box increments only static counters and a moving CCD box increments only
// moving counters.
TEST(DeformableBody, MovingAndStaticRigidSurfaceCcdCollectorsAreDisjoint)
{
  const Eigen::Vector3d halfExtents(0.05, 1.0, 1.0);

  {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.1);
    addStaticSurfaceCcdBox(
        world, "static_box", Eigen::Vector3d::Zero(), halfExtents);
    auto body = world.addDeformableBody(
        "fast_point",
        makeSingleNodeBodyOptions(
            Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(20.0, 0.0, 0.0)));
    compute::SequentialExecutor executor;
    compute::DeformableDynamicsStage stage;
    compute::WorldStepPipeline pipeline;
    pipeline.addStage(stage);
    world.step(executor, pipeline);
    EXPECT_TRUE(body.isValid());
    const auto& stats = stage.getLastStats();
    EXPECT_EQ(stats.staticRigidSurfaceCcdBoxCount, 1u);
    EXPECT_EQ(stats.movingRigidSurfaceCcdBoxCount, 0u);
    EXPECT_EQ(stats.movingRigidSurfaceCcdCandidateBuilds, 0u);
    EXPECT_EQ(stats.movingRigidSurfaceCcdHits, 0u);
  }

  {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.1);
    addMovingSurfaceCcdBox(
        world,
        "moving_box",
        Eigen::Vector3d(2.0, 0.0, 0.0),
        halfExtents,
        Eigen::Vector3d(-20.0, 0.0, 0.0));
    auto body = world.addDeformableBody(
        "fast_point",
        makeSingleNodeBodyOptions(
            Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(20.0, 0.0, 0.0)));
    compute::SequentialExecutor executor;
    compute::DeformableDynamicsStage stage;
    compute::WorldStepPipeline pipeline;
    pipeline.addStage(stage);
    world.step(executor, pipeline);
    EXPECT_TRUE(body.isValid());
    const auto& stats = stage.getLastStats();
    EXPECT_EQ(stats.movingRigidSurfaceCcdBoxCount, 1u);
    EXPECT_EQ(stats.staticRigidSurfaceCcdBoxCount, 0u);
    EXPECT_EQ(stats.staticRigidSurfaceCcdCandidateBuilds, 0u);
    EXPECT_EQ(stats.staticRigidSurfaceCcdHits, 0u);
  }
}

//==============================================================================
// Kinematic rigid IPC bodies are already advanced before deformableDynamics in
// the IPC pipeline. The deformable surface-CCD collector should therefore use
// the realized current step trace, not predict an extra velocity step into the
// next frame.
TEST(DeformableBody, KinematicSurfaceCcdObstacleUsesRealizedPoseInIpcPipeline)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  const Eigen::Vector3d halfExtents(0.05, 1.0, 1.0);
  const Eigen::Vector3d velocity(1.0, 0.0, 0.0);
  auto box = addMovingSurfaceCcdBox(
      world, "kinematic_box", Eigen::Vector3d::Zero(), halfExtents, velocity);
  box.setKinematic(true);

  auto body = world.addDeformableBody(
      "far_point",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(10.0, 0.0, 0.0), Eigen::Vector3d::Zero()));

  compute::SequentialExecutor executor;
  compute::RigidIpcContactStage ipcStage;
  compute::DeformableDynamicsStage deformableStage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage).addStage(deformableStage);
  world.step(executor, pipeline);

  EXPECT_TRUE(body.isValid());
  expectVectorNear(box.getTranslation(), velocity * world.getTimeStep());
  const auto& stats = deformableStage.getLastStats();
  EXPECT_EQ(stats.staticRigidSurfaceCcdBoxCount, 0u);
  EXPECT_EQ(stats.movingRigidSurfaceCcdBoxCount, 1u);
  EXPECT_GE(stats.movingRigidSurfaceCcdSampleCount, 2u);
  EXPECT_EQ(
      stats.movingRigidSurfaceCcdTriangleCount,
      12u * stats.movingRigidSurfaceCcdSampleCount);
  EXPECT_EQ(stats.movingRigidSurfaceCcdHits, 0u);
}

//==============================================================================
// Kinematic spheres are advanced by the rigid IPC stage, but moving rigid
// surface-CCD snapshots currently support boxes only. Keep the realized
// current-pose sphere snapshot active so deformables still get a conservative
// limiter until swept-sphere snapshots are implemented.
TEST(DeformableBody, KinematicSurfaceCcdSphereKeepsCurrentPoseSnapshot)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  const Eigen::Vector3d velocity(1.0, 0.0, 0.0);
  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d::Zero();
  auto sphere = world.addRigidBody("kinematic_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  sphere.setDeformableSurfaceCcdObstacle(true);
  sphere.setLinearVelocity(velocity);
  sphere.setKinematic(true);

  auto body = world.addDeformableBody(
      "fast_point",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(20.0, 0.0, 0.0)));

  compute::SequentialExecutor executor;
  compute::RigidIpcContactStage ipcStage;
  compute::DeformableDynamicsStage deformableStage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage).addStage(deformableStage);
  world.step(executor, pipeline);

  expectVectorNear(sphere.getTranslation(), velocity * world.getTimeStep());
  const auto& stats = deformableStage.getLastStats();
  EXPECT_EQ(stats.staticRigidSurfaceCcdSphereCount, 1u);
  EXPECT_EQ(stats.staticRigidSurfaceCcdBoxCount, 0u);
  EXPECT_EQ(stats.movingRigidSurfaceCcdBoxCount, 0u);
  EXPECT_GT(stats.staticRigidSurfaceCcdHits, 0u);
  EXPECT_GT(stats.staticRigidSurfaceCcdLimitedSteps, 0u);

  const Eigen::Vector3d finalPosition = body.getPosition(0);
  EXPECT_GT(finalPosition.x(), -1.0);
  EXPECT_LT(finalPosition.x(), -0.4);
}

//==============================================================================
// The IPC trace must cover the realized kinematic start->end sweep. A
// final-pose snapshot at x = 2 would not block this deformable point, but the
// realized corridor from x = -2 to x = 2 crosses its path and must limit it.
TEST(DeformableBody, KinematicSurfaceCcdObstacleSweepsRealizedIpcMotion)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  const Eigen::Vector3d halfExtents(0.05, 1.0, 1.0);
  const Eigen::Vector3d startPosition(-2.0, 0.0, 0.0);
  const Eigen::Vector3d velocity(40.0, 0.0, 0.0);
  auto box = addMovingSurfaceCcdBox(
      world, "kinematic_box", startPosition, halfExtents, velocity);
  box.setKinematic(true);

  auto body = world.addDeformableBody(
      "swept_point",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0)));

  compute::SequentialExecutor executor;
  compute::RigidIpcContactStage ipcStage;
  compute::DeformableDynamicsStage deformableStage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage).addStage(deformableStage);
  world.step(executor, pipeline);

  EXPECT_TRUE(body.isValid());
  expectVectorNear(
      box.getTranslation(), startPosition + velocity * world.getTimeStep());
  const auto& stats = deformableStage.getLastStats();
  EXPECT_EQ(stats.staticRigidSurfaceCcdBoxCount, 0u);
  EXPECT_EQ(stats.movingRigidSurfaceCcdBoxCount, 1u);
  EXPECT_GT(stats.movingRigidSurfaceCcdHits, 0u);
  EXPECT_GT(stats.movingRigidSurfaceCcdLimitedSteps, 0u);

  const Eigen::Vector3d finalPosition = body.getPosition(0);
  EXPECT_GT(finalPosition.x(), 0.0);
  EXPECT_LT(finalPosition.x(), 0.1);
}

//==============================================================================
// A very fast obstacle exceeds the sample cap; the sampled boxes are then
// inflated to keep overlapping, so the deformable still cannot tunnel into the
// swept corridor (the conservative invariant holds past the cap).
TEST(
    DeformableBody, MovingRigidSurfaceCcdInflatesSamplesPastCapWithoutTunneling)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  // minHalfExtent = 0.05; motion = 200 * 0.1 = 20 >> 64 * 0.05, so the sample
  // count is capped and the boxes must be inflated to bridge the residual gaps.
  const Eigen::Vector3d halfExtents(0.05, 1.0, 1.0);
  addMovingSurfaceCcdBox(
      world,
      "very_fast_box",
      Eigen::Vector3d(2.0, 0.0, 0.0),
      halfExtents,
      Eigen::Vector3d(-200.0, 0.0, 0.0));

  // Node approaches the corridor from the +x side (outside the swept region).
  auto body = world.addDeformableBody(
      "approaching_point",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(3.0, 0.0, 0.0), Eigen::Vector3d(-10.0, 0.0, 0.0)));

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_EQ(stats.movingRigidSurfaceCcdBoxCount, 1u);
  EXPECT_GT(stats.movingRigidSurfaceCcdInflatedBoxCount, 0u);
  EXPECT_GT(stats.movingRigidSurfaceCcdLimitedSteps, 0u);

  // The node is stopped before entering the swept corridor (start box right
  // face is at x = 2.05); it must not tunnel into or across the corridor.
  const Eigen::Vector3d finalPosition = body.getPosition(0);
  EXPECT_GT(finalPosition.x(), 2.0);
  EXPECT_LT(finalPosition.x(), 3.0);
}

//==============================================================================
// End-to-end through the FULL default pipeline: the obstacle is actually
// integrated by RigidBodyPositionStage (stage 5) after the deformable solve
// (stage 4). With gravity off and no rigid contacts the obstacle velocity is
// constant across stages, so the velocity-based prediction the deformable stage
// limits against equals the pose the obstacle really reaches, and the node ends
// outside the obstacle's realized end pose.
TEST(DeformableBody, MovingRigidSurfaceCcdMatchesRealizedMotionInFullPipeline)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  const Eigen::Vector3d boxStart(1.0, 0.0, 0.0);
  const Eigen::Vector3d halfExtents(0.2, 1.0, 1.0);
  const Eigen::Vector3d boxVelocity(-1.0, 0.0, 0.0);
  auto box = addMovingSurfaceCcdBox(
      world, "moving_box", boxStart, halfExtents, boxVelocity);

  // Free target (+1.0) lies inside the obstacle's end pose, so the node must be
  // limited by the full-pipeline solve.
  auto body = world.addDeformableBody(
      "approaching_point",
      makeSingleNodeBodyOptions(
          Eigen::Vector3d(-0.5, 0.0, 0.0), Eigen::Vector3d(15.0, 0.0, 0.0)));

  world.step(); // default pipeline: integrates the obstacle at stage 5

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_GT(diagnostics.movingRigidSurfaceCcdSnapshotBuilds, 0u);
  EXPECT_EQ(diagnostics.movingRigidSurfaceCcdBoxCount, 1u);
  EXPECT_GE(diagnostics.movingRigidSurfaceCcdSampleCount, 2u);
  EXPECT_EQ(
      diagnostics.movingRigidSurfaceCcdTriangleCount,
      12u * diagnostics.movingRigidSurfaceCcdSampleCount);
  EXPECT_EQ(
      diagnostics.movingRigidSurfaceCcdEdgeCount,
      12u * diagnostics.movingRigidSurfaceCcdSampleCount);
  EXPECT_GT(diagnostics.movingRigidSurfaceCcdCandidateBuilds, 0u);
  EXPECT_GT(diagnostics.movingRigidSurfaceCcdPointTriangleCandidates, 0u);
  EXPECT_GT(diagnostics.movingRigidSurfaceCcdPointTriangleChecks, 0u);
  EXPECT_GT(diagnostics.movingRigidSurfaceCcdHits, 0u);
  EXPECT_GT(diagnostics.movingRigidSurfaceCcdLimitedSteps, 0u);
  EXPECT_EQ(diagnostics.staticRigidSurfaceCcdBoxCount, 0u);

  // Obstacle velocity is unchanged (no gravity, no rigid contacts), so the
  // predicted end pose equals the realized one: center = 1.0 + (-1.0) * 0.1.
  expectVectorNear(box.getLinearVelocity(), boxVelocity, 1e-12);
  const Eigen::Vector3d realizedEndCenter
      = boxStart + boxVelocity * world.getTimeStep();

  const Eigen::Vector3d finalPosition = body.getPosition(0);
  EXPECT_GT(finalPosition.x(), -0.5); // advanced toward its target
  EXPECT_TRUE(isOutsideBox(
      finalPosition,
      realizedEndCenter,
      Eigen::Quaterniond::Identity(),
      halfExtents));
}

//==============================================================================
// Two facing triangles in ONE deformable body: the lower triangle is fixed and
// the upper triangle (directly above) is driven down into it. The self-contact
// barrier must produce a repulsive force that keeps the surfaces apart, beyond
// what the CCD min-separation alone would give.
namespace {
sx::DeformableBodyOptions makeTwoFacingTrianglesOptions(
    double gap, double downwardVelocity)
{
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(0.0, 0.0, 0.0),
         Eigen::Vector3d(1.0, 0.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0),
         Eigen::Vector3d(0.0, 0.0, gap),
         Eigen::Vector3d(1.0, 0.0, gap),
         Eigen::Vector3d(0.0, 1.0, gap)};
  options.velocities
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d(0.0, 0.0, -downwardVelocity),
         Eigen::Vector3d(0.0, 0.0, -downwardVelocity),
         Eigen::Vector3d(0.0, 0.0, -downwardVelocity)};
  options.masses = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  options.fixedNodes = {0, 1, 2}; // lower triangle is a fixed obstacle
  options.edgeStiffness = 0.0;    // no springs: isolate the contact response
  options.damping = 0.0;
  options.surfaceTriangles
      = {sx::DeformableSurfaceTriangle{0, 1, 2},
         sx::DeformableSurfaceTriangle{3, 4, 5}};
  return options;
}

double minUpperTriangleHeight(const sx::DeformableBody& body)
{
  return std::min(
      {body.getPosition(3).z(),
       body.getPosition(4).z(),
       body.getPosition(5).z()});
}
} // namespace

//==============================================================================
TEST(DeformableBody, SelfContactBarrierKeepsDrivenSurfacesApart)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  // Start within the barrier activation band; drive the upper triangle hard
  // enough that its free (barrier-free) target would penetrate the lower one.
  auto body = world.addDeformableBody(
      "facing_triangles", makeTwoFacingTrianglesOptions(0.015, 0.2));

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_GT(stats.selfContactBarrierCandidateBuilds, 0u);
  EXPECT_GT(stats.selfContactBarrierActiveContacts, 0u);

  // Projected Newton is the dominant search direction here: the 12x12 barrier
  // Hessian assembly and per-element PSD projection run on every iteration
  // under a heavily-loaded active set (~255 contacts). At most an occasional
  // near-convergence iteration, where the Newton step's directional derivative
  // rounds toward zero, degrades gracefully to the steepest-descent fallback,
  // so Newton steps strictly outnumber fallbacks.
  EXPECT_GT(stats.projectedNewtonSteps, 0u);
  EXPECT_GT(stats.projectedNewtonSteps, stats.projectedNewtonFallbacks);

  // The barrier produces a strong repulsive force that holds the surfaces at a
  // stable separation inside the activation band (d_hat = 2e-2), far beyond the
  // CCD min-separation (1e-4) that CCD-only limiting would leave.
  const double height = minUpperTriangleHeight(body);
  EXPECT_GT(height, 5e-3); // clear repulsion, orders of magnitude above min-sep
  EXPECT_LT(height, 2e-2); // settles within the barrier activation band
}

//==============================================================================
TEST(DeformableBody, SelfContactBarrierInactiveWhenFarApart)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  // Gap far exceeds d_hat (2e-2); a tiny downward velocity keeps it that way.
  auto body = world.addDeformableBody(
      "facing_triangles_far", makeTwoFacingTrianglesOptions(0.5, 0.1));

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_EQ(stats.selfContactBarrierActiveContacts, 0u);
  // The upper triangle reaches its free target (0.5 - 0.1 * 0.1 = 0.49).
  EXPECT_NEAR(minUpperTriangleHeight(body), 0.49, 1e-6);
}

//==============================================================================
// The converged contact diagnostic reports a positive closest approach inside
// the activation band whenever self-contact holds two surfaces apart -- the IPC
// intersection-free "minimum distance" statistic.
TEST(DeformableBody, SelfContactBarrierReportsConvergedContactDistance)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  auto body = world.addDeformableBody(
      "facing_triangles", makeTwoFacingTrianglesOptions(0.015, 0.2));

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  // The terminal active set is nonempty and the closest approach is a positive
  // distance strictly inside the barrier activation band (d_hat = 2e-2): the
  // surfaces are held apart, not penetrating and not yet outside the band.
  EXPECT_GT(stats.convergedActiveContactCount, 0u);
  EXPECT_GT(stats.minActiveContactDistance, 0.0);
  EXPECT_LT(stats.minActiveContactDistance, 2e-2);
  // It never exceeds the cumulative active count (a single-iteration snapshot
  // versus the sum over every outer iteration).
  EXPECT_LE(
      stats.convergedActiveContactCount,
      stats.selfContactBarrierActiveContacts);
  // The reported closest approach reflects a genuine positive separation: the
  // surfaces are held apart, not pinned together.
  EXPECT_GT(minUpperTriangleHeight(body), 0.0);
}

//==============================================================================
// With the surfaces far outside the activation band, the converged contact
// diagnostic reports an empty active set and a zero closest approach (the
// sentinel for "no active self-contact").
TEST(
    DeformableBody, SelfContactBarrierReportsNoConvergedContactDistanceFarApart)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  auto body = world.addDeformableBody(
      "facing_triangles_far", makeTwoFacingTrianglesOptions(0.5, 0.1));

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_EQ(stats.convergedActiveContactCount, 0u);
  EXPECT_EQ(stats.minActiveContactDistance, 0.0);
  // The surfaces remain well outside the activation band (d_hat = 2e-2).
  EXPECT_GT(minUpperTriangleHeight(body), 2e-2);
}

//==============================================================================
TEST(DeformableBody, SelfContactBarrierIsDeterministic)
{
  const auto run = [] {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.1);
    auto body = world.addDeformableBody(
        "facing_triangles", makeTwoFacingTrianglesOptions(0.015, 0.2));
    compute::SequentialExecutor executor;
    compute::DeformableDynamicsStage stage;
    compute::WorldStepPipeline pipeline;
    pipeline.addStage(stage);
    world.step(executor, pipeline);
    return minUpperTriangleHeight(body);
  };
  EXPECT_DOUBLE_EQ(run(), run());
}

//==============================================================================
// The self-contact barrier must use the same surface point mask as the CCD
// path: a volumetric body's interior node (not a vertex of any surface
// triangle) must never receive a barrier point-triangle force against its own
// shell, even when it lies within the activation band.
TEST(DeformableBody, SelfContactBarrierIgnoresVolumetricInteriorNodes)
{
  auto options = makeVolumetricInteriorNodeCrossingOptions();
  // Place the interior apex (node 4) within d_hat (2e-2) of the bottom surface
  // triangle and let it drift slowly; without the mask it would generate a
  // spurious barrier contact against the shell.
  options.positions[4] = Eigen::Vector3d(0.0, 0.0, 0.015);
  options.velocities[4] = Eigen::Vector3d(0.0, 0.0, -0.05);

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);
  auto body = world.addDeformableBody("volumetric_interior_barrier", options);

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_TRUE(body.isValid());
  EXPECT_GT(stats.selfContactBarrierCandidateBuilds, 0u); // barrier path ran
  // The masked interior node produces no surface self-contact barrier force.
  EXPECT_EQ(stats.selfContactBarrierActiveContacts, 0u);
}

//==============================================================================
// The deformable solve uses a projected-Newton search direction: it engages
// (no fallback) and converges the linear spring step in very few iterations,
// reaching the analytic implicit-Euler equilibrium.
TEST(DeformableBody, ProjectedNewtonSolvesSpringStepInFewIterations)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  auto options = makeTwoNodeBody();
  options.positions[1] = Eigen::Vector3d(2.0, 0.0, 0.0);
  options.edges = {sx::DeformableEdge{0, 1, 1.0}};
  options.edgeStiffness = 100.0;
  options.masses = {1.0, 1.0};
  auto body = world.addDeformableBody("newton_spring", options);

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  EXPECT_GT(stats.projectedNewtonSteps, 0u);
  EXPECT_EQ(stats.projectedNewtonFallbacks, 0u);
  // This (effectively linear) spring step is solved by a single Newton step,
  // with one confirming iteration that breaks on the gradient tolerance: two
  // outer iterations total, far fewer than steepest descent. A tight bound
  // makes this a real regression guard against a convergence slowdown.
  EXPECT_LE(stats.solverIterations, 2u);
  // The reported convergence residual reflects a converged solve (the loop
  // terminates on the gradient tolerance).
  EXPECT_LT(stats.finalGradientResidualNorm, 1e-6);

  constexpr double inertialWeight = 1.0 / (0.1 * 0.1);
  constexpr double expectedX
      = (inertialWeight * 2.0 + 100.0 * 1.0) / (inertialWeight + 100.0);
  expectVectorNear(body.getPosition(1), Eigen::Vector3d(expectedX, 0.0, 0.0));
}

//==============================================================================
// Sparse projected Newton scales past the former dense 256-node cap: a 300-node
// spring chain (well beyond the old limit) is solved on the Newton path. The
// previous dense solver returned false for any body over 256 nodes, so this
// whole body would have fallen back to steepest descent for every iteration;
// the sparse Cholesky assembly now keeps it on the Newton path.
TEST(DeformableBody, SparseProjectedNewtonScalesBeyondDenseCap)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);

  constexpr std::size_t kNodeCount = 300; // > the former 256-node dense cap
  constexpr double spacing = 0.1;

  sx::DeformableBodyOptions options;
  options.positions.reserve(kNodeCount);
  for (std::size_t i = 0; i < kNodeCount; ++i) {
    options.positions.emplace_back(static_cast<double>(i) * spacing, 0.0, 0.0);
  }
  options.masses.assign(kNodeCount, 1.0);
  options.edges.reserve(kNodeCount - 1);
  for (std::size_t i = 0; i + 1 < kNodeCount; ++i) {
    options.edges.push_back(sx::DeformableEdge{i, i + 1, spacing});
  }
  options.edgeStiffness = 100.0;
  options.fixedNodes = {0}; // anchor one end of the chain

  auto body = world.addDeformableBody("sparse_chain", options);

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  world.step(executor, pipeline);

  const auto& stats = stage.getLastStats();
  // The >256-node body is solved on the sparse Newton path; the old dense
  // solver would have reported projectedNewtonSteps == 0 (all fallback).
  EXPECT_GT(stats.projectedNewtonSteps, 0u);
  EXPECT_GT(stats.projectedNewtonSteps, stats.projectedNewtonFallbacks);

  // The solve stays finite, keeps the anchor pinned, and lets gravity sag the
  // free end downward.
  expectVectorNear(body.getPosition(0), Eigen::Vector3d::Zero());
  for (std::size_t i = 0; i < kNodeCount; ++i) {
    EXPECT_TRUE(body.getPosition(i).allFinite());
  }
  EXPECT_LT(body.getPosition(kNodeCount - 1).z(), 0.0);
}

//==============================================================================
// Sparse projected Newton scales past the dense cap WITH active contact: a
// 320-node grid (> the former 256-node cap) pressed onto a static ground
// barrier is solved on the Newton path, so the sparse assembly's ground-barrier
// Hessian scatter runs above the old cap. The barrier holds it non-penetrating.
TEST(DeformableBody, SparseProjectedNewtonScalesWithGroundBarrier)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);

  // Ground barrier: static box whose top surface sits at z = 0.
  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(100.0, 100.0, 0.5)));
  ground.setDeformableGroundBarrier(true);

  // 20x16 = 320-node grid lying just inside the barrier activation band
  // (d_hat = 2e-2), connected by structural springs along both axes.
  constexpr std::size_t kColumns = 20;
  constexpr std::size_t kRows = 16;
  constexpr std::size_t kNodeCount = kColumns * kRows; // 320 > 256
  constexpr double spacing = 0.1;
  constexpr double restZ = 0.01; // inside (0, d_hat)

  const auto index = [&](std::size_t c, std::size_t r) {
    return r * kColumns + c;
  };

  sx::DeformableBodyOptions options;
  options.positions.reserve(kNodeCount);
  for (std::size_t r = 0; r < kRows; ++r) {
    for (std::size_t c = 0; c < kColumns; ++c) {
      options.positions.emplace_back(
          static_cast<double>(c) * spacing,
          static_cast<double>(r) * spacing,
          restZ);
    }
  }
  options.masses.assign(kNodeCount, 1.0);
  for (std::size_t r = 0; r < kRows; ++r) {
    for (std::size_t c = 0; c < kColumns; ++c) {
      if (c + 1 < kColumns) {
        options.edges.push_back(
            sx::DeformableEdge{index(c, r), index(c + 1, r), spacing});
      }
      if (r + 1 < kRows) {
        options.edges.push_back(
            sx::DeformableEdge{index(c, r), index(c, r + 1), spacing});
      }
    }
  }
  options.edgeStiffness = 100.0;

  auto body = world.addDeformableBody("sparse_grid", options);

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  for (int step = 0; step < 5; ++step) {
    world.step(executor, pipeline);
  }

  const auto& stats = stage.getLastStats();
  // The >256-node body is solved on the sparse Newton path with the ground
  // barrier configured, so the sparse assembly's ground-barrier Hessian scatter
  // runs above the former dense cap.
  EXPECT_GT(stats.projectedNewtonSteps, 0u);
  EXPECT_EQ(stats.staticGroundBarrierCount, 1u);

  // Gravity presses the grid onto the barrier, which holds every node above the
  // ground surface (z = 0) without penetration.
  for (std::size_t i = 0; i < kNodeCount; ++i) {
    EXPECT_TRUE(body.getPosition(i).allFinite());
    EXPECT_GE(body.getPosition(i).z(), -1e-3);
  }
}

//==============================================================================
// The finalStepInfinityNorm diagnostic reports the last accepted per-node step,
// a converged-ness measure that complements the gradient residual. For stiff
// clamped-log barrier contact the barrier Hessian is near-singular, so the raw
// gradient norm can stay large even at equilibrium; the step norm instead
// shrinks toward zero as the solve settles. A grid pressed onto the ground
// barrier accepts a measurable step while actively settling, then drives the
// step norm down to a negligible value at the feasible equilibrium.
TEST(DeformableBody, StiffGroundBarrierSettlesByStepNorm)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(100.0, 100.0, 0.5)));
  ground.setDeformableGroundBarrier(true);

  constexpr std::size_t kColumns = 20;
  constexpr std::size_t kRows = 16;
  constexpr std::size_t kNodeCount = kColumns * kRows;
  constexpr double spacing = 0.1;
  constexpr double restZ = 0.01;
  const auto index = [&](std::size_t c, std::size_t r) {
    return r * kColumns + c;
  };

  sx::DeformableBodyOptions options;
  options.positions.reserve(kNodeCount);
  for (std::size_t r = 0; r < kRows; ++r) {
    for (std::size_t c = 0; c < kColumns; ++c) {
      options.positions.emplace_back(
          static_cast<double>(c) * spacing,
          static_cast<double>(r) * spacing,
          restZ);
    }
  }
  options.masses.assign(kNodeCount, 1.0);
  for (std::size_t r = 0; r < kRows; ++r) {
    for (std::size_t c = 0; c < kColumns; ++c) {
      if (c + 1 < kColumns) {
        options.edges.push_back(
            sx::DeformableEdge{index(c, r), index(c + 1, r), spacing});
      }
      if (r + 1 < kRows) {
        options.edges.push_back(
            sx::DeformableEdge{index(c, r), index(c, r + 1), spacing});
      }
    }
  }
  options.edgeStiffness = 100.0;

  auto body = world.addDeformableBody("stiff_grid", options);

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);

  // First step: the grid is actively settling against the stiff barrier, so it
  // accepts a measurable Newton step.
  world.step(executor, pipeline);
  const double earlyStepInfinityNorm
      = stage.getLastStats().finalStepInfinityNorm;

  for (int step = 0; step < 7; ++step) {
    world.step(executor, pipeline);
  }

  const auto& stats = stage.getLastStats();
  // Feasible: no node penetrates the ground surface.
  for (std::size_t i = 0; i < kNodeCount; ++i) {
    EXPECT_GE(body.getPosition(i).z(), -1e-3);
  }
  // The converged-ness diagnostic is measured (positive while settling) and
  // shrinks toward zero as the configuration reaches equilibrium, even though
  // the barrier Hessian is stiff. This is the honest convergence signal: the
  // last accepted per-node step becomes negligible.
  EXPECT_GT(earlyStepInfinityNorm, 0.0);
  EXPECT_LT(stats.finalStepInfinityNorm, earlyStepInfinityNorm);
  EXPECT_LT(stats.finalStepInfinityNorm, 1e-4);
}

//==============================================================================
// Library-level reproduction of the deformable drape demo:
// a >256-node mat drapes over a raised box ground barrier (a finite-footprint
// step in the support height field) onto the surrounding flat ground. The mat
// is solved on the sparse Newton path and conforms to the step -- center nodes
// held at the box top, overhang nodes draped toward the ground -- so the node
// heights span a clear range without penetrating either surface.
TEST(DeformableBody, SparseProjectedNewtonDrapesMatOverStepBarrier)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);

  // Flat ground barrier (top surface at z = 0).
  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(100.0, 100.0, 0.5)));
  ground.setDeformableGroundBarrier(true);

  // Raised central box barrier: a finite-footprint step with top at z = 0.24.
  constexpr double boxHalf = 0.30;
  constexpr double boxTop = 0.24;
  sx::RigidBodyOptions boxOptions;
  boxOptions.isStatic = true;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.5 * boxTop);
  auto box = world.addRigidBody("step", boxOptions);
  box.setCollisionShape(
      sx::CollisionShape::makeBox(
          Eigen::Vector3d(boxHalf, boxHalf, 0.5 * boxTop)));
  box.setDeformableGroundBarrier(true);

  // 18x16 = 288-node mat (> 256) overhanging the box footprint. It is started
  // near its draped equilibrium -- step-shaped, just above the box top over the
  // footprint and just above the ground beyond it -- so the solve settles in a
  // few iterations (the deformable drape demo runs the full
  // dynamic fall; here we only need to validate the conforming equilibrium).
  constexpr std::size_t kColumns = 18;
  constexpr std::size_t kRows = 16;
  constexpr std::size_t kNodeCount = kColumns * kRows;
  constexpr double spacing = 0.05;
  const double halfWidth = 0.5 * spacing * static_cast<double>(kColumns - 1);
  const double halfDepth = 0.5 * spacing * static_cast<double>(kRows - 1);

  const auto index = [&](std::size_t c, std::size_t r) {
    return r * kColumns + c;
  };

  sx::DeformableBodyOptions options;
  options.edgeStiffness = 25.0;
  options.damping = 1.5;
  options.positions.reserve(kNodeCount);
  for (std::size_t r = 0; r < kRows; ++r) {
    for (std::size_t c = 0; c < kColumns; ++c) {
      const double x = static_cast<double>(c) * spacing - halfWidth;
      const double y = static_cast<double>(r) * spacing - halfDepth;
      const bool overBox = std::abs(x) <= boxHalf && std::abs(y) <= boxHalf;
      options.positions.emplace_back(x, y, overBox ? boxTop + 0.005 : 0.005);
    }
  }
  options.masses.assign(kNodeCount, 0.05);
  for (std::size_t r = 0; r < kRows; ++r) {
    for (std::size_t c = 0; c < kColumns; ++c) {
      if (c + 1 < kColumns) {
        options.edges.push_back(
            sx::DeformableEdge{index(c, r), index(c + 1, r), -1.0});
      }
      if (r + 1 < kRows) {
        options.edges.push_back(
            sx::DeformableEdge{index(c, r), index(c, r + 1), -1.0});
      }
      if (c + 1 < kColumns && r + 1 < kRows) {
        options.edges.push_back(
            sx::DeformableEdge{index(c, r), index(c + 1, r + 1), -1.0});
        options.edges.push_back(
            sx::DeformableEdge{index(c + 1, r), index(c, r + 1), -1.0});
      }
    }
  }

  auto body = world.addDeformableBody("drape_mat", options);

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  for (int step = 0; step < 20; ++step) {
    world.step(executor, pipeline);
  }

  const auto& stats = stage.getLastStats();
  // Two ground barriers (flat ground + raised step), solved on the sparse
  // Newton path above the former dense cap.
  EXPECT_GT(stats.projectedNewtonSteps, 0u);
  EXPECT_EQ(stats.staticGroundBarrierCount, 2u);

  double minZ = std::numeric_limits<double>::infinity();
  double maxZ = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < kNodeCount; ++i) {
    const auto position = body.getPosition(i);
    ASSERT_TRUE(position.allFinite());
    EXPECT_GE(position.z(), -1e-3); // no penetration of either surface
    minZ = std::min(minZ, position.z());
    maxZ = std::max(maxZ, position.z());
  }

  // The mat conforms to the step: some nodes ride the box top while overhang
  // nodes drape toward the ground, so the height range spans a good fraction of
  // the step height (a flat sheet would collapse this range to ~0).
  EXPECT_LT(minZ, 0.1 * boxTop); // overhang nodes reached near the ground
  EXPECT_GT(maxZ, 0.5 * boxTop); // supported nodes remain near the box top
}

//==============================================================================
// A fixed-topology spring body (no contact) has an unchanging Hessian sparsity
// pattern, so the projected-Newton solve reuses its fill-reducing symbolic
// factorization: the first step analyzes the pattern, and a later step performs
// numeric factorizations with zero new symbolic analyses. Behavior is
// unchanged -- this only asserts the analysis is amortized.
TEST(DeformableBody, SparseProjectedNewtonReusesSymbolicFactorization)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);

  constexpr std::size_t kNodeCount = 8;
  constexpr double spacing = 0.1;
  sx::DeformableBodyOptions options;
  for (std::size_t i = 0; i < kNodeCount; ++i) {
    options.positions.emplace_back(static_cast<double>(i) * spacing, 0.0, 0.0);
  }
  options.masses.assign(kNodeCount, 1.0);
  for (std::size_t i = 0; i + 1 < kNodeCount; ++i) {
    options.edges.push_back(sx::DeformableEdge{i, i + 1, spacing});
  }
  options.edgeStiffness = 100.0;
  options.fixedNodes = {0};

  auto body = world.addDeformableBody("symbolic_reuse_chain", options);

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);

  world.step(executor, pipeline); // first step analyzes the sparsity pattern
  EXPECT_GE(stage.getLastStats().projectedNewtonSymbolicFactorizations, 1u);

  world.step(executor, pipeline); // pattern unchanged -> symbolic reused
  const auto& stats = stage.getLastStats();
  EXPECT_GT(stats.projectedNewtonNumericFactorizations, 0u);
  EXPECT_EQ(stats.projectedNewtonSymbolicFactorizations, 0u);

  // The reused factorization still produces a finite, sane solve.
  EXPECT_TRUE(body.getPosition(kNodeCount - 1).allFinite());
}

//==============================================================================
// The iterative (conjugate-gradient) projected-Newton linear solve is a
// matrix-light alternative to the sparse Cholesky factorization: it never
// factorizes, so its memory stays near O(nnz) and it scales to far larger
// meshes. On a small mesh it must reach the same equilibrium as the direct
// solve. This drops an identical FEM cube onto a ground barrier with each
// solver and checks (a) the runs took mutually exclusive solve paths -- the
// direct run only factorized, the iterative run only ran CG and never
// factorized -- and (b) they settle to the same configuration.
TEST(DeformableBody, IterativeLinearSolverMatchesDirectSolve)
{
  struct CubeRun
  {
    std::vector<Eigen::Vector3d> positions;
    std::size_t cgSolves = 0;
    std::size_t numericFactorizations = 0;
    std::size_t symbolicFactorizations = 0;
  };

  const auto runCube = [](bool iterative) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.004);

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
    auto ground = world.addRigidBody("ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
    ground.setDeformableGroundBarrier(true); // top face at z = 0

    auto options = makeFemCubeBody(0.2, Eigen::Vector3d(0.0, 0.0, 0.3), 1.5e5);
    options.material.useIterativeLinearSolver = iterative;
    auto body = world.addDeformableBody("fem_cube", options);

    compute::SequentialExecutor executor;
    compute::DeformableDynamicsStage stage;
    compute::WorldStepPipeline pipeline;
    pipeline.addStage(stage);

    CubeRun run;
    for (int i = 0; i < 200; ++i) {
      world.step(executor, pipeline);
      const auto& stats = stage.getLastStats();
      run.cgSolves += stats.projectedNewtonIterativeSolves;
      run.numericFactorizations += stats.projectedNewtonNumericFactorizations;
      run.symbolicFactorizations += stats.projectedNewtonSymbolicFactorizations;
    }
    for (std::size_t i = 0; i < body.getNodeCount(); ++i) {
      run.positions.push_back(body.getPosition(i));
    }
    return run;
  };

  const CubeRun direct = runCube(false);
  const CubeRun iterative = runCube(true);

  // The direct run factorized and never took the CG path; the iterative run is
  // its mirror image -- all CG, no factorization (neither numeric nor
  // symbolic).
  EXPECT_EQ(direct.cgSolves, 0u);
  EXPECT_GT(direct.numericFactorizations, 0u);
  EXPECT_GT(iterative.cgSolves, 0u);
  EXPECT_EQ(iterative.numericFactorizations, 0u);
  EXPECT_EQ(iterative.symbolicFactorizations, 0u);

  // Same physics: both solvers settle the cube to the same configuration.
  ASSERT_EQ(direct.positions.size(), iterative.positions.size());
  for (std::size_t i = 0; i < direct.positions.size(); ++i) {
    ASSERT_TRUE(iterative.positions[i].allFinite());
    expectVectorNear(iterative.positions[i], direct.positions[i], 1e-4);
  }
}

//==============================================================================
// The matrix-free projected-Newton path evaluates Hessian-vector products from
// local blocks instead of first assembling an Eigen SparseMatrix. It is kept as
// an explicit opt-in so the existing sparse direct and sparse IC-CG paths
// remain unchanged, but on a contact-free FEM cube it should produce the same
// settled configuration while reporting zero sparse Hessian footprint.
TEST(DeformableBody, MatrixFreeLinearSolverMatchesSparseDirectSolve)
{
  struct CubeRun
  {
    std::vector<Eigen::Vector3d> positions;
    std::size_t cgSolves = 0;
    std::size_t matrixFreeSolves = 0;
    std::size_t numericFactorizations = 0;
    std::size_t hessianNonZeros = 0;
  };

  const auto runCube = [](bool matrixFree) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.004);

    auto options = makeFemCubeBody(0.2, Eigen::Vector3d(0.0, 0.0, 0.4), 1.5e5);
    options.fixedNodes = {0, 1, 2, 3};
    options.material.useMatrixFreeLinearSolver = matrixFree;
    auto body = world.addDeformableBody("fem_cube", options);

    compute::SequentialExecutor executor;
    compute::DeformableDynamicsStage stage;
    compute::WorldStepPipeline pipeline;
    pipeline.addStage(stage);

    CubeRun run;
    for (int i = 0; i < 80; ++i) {
      world.step(executor, pipeline);
      const auto& stats = stage.getLastStats();
      run.cgSolves += stats.projectedNewtonIterativeSolves;
      run.matrixFreeSolves += stats.projectedNewtonMatrixFreeSolves;
      run.numericFactorizations += stats.projectedNewtonNumericFactorizations;
      run.hessianNonZeros
          = std::max(run.hessianNonZeros, stats.projectedNewtonHessianNonZeros);
    }
    for (std::size_t i = 0; i < body.getNodeCount(); ++i) {
      run.positions.push_back(body.getPosition(i));
    }
    return run;
  };

  const CubeRun direct = runCube(false);
  const CubeRun matrixFree = runCube(true);

  EXPECT_EQ(direct.cgSolves, 0u);
  EXPECT_EQ(direct.matrixFreeSolves, 0u);
  EXPECT_GT(direct.numericFactorizations, 0u);
  EXPECT_GT(direct.hessianNonZeros, 0u);

  EXPECT_GT(matrixFree.cgSolves, 0u);
  EXPECT_EQ(matrixFree.cgSolves, matrixFree.matrixFreeSolves);
  EXPECT_EQ(matrixFree.numericFactorizations, 0u);
  EXPECT_EQ(matrixFree.hessianNonZeros, 0u);

  ASSERT_EQ(direct.positions.size(), matrixFree.positions.size());
  for (std::size_t i = 0; i < direct.positions.size(); ++i) {
    ASSERT_TRUE(matrixFree.positions[i].allFinite());
    expectVectorNear(matrixFree.positions[i], direct.positions[i], 1e-4);
  }
}

//==============================================================================
// Ground contact adds a stiff barrier block to the projected-Newton Hessian.
// This compares all three solve paths on the same contacting FEM cube: sparse
// direct, sparse IC-CG, and matrix-free CG. The matrix-free path must stay on
// Hessian-vector products (zero sparse footprint) while reaching the same
// contact equilibrium as the assembled sparse solvers.
TEST(DeformableBody, MatrixFreeLinearSolverMatchesSparseSolversOnGroundContact)
{
  enum class SolveMode
  {
    Direct,
    SparseCg,
    MatrixFreeCg,
  };

  struct CubeRun
  {
    std::vector<Eigen::Vector3d> positions;
    std::size_t cgSolves = 0;
    std::size_t matrixFreeSolves = 0;
    std::size_t numericFactorizations = 0;
    std::size_t hessianNonZeros = 0;
    std::size_t hessianStorageBytes = 0;
    double minZ = std::numeric_limits<double>::infinity();
  };

  const auto runCube = [](SolveMode mode) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.004);

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
    auto ground = world.addRigidBody("ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
    ground.setDeformableGroundBarrier(true); // top face at z = 0

    auto options = makeFemCubeBody(0.2, Eigen::Vector3d(0.0, 0.0, 0.3), 1.5e5);
    options.material.useIterativeLinearSolver = mode == SolveMode::SparseCg;
    options.material.useMatrixFreeLinearSolver
        = mode == SolveMode::MatrixFreeCg;
    auto body = world.addDeformableBody("fem_cube", options);

    compute::SequentialExecutor executor;
    compute::DeformableDynamicsStage stage;
    compute::WorldStepPipeline pipeline;
    pipeline.addStage(stage);

    CubeRun run;
    for (int i = 0; i < 200; ++i) {
      world.step(executor, pipeline);
      const auto& stats = stage.getLastStats();
      run.cgSolves += stats.projectedNewtonIterativeSolves;
      run.matrixFreeSolves += stats.projectedNewtonMatrixFreeSolves;
      run.numericFactorizations += stats.projectedNewtonNumericFactorizations;
      run.hessianNonZeros
          = std::max(run.hessianNonZeros, stats.projectedNewtonHessianNonZeros);
      run.hessianStorageBytes = std::max(
          run.hessianStorageBytes, stats.projectedNewtonHessianStorageBytes);
    }
    for (std::size_t i = 0; i < body.getNodeCount(); ++i) {
      run.positions.push_back(body.getPosition(i));
      run.minZ = std::min(run.minZ, body.getPosition(i).z());
    }
    return run;
  };

  const CubeRun direct = runCube(SolveMode::Direct);
  const CubeRun sparseCg = runCube(SolveMode::SparseCg);
  const CubeRun matrixFree = runCube(SolveMode::MatrixFreeCg);

  EXPECT_EQ(direct.cgSolves, 0u);
  EXPECT_EQ(direct.matrixFreeSolves, 0u);
  EXPECT_GT(direct.numericFactorizations, 0u);
  EXPECT_GT(direct.hessianNonZeros, 0u);
  EXPECT_GT(direct.hessianStorageBytes, 0u);

  EXPECT_GT(sparseCg.cgSolves, 0u);
  EXPECT_EQ(sparseCg.matrixFreeSolves, 0u);
  EXPECT_EQ(sparseCg.numericFactorizations, 0u);
  EXPECT_GT(sparseCg.hessianNonZeros, 0u);
  EXPECT_GT(sparseCg.hessianStorageBytes, 0u);

  EXPECT_GT(matrixFree.cgSolves, 0u);
  EXPECT_EQ(matrixFree.cgSolves, matrixFree.matrixFreeSolves);
  EXPECT_EQ(matrixFree.numericFactorizations, 0u);
  EXPECT_EQ(matrixFree.hessianNonZeros, 0u);
  EXPECT_EQ(matrixFree.hessianStorageBytes, 0u);

  EXPECT_GE(direct.minZ, -1e-3);
  EXPECT_GE(sparseCg.minZ, -1e-3);
  EXPECT_GE(matrixFree.minZ, -1e-3);

  ASSERT_EQ(direct.positions.size(), sparseCg.positions.size());
  ASSERT_EQ(direct.positions.size(), matrixFree.positions.size());
  for (std::size_t i = 0; i < direct.positions.size(); ++i) {
    ASSERT_TRUE(sparseCg.positions[i].allFinite());
    ASSERT_TRUE(matrixFree.positions[i].allFinite());
    expectVectorNear(sparseCg.positions[i], direct.positions[i], 1e-4);
    expectVectorNear(matrixFree.positions[i], direct.positions[i], 1e-4);
  }
}

//==============================================================================
// Stiff barrier contact produces an ill-conditioned Hessian; a weak diagonal
// (Jacobi) CG preconditioner stalls there and forces the iterative solver to
// fall back to steepest descent. The incomplete-Cholesky preconditioner
// collapses the CG iteration count so the iterative path carries the solve.
// This drops a stiff FEM cube onto a ground barrier with the iterative solver
// and checks it settles intersection-free through the CG path (never
// factorizing), that accepted CG steps dominate the fallbacks (the
// preconditioner is strong enough that CG -- not steepest descent -- does the
// work), and that it reaches the same equilibrium as the direct solver on the
// identical stiff scene.
TEST(DeformableBody, IterativeSolverConvergesOnStiffGroundContact)
{
  struct StiffRun
  {
    std::vector<Eigen::Vector3d> positions;
    std::size_t cgSolves = 0;
    std::size_t numericFactorizations = 0;
    std::size_t fallbacks = 0;
    double minZ = std::numeric_limits<double>::infinity();
  };

  const auto runStiffCube = [](bool iterative) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.004);

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
    auto ground = world.addRigidBody("ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
    ground.setDeformableGroundBarrier(true); // top face at z = 0

    // A stiffer cube (E = 1e6) than the soft-contact equivalence test, so the
    // contact Hessian is ill-conditioned enough to exercise the preconditioner.
    auto options = makeFemCubeBody(0.2, Eigen::Vector3d(0.0, 0.0, 0.3), 1.0e6);
    options.material.useIterativeLinearSolver = iterative;
    auto body = world.addDeformableBody("stiff_cube", options);

    compute::SequentialExecutor executor;
    compute::DeformableDynamicsStage stage;
    compute::WorldStepPipeline pipeline;
    pipeline.addStage(stage);

    StiffRun run;
    for (int i = 0; i < 250; ++i) {
      world.step(executor, pipeline);
      const auto& stats = stage.getLastStats();
      run.cgSolves += stats.projectedNewtonIterativeSolves;
      run.numericFactorizations += stats.projectedNewtonNumericFactorizations;
      run.fallbacks += stats.projectedNewtonFallbacks;
    }
    for (std::size_t i = 0; i < body.getNodeCount(); ++i) {
      run.positions.push_back(body.getPosition(i));
      run.minZ = std::min(run.minZ, body.getPosition(i).z());
    }
    return run;
  };

  const StiffRun direct = runStiffCube(false);
  const StiffRun iterative = runStiffCube(true);

  // The iterative run took the CG path and never factorized.
  EXPECT_GT(iterative.cgSolves, 0u);
  EXPECT_EQ(iterative.numericFactorizations, 0u);

  // Both settle intersection-free above the ground top (z = 0).
  EXPECT_GE(direct.minZ, -1e-3);
  EXPECT_GE(iterative.minZ, -1e-3);

  // The incomplete-Cholesky CG carries the solve on this stiff contact:
  // accepted CG steps strictly dominate the fallbacks. A diagonal
  // preconditioner would stall on the ill-conditioned contact Hessian and let
  // fallbacks pile up.
  EXPECT_GT(iterative.cgSolves, iterative.fallbacks);

  // Same physics: the iterative solver reaches the same stiff-contact
  // equilibrium as the direct solver.
  ASSERT_EQ(direct.positions.size(), iterative.positions.size());
  for (std::size_t i = 0; i < direct.positions.size(); ++i) {
    ASSERT_TRUE(iterative.positions[i].allFinite());
    expectVectorNear(iterative.positions[i], direct.positions[i], 2e-3);
  }
}

//==============================================================================
// A chunky 3D FEM cube (solid N^3 cells, wide Hessian bandwidth) is exactly the
// regime the iterative solver targets: the sparse Cholesky direct solve suffers
// super-linear 3D fill-in there while the conjugate gradient stays near O(nnz).
// This pins such a cube at its x == 0 face, lets it sag under gravity, and
// confirms the iterative solver reaches the same equilibrium as the direct
// solve on the wide-bandwidth mesh (the correctness guarantee behind the
// BM_DeformableCube3d* scaling benchmarks), through the CG path (no
// factorization).
TEST(DeformableBody, IterativeSolverMatchesDirectOnChunky3dMesh)
{
  constexpr int kCells = 3; // 4^3 = 64 nodes, a solid 3D block
  constexpr double h = 0.1;
  const int n = kCells + 1;
  const auto nodeIndex = [&](int i, int j, int k) {
    return static_cast<std::size_t>(i + n * (j + n * k));
  };
  static constexpr int kCubeTets[6][4]
      = {{0, 1, 3, 7},
         {0, 3, 2, 7},
         {0, 2, 6, 7},
         {0, 6, 4, 7},
         {0, 4, 5, 7},
         {0, 5, 1, 7}};

  const auto runCube = [&](bool iterative) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.005);

    sx::DeformableBodyOptions options;
    options.material.youngsModulus = 2.0e5;
    options.material.poissonRatio = 0.3;
    options.material.useFiniteElementElasticity = true;
    options.material.useIterativeLinearSolver = iterative;
    for (int k = 0; k < n; ++k) {
      for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
          options.positions.emplace_back(i * h, j * h, 0.5 + k * h);
        }
      }
    }
    for (int ck = 0; ck < kCells; ++ck) {
      for (int cj = 0; cj < kCells; ++cj) {
        for (int ci = 0; ci < kCells; ++ci) {
          std::array<std::size_t, 8> corner{};
          for (int b = 0; b < 8; ++b) {
            corner[static_cast<std::size_t>(b)] = nodeIndex(
                ci + (b & 1), cj + ((b >> 1) & 1), ck + ((b >> 2) & 1));
          }
          for (const auto& tet : kCubeTets) {
            options.tetrahedra.push_back(
                sx::DeformableTetrahedron{
                    corner[static_cast<std::size_t>(tet[0])],
                    corner[static_cast<std::size_t>(tet[1])],
                    corner[static_cast<std::size_t>(tet[2])],
                    corner[static_cast<std::size_t>(tet[3])]});
          }
        }
      }
    }
    for (int k = 0; k < n; ++k) {
      for (int j = 0; j < n; ++j) {
        options.fixedNodes.push_back(nodeIndex(0, j, k));
      }
    }

    auto body = world.addDeformableBody("chunky_cube", options);
    compute::SequentialExecutor executor;
    compute::DeformableDynamicsStage stage;
    compute::WorldStepPipeline pipeline;
    pipeline.addStage(stage);

    std::size_t cgSolves = 0;
    std::size_t cgIterations = 0;
    std::size_t numericFactorizations = 0;
    for (int i = 0; i < 80; ++i) {
      world.step(executor, pipeline);
      cgSolves += stage.getLastStats().projectedNewtonIterativeSolves;
      cgIterations += stage.getLastStats().projectedNewtonIterativeIterations;
      numericFactorizations
          += stage.getLastStats().projectedNewtonNumericFactorizations;
    }
    std::vector<Eigen::Vector3d> positions;
    for (std::size_t i = 0; i < body.getNodeCount(); ++i) {
      positions.push_back(body.getPosition(i));
    }
    return std::make_tuple(
        positions, cgSolves, cgIterations, numericFactorizations);
  };

  const auto [directPos, directCg, directCgIters, directFact] = runCube(false);
  const auto [iterPos, iterCg, iterCgIters, iterFact] = runCube(true);

  // Mutually exclusive solve paths, as on the smaller meshes.
  EXPECT_EQ(directCg, 0u);
  EXPECT_EQ(directCgIters, 0u);
  EXPECT_GT(directFact, 0u);
  EXPECT_GT(iterCg, 0u);
  EXPECT_GT(iterCgIters, 0u);
  EXPECT_EQ(iterFact, 0u);

  // Same sagged equilibrium on the wide-bandwidth 3D mesh.
  ASSERT_EQ(directPos.size(), iterPos.size());
  for (std::size_t i = 0; i < directPos.size(); ++i) {
    ASSERT_TRUE(iterPos[i].allFinite());
    expectVectorNear(iterPos[i], directPos[i], 1e-4);
  }
}

namespace {
struct GroundSlideResult
{
  double x = 0.0;
  double vx = 0.0;
  double z = 0.0;
};

// Slide a single node along a static ground barrier under gravity with the
// given Coulomb friction coefficient, returning its final tangential position,
// tangential velocity, and height after `steps` steps.
GroundSlideResult runGroundFrictionSlide(double frictionCoefficient, int steps)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(100.0, 100.0, 0.5)));
  ground.setDeformableGroundBarrier(true);

  // Start inside the barrier band (d_hat = 2e-2) with a tangential velocity.
  auto options = makeSingleNodeBodyOptions(
      Eigen::Vector3d(0.0, 0.0, 0.01), Eigen::Vector3d(2.0, 0.0, 0.0));
  options.material.frictionCoefficient = frictionCoefficient;
  auto body = world.addDeformableBody("slider", options);

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  for (int i = 0; i < steps; ++i) {
    world.step(executor, pipeline);
  }

  const auto position = body.getPosition(0);
  const auto velocity = body.getVelocity(0);
  return {position.x(), velocity.x(), position.z()};
}
} // namespace

//==============================================================================
// Coulomb friction (mu > 0) opposes a node sliding while in static-ground
// contact: it travels less far and ends slower than the frictionless control,
// while staying in non-penetrating contact.
TEST(DeformableBody, GroundFrictionDeceleratesSlidingNode)
{
  const auto frictionless = runGroundFrictionSlide(0.0, 30);
  const auto frictional = runGroundFrictionSlide(0.8, 30);

  EXPECT_GE(frictionless.z, -1e-3);
  EXPECT_GE(frictional.z, -1e-3);

  // The frictionless node slides essentially freely (gravity is vertical).
  EXPECT_GT(frictionless.x, 0.4);
  EXPECT_NEAR(frictionless.vx, 2.0, 0.2);

  // Friction opposes the slide: shorter distance and lower final speed.
  EXPECT_LT(frictional.x, frictionless.x);
  EXPECT_LT(frictional.vx, frictionless.vx);
  EXPECT_GE(frictional.x, 0.0); // not pushed backward
}

//==============================================================================
// Friction is inactive without contact: a node above the barrier band has no
// normal force, so a high friction coefficient does not slow its free slide.
TEST(DeformableBody, GroundFrictionInactiveWithoutGroundContact)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(100.0, 100.0, 0.5)));
  ground.setDeformableGroundBarrier(true);

  auto options = makeSingleNodeBodyOptions(
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(2.0, 0.0, 0.0));
  options.material.frictionCoefficient = 1.0;
  auto body = world.addDeformableBody("floater", options);

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  for (int i = 0; i < 10; ++i) {
    world.step(executor, pipeline);
  }

  // Out of contact: slides freely at constant velocity (no friction force).
  EXPECT_NEAR(body.getVelocity(0).x(), 2.0, 1e-6);
  EXPECT_NEAR(body.getPosition(0).x(), 2.0 * 0.01 * 10, 1e-3);
}

namespace {
struct TiltedGroundSlideResult
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
};

// Drive a single node in the ground-barrier band just above a 45-degree tilted
// box surface (top-face normal (1, 0, 1)/sqrt(2)) with the given initial
// velocity and Coulomb friction coefficient, returning its final position and
// velocity after `steps` steps under zero gravity.
TiltedGroundSlideResult runTiltedGroundFrictionSlide(
    double frictionCoefficient,
    const Eigen::Vector3d& initialVelocity,
    int steps)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.orientation = Eigen::AngleAxisd(
      0.25 * 3.14159265358979323846, Eigen::Vector3d::UnitY());
  auto ground = world.addRigidBody("tilted_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(4.0, 4.0, 0.2)));
  ground.setDeformableGroundBarrier(true);

  // The tilted top face directly above (0, 0) sits at z = sqrt(2) * 0.2 ~=
  // 0.283; start ~0.007 into the d_hat = 2e-2 activation band.
  auto options = makeSingleNodeBodyOptions(
      Eigen::Vector3d(0.0, 0.0, 0.29), initialVelocity);
  options.material.frictionCoefficient = frictionCoefficient;
  auto body = world.addDeformableBody("tilted_slider", options);

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  for (int i = 0; i < steps; ++i) {
    world.step(executor, pipeline);
  }
  return {body.getPosition(0), body.getVelocity(0)};
}
} // namespace

//==============================================================================
// Friction resolves its tangent plane against the true geometric ground normal,
// not a hardcoded xy plane. The static-ground barrier is a vertical height
// field, so a node dropped straight down (-z) onto a 45-degree tilted slope
// feels no horizontal force from the barrier or (zero) gravity: the
// frictionless control stays exactly on the x = 0 line. Tilt-aware friction,
// however, couples the normal and tangential directions through the slope's
// tilted tangent plane (normal (1, 0, 1)/sqrt(2)), so the arrested vertical
// impact deflects the node down-slope in +x -- the contact behaves like a real
// incline. An xy-only tangent model has no x/z coupling and would leave x at
// zero, so the nonzero +x deflection is the signature of the tilt-aware tangent
// basis.
TEST(DeformableBody, GroundFrictionFollowsTiltedSlopeNormal)
{
  const Eigen::Vector3d drop(0.0, 0.0, -2.0);
  const auto frictionless = runTiltedGroundFrictionSlide(0.0, drop, 20);
  const auto frictional = runTiltedGroundFrictionSlide(1.0, drop, 20);

  // Frictionless: only the vertical barrier and (zero) gravity act, so x is
  // untouched to machine precision.
  EXPECT_NEAR(frictionless.position.x(), 0.0, 1e-9);

  // Tilt-aware friction couples normal/tangential and deflects the vertical
  // drop down-slope (+x); an xy-only tangent model would leave x at zero here.
  EXPECT_GT(frictional.position.x(), 1e-3);

  // Both stay above the tilted surface (no deep penetration of the ~0.283
  // contact height directly above the origin).
  EXPECT_GT(frictional.position.z(), 0.25);
  EXPECT_GT(frictionless.position.z(), 0.25);
}

//==============================================================================
// The solver reports friction diagnostics for the step: a node sliding in
// static-ground contact dissipates a positive friction energy over a nonzero
// active friction-contact set, while the frictionless control reports exactly
// zero for both (the diagnostic is gated on a positive friction coefficient).
TEST(DeformableBody, FrictionDiagnosticsReportSlidingDissipation)
{
  const auto runWithStats = [](double frictionCoefficient) {
    sx::World world;
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
    auto ground = world.addRigidBody("ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(100.0, 100.0, 0.5)));
    ground.setDeformableGroundBarrier(true);

    // Start in the d_hat = 2e-2 band with a tangential velocity so the node
    // slides while staying in static-ground contact.
    auto options = makeSingleNodeBodyOptions(
        Eigen::Vector3d(0.0, 0.0, 0.01), Eigen::Vector3d(2.0, 0.0, 0.0));
    options.material.frictionCoefficient = frictionCoefficient;
    world.addDeformableBody("slider", options);

    compute::SequentialExecutor executor;
    compute::DeformableDynamicsStage stage;
    compute::WorldStepPipeline pipeline;
    pipeline.addStage(stage);
    for (int i = 0; i < 5; ++i) {
      world.step(executor, pipeline);
    }
    return stage.getLastStats();
  };

  const auto frictionless = runWithStats(0.0);
  const auto frictional = runWithStats(0.8);

  // Frictionless: the diagnostic is disabled, so no dissipation and no active
  // friction contacts are reported.
  EXPECT_EQ(frictionless.frictionDissipation, 0.0);
  EXPECT_EQ(frictionless.activeFrictionContacts, 0u);

  // Frictional sliding node in contact: positive dissipation over at least one
  // active friction contact.
  EXPECT_GT(frictional.frictionDissipation, 0.0);
  EXPECT_GE(frictional.activeFrictionContacts, 1u);
}

namespace {
struct SelfContactSlideResult
{
  double centroidX = 0.0;
  double minUpperZ = 0.0;
};

// Drive an upper triangle in self-contact with a fixed lower triangle, with a
// tangential (+x) slide velocity and the given friction coefficient. Returns
// the upper triangle's final centroid x and minimum height.
SelfContactSlideResult runSelfContactFrictionSlide(
    double frictionCoefficient, int steps)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  // Lower triangle fixed at z=0; upper triangle just inside the barrier band.
  auto options = makeTwoFacingTrianglesOptions(0.012, 0.1);
  // Give the free (upper) triangle a tangential slide on top of the gentle
  // downward press that keeps it in contact.
  for (std::size_t i = 3; i < 6; ++i) {
    options.velocities[i] = Eigen::Vector3d(0.5, 0.0, -0.1);
  }
  options.material.frictionCoefficient = frictionCoefficient;
  auto body = world.addDeformableBody("facing_friction", options);

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  for (int i = 0; i < steps; ++i) {
    world.step(executor, pipeline);
  }

  const double centroidX = (body.getPosition(3).x() + body.getPosition(4).x()
                            + body.getPosition(5).x())
                           / 3.0;
  const double minUpperZ = std::min(
      {body.getPosition(3).z(),
       body.getPosition(4).z(),
       body.getPosition(5).z()});
  return {centroidX, minUpperZ};
}
} // namespace

//==============================================================================
// Self-contact Coulomb friction (mu > 0) opposes one deformable surface sliding
// tangentially against another while in self-contact: the upper triangle
// travels less far than the frictionless control, while the barrier keeps the
// surfaces separated.
TEST(DeformableBody, SelfContactFrictionDeceleratesSlidingSurface)
{
  const auto frictionless = runSelfContactFrictionSlide(0.0, 20);
  const auto frictional = runSelfContactFrictionSlide(0.8, 20);

  // The self-contact barrier holds the upper surface above the lower (z=0).
  EXPECT_GT(frictionless.minUpperZ, 0.0);
  EXPECT_GT(frictional.minUpperZ, 0.0);

  // Frictionless: the upper triangle slides tangentially essentially freely
  // (the barrier force is normal, not tangential).
  EXPECT_GT(frictionless.centroidX, 0.05);

  // Friction opposes the tangential slide: shorter travel.
  EXPECT_LT(frictional.centroidX, frictionless.centroidX);
  EXPECT_GE(frictional.centroidX, 0.0); // not pushed backward
}

namespace {
// Two near-orthogonal thin triangles whose long edges cross with a small
// vertical gap form an edge-edge self-contact. The lower triangle is fixed; the
// upper one is pressed down and slid tangentially with the given friction.
SelfContactSlideResult runEdgeEdgeFrictionSlide(
    double frictionCoefficient, int steps)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  constexpr double gap = 0.012; // inside the self-contact band (d_hat = 2e-2)
  sx::DeformableBodyOptions options;
  options.edgeStiffness = 0.0; // isolate the contact response (no springs)
  options.damping = 0.0;
  // Lower triangle: a thin sliver along x at z = 0.
  options.positions = {
      Eigen::Vector3d(-0.6, 0.0, 0.0),
      Eigen::Vector3d(0.6, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.06, 0.0),
      // Upper triangle: a thin sliver along y at z = gap (edge crosses above).
      Eigen::Vector3d(0.0, -0.6, gap),
      Eigen::Vector3d(0.0, 0.6, gap),
      Eigen::Vector3d(0.06, 0.0, gap)};
  options.velocities
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d(0.5, 0.0, -0.1),
         Eigen::Vector3d(0.5, 0.0, -0.1),
         Eigen::Vector3d(0.5, 0.0, -0.1)};
  options.masses = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  options.fixedNodes = {0, 1, 2};
  options.surfaceTriangles
      = {sx::DeformableSurfaceTriangle{0, 1, 2},
         sx::DeformableSurfaceTriangle{3, 4, 5}};
  options.material.frictionCoefficient = frictionCoefficient;
  auto body = world.addDeformableBody("crossing_edges", options);

  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);
  for (int i = 0; i < steps; ++i) {
    world.step(executor, pipeline);
  }

  const double centroidX = (body.getPosition(3).x() + body.getPosition(4).x()
                            + body.getPosition(5).x())
                           / 3.0;
  const double minUpperZ = std::min(
      {body.getPosition(3).z(),
       body.getPosition(4).z(),
       body.getPosition(5).z()});
  return {centroidX, minUpperZ};
}
} // namespace

//==============================================================================
// Edge-edge self-contact friction (mu > 0) opposes one crossing edge sliding
// tangentially over another while the edge-edge barrier holds them apart.
TEST(DeformableBody, EdgeEdgeSelfContactFrictionDeceleratesSlidingEdge)
{
  const auto frictionless = runEdgeEdgeFrictionSlide(0.0, 20);
  const auto frictional = runEdgeEdgeFrictionSlide(0.8, 20);

  // The edge-edge barrier keeps the upper edge above the lower (z = 0).
  EXPECT_GT(frictionless.minUpperZ, 0.0);
  EXPECT_GT(frictional.minUpperZ, 0.0);

  // Frictionless: the upper edge slides tangentially essentially freely.
  EXPECT_GT(frictionless.centroidX, 0.05);

  // Friction opposes the tangential slide: shorter travel.
  EXPECT_LT(frictional.centroidX, frictionless.centroidX);
  EXPECT_GE(frictional.centroidX, 0.0);
}
