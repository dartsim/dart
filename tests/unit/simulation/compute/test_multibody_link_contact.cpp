/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/comps/multibody.hpp>
#include <dart/simulation/compute/multibody_dynamics.hpp>
#include <dart/simulation/detail/entity_conversion.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/multibody/joint.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>

#include <dart/common/memory_manager.hpp>

#include <entt/entt.hpp>
#include <gtest/gtest.h>

#include <vector>

#include <cmath>

namespace {

namespace sx = dart::simulation;

TEST(MultibodyLinkContact, ProblemRowsUseProvidedAllocator)
{
  namespace common = dart::common;

  common::MemoryManager memoryManager;
  auto& freeList = memoryManager.getFreeListAllocator();
  const auto allocationsBeforeProblem = freeList.getAllocationCount();

  {
    sx::compute::MultibodyLinkContactProblem problem(
        memoryManager.getFreeAllocator());
    const common::StlAllocator<sx::compute::MultibodyLinkContactRow>
        expectedAllocator{memoryManager.getFreeAllocator()};
    EXPECT_EQ(problem.rows.get_allocator(), expectedAllocator);

    problem.rows.reserve(4);
    EXPECT_GT(freeList.getAllocationCount(), allocationsBeforeProblem)
        << "allocator-aware multibody contact rows should reserve from the "
           "provided free allocator";
  }

  EXPECT_EQ(freeList.getAllocationCount(), allocationsBeforeProblem);
}

//==============================================================================
// The diagonal Delassus denominator the assembler must store for one row:
// J M^-1 J^T plus a two-sided rigid obstacle's point inverse mass (zero when
// the obstacle is immovable or absent). Recomputing it from the stored row
// fields locks the assembler's contract: the denominator must agree with the
// Jacobian, inverse mass, arm, and inverse inertia it actually saved.
double expectedDenominator(
    const sx::compute::MultibodyLinkContactRow& row,
    const Eigen::MatrixXd& inverseMass,
    const Eigen::VectorXd& jacobian,
    const Eigen::Vector3d& direction)
{
  double value = jacobian.dot(inverseMass * jacobian) + row.otherInvMass;
  const Eigen::Vector3d arm = row.otherArm.cross(direction);
  value += arm.dot(row.otherInvInertia * arm);
  return value;
}

//==============================================================================
void expectRowsExactlyEqual(
    const sx::compute::MultibodyLinkContactRow& lhs,
    const sx::compute::MultibodyLinkContactRow& rhs)
{
  EXPECT_EQ(lhs.active, rhs.active);
  EXPECT_EQ(lhs.otherBody, rhs.otherBody);
  EXPECT_EQ(lhs.otherLink, rhs.otherLink);
  EXPECT_EQ(lhs.otherMultibody, rhs.otherMultibody);
  EXPECT_EQ(lhs.otherMultibodyIndex, rhs.otherMultibodyIndex);
  EXPECT_EQ(lhs.normalJacobian, rhs.normalJacobian);
  EXPECT_EQ(lhs.tangentJacobian1, rhs.tangentJacobian1);
  EXPECT_EQ(lhs.tangentJacobian2, rhs.tangentJacobian2);
  EXPECT_EQ(lhs.otherNormalJacobian, rhs.otherNormalJacobian);
  EXPECT_EQ(lhs.otherTangentJacobian1, rhs.otherTangentJacobian1);
  EXPECT_EQ(lhs.otherTangentJacobian2, rhs.otherTangentJacobian2);
  EXPECT_EQ(lhs.normal, rhs.normal);
  EXPECT_EQ(lhs.tangent1, rhs.tangent1);
  EXPECT_EQ(lhs.tangent2, rhs.tangent2);
  EXPECT_EQ(lhs.point, rhs.point);
  EXPECT_EQ(lhs.otherArm, rhs.otherArm);
  EXPECT_EQ(lhs.otherInvInertia, rhs.otherInvInertia);
  EXPECT_DOUBLE_EQ(lhs.normalDenominator, rhs.normalDenominator);
  EXPECT_DOUBLE_EQ(lhs.tangentDenominator1, rhs.tangentDenominator1);
  EXPECT_DOUBLE_EQ(lhs.tangentDenominator2, rhs.tangentDenominator2);
  EXPECT_DOUBLE_EQ(lhs.bias, rhs.bias);
  EXPECT_DOUBLE_EQ(lhs.restitutionTarget, rhs.restitutionTarget);
  EXPECT_DOUBLE_EQ(lhs.friction, rhs.friction);
  EXPECT_DOUBLE_EQ(lhs.normalRhs, rhs.normalRhs);
  EXPECT_DOUBLE_EQ(lhs.tangentRhs1, rhs.tangentRhs1);
  EXPECT_DOUBLE_EQ(lhs.tangentRhs2, rhs.tangentRhs2);
  EXPECT_DOUBLE_EQ(lhs.restitution, rhs.restitution);
  EXPECT_DOUBLE_EQ(lhs.otherInvMass, rhs.otherInvMass);
}

//==============================================================================
// A fixed-base multibody with one unit-mass prismatic link sliding along +Z.
struct PrismaticLegWorld
{
  sx::World world;
  entt::entity multibody = entt::null;
  entt::entity link = entt::null;

  PrismaticLegWorld()
  {
    world.setGravity(Eigen::Vector3d::Zero());
    auto robot = world.addMultibody("robot");
    auto base = robot.addLink("base");

    sx::JointSpec spec;
    spec.name = "slider";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitZ();
    auto leg = robot.addLink("leg", base, spec);
    leg.setMass(1.0);

    multibody = dart::simulation::detail::toRegistryEntity(robot.getEntity());
    link = dart::simulation::detail::toRegistryEntity(leg.getEntity());
  }

  const sx::comps::MultibodyStructure& structure()
  {
    return dart::simulation::detail::registryOf(world)
        .get<sx::comps::MultibodyStructure>(multibody);
  }
};

//==============================================================================
// A fixed-base multibody with two independent unit-mass prismatic links sliding
// along +Z.
struct TwoPrismaticLinksWorld
{
  sx::World world;
  entt::entity multibody = entt::null;
  entt::entity lower = entt::null;
  entt::entity upper = entt::null;

  TwoPrismaticLinksWorld()
  {
    world.setGravity(Eigen::Vector3d::Zero());
    auto robot = world.addMultibody("robot");
    auto base = robot.addLink("base");

    sx::JointSpec lowerSpec;
    lowerSpec.name = "lower";
    lowerSpec.type = sx::JointType::Prismatic;
    lowerSpec.axis = Eigen::Vector3d::UnitZ();
    auto lowerLink = robot.addLink("lower", base, lowerSpec);
    lowerLink.setMass(1.0);

    sx::JointSpec upperSpec;
    upperSpec.name = "upper";
    upperSpec.type = sx::JointType::Prismatic;
    upperSpec.axis = Eigen::Vector3d::UnitZ();
    auto upperLink = robot.addLink("upper", base, upperSpec);
    upperLink.setMass(1.0);

    multibody = dart::simulation::detail::toRegistryEntity(robot.getEntity());
    lower = dart::simulation::detail::toRegistryEntity(lowerLink.getEntity());
    upper = dart::simulation::detail::toRegistryEntity(upperLink.getEntity());
  }

  const sx::comps::MultibodyStructure& structure()
  {
    return dart::simulation::detail::registryOf(world)
        .get<sx::comps::MultibodyStructure>(multibody);
  }
};

} // namespace

//==============================================================================
TEST(MultibodyLinkContact, AssemblesOneSidedLinkContactRowDeterministically)
{
  PrismaticLegWorld scene;

  // A static ground below the leg pushes up into the link (normal +Z). The leg
  // is descending, so the contact is approaching and restitution is active.
  sx::compute::LinkContact contact;
  contact.link = scene.link;
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.point = Eigen::Vector3d(0.3, -0.2, 0.0);
  contact.depth = 0.02;
  contact.friction = 0.5;
  contact.restitution = 0.3;
  contact.otherBody = entt::null; // immovable obstacle: one-sided

  Eigen::VectorXd nextVelocity(1);
  nextVelocity << -0.5;

  const std::vector<sx::compute::LinkContact> contacts{contact};
  const double timeStep = 0.01;

  const auto problem = sx::compute::assembleMultibodyLinkContactProblem(
      dart::simulation::detail::registryOf(scene.world),
      scene.structure(),
      nextVelocity,
      timeStep,
      contacts);
  const auto repeated = sx::compute::assembleMultibodyLinkContactProblem(
      dart::simulation::detail::registryOf(scene.world),
      scene.structure(),
      nextVelocity,
      timeStep,
      contacts);

  ASSERT_EQ(problem.rows.size(), 1u);
  ASSERT_EQ(problem.inverseMass.rows(), 1);
  ASSERT_EQ(problem.inverseMass.cols(), 1);
  // Prismatic translation of a unit point mass: joint-space M is the mass.
  EXPECT_NEAR(problem.inverseMass(0, 0), 1.0, 1e-12);

  const auto& row = problem.rows[0];
  EXPECT_TRUE(row.active);
  EXPECT_TRUE(row.otherBody == entt::null);
  EXPECT_DOUBLE_EQ(row.otherInvMass, 0.0);
  EXPECT_TRUE(row.otherInvInertia.isZero(0.0));
  EXPECT_TRUE(row.otherArm.isZero(0.0));

  // Material and Baumgarte fields are carried verbatim / by the fixed formula.
  EXPECT_DOUBLE_EQ(row.friction, 0.5);
  EXPECT_DOUBLE_EQ(
      row.bias, 0.2 * std::max(0.0, contact.depth - 1e-4) / timeStep);

  // Normal and tangents form an orthonormal contact frame.
  EXPECT_TRUE(row.normal.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_NEAR(row.normal.dot(row.tangent1), 0.0, 1e-12);
  EXPECT_NEAR(row.normal.dot(row.tangent2), 0.0, 1e-12);
  EXPECT_NEAR(row.tangent1.dot(row.tangent2), 0.0, 1e-12);
  EXPECT_NEAR(row.tangent1.norm(), 1.0, 1e-12);
  EXPECT_NEAR(row.tangent2.norm(), 1.0, 1e-12);

  // Denominators are J M^-1 J^T (one-sided: no obstacle compliance term).
  EXPECT_DOUBLE_EQ(
      row.normalDenominator,
      expectedDenominator(
          row, problem.inverseMass, row.normalJacobian, row.normal));
  EXPECT_DOUBLE_EQ(
      row.tangentDenominator1,
      expectedDenominator(
          row, problem.inverseMass, row.tangentJacobian1, row.tangent1));
  EXPECT_DOUBLE_EQ(
      row.tangentDenominator2,
      expectedDenominator(
          row, problem.inverseMass, row.tangentJacobian2, row.tangent2));
  EXPECT_GT(row.normalDenominator, 0.0);

  // Restitution rebounds the pre-solve approach velocity above the threshold.
  const double approach = row.normalJacobian.dot(nextVelocity);
  const double expectedRestitution
      = (approach < -1e-2) ? -contact.restitution * approach : 0.0;
  EXPECT_DOUBLE_EQ(row.restitutionTarget, expectedRestitution);
  EXPECT_GT(row.restitutionTarget, 0.0); // the leg is descending into ground

  // Boxed-LCP right-hand sides: the normal target mirrors the Gauss-Seidel
  // normal update; the tangent targets drive tangential relative velocity to
  // zero (one-sided, so no obstacle term).
  EXPECT_DOUBLE_EQ(
      row.normalRhs, -approach + std::max(row.bias, row.restitutionTarget));
  EXPECT_DOUBLE_EQ(row.tangentRhs1, -row.tangentJacobian1.dot(nextVelocity));
  EXPECT_DOUBLE_EQ(row.tangentRhs2, -row.tangentJacobian2.dot(nextVelocity));

  // Element-exact repeat assembly: the seam is deterministic.
  ASSERT_EQ(repeated.rows.size(), problem.rows.size());
  expectRowsExactlyEqual(problem.rows[0], repeated.rows[0]);
  EXPECT_EQ(problem.inverseMass, repeated.inverseMass);
}

//==============================================================================
TEST(MultibodyLinkContact, UsesRelativeJacobianForSameMultibodyLinkObstacle)
{
  TwoPrismaticLinksWorld scene;

  sx::compute::LinkContact contact;
  contact.link = scene.lower;
  contact.otherLink = scene.upper;
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.point = Eigen::Vector3d::Zero();
  contact.depth = 0.0; // isolate relative velocity from Baumgarte bias
  contact.friction = 0.4;
  contact.restitution = 0.0;

  Eigen::VectorXd nextVelocity(2);
  nextVelocity << -0.5, 0.25;

  const std::vector<sx::compute::LinkContact> contacts{contact};
  const auto problem = sx::compute::assembleMultibodyLinkContactProblem(
      dart::simulation::detail::registryOf(scene.world),
      scene.structure(),
      nextVelocity,
      0.01,
      contacts);

  ASSERT_EQ(problem.rows.size(), 1u);
  ASSERT_EQ(problem.inverseMass.rows(), 2);
  ASSERT_EQ(problem.inverseMass.cols(), 2);

  const auto& row = problem.rows[0];
  EXPECT_TRUE(row.active);
  EXPECT_TRUE(row.otherBody == entt::null);
  EXPECT_DOUBLE_EQ(row.otherInvMass, 0.0);
  EXPECT_TRUE(row.otherInvInertia.isZero(0.0));
  EXPECT_TRUE(row.otherArm.isZero(0.0));

  // The row is the primary link point velocity minus the same-multibody
  // obstacle link point velocity. With two independent +Z prismatic joints
  // that relative normal Jacobian is [1, -1].
  Eigen::Vector2d expectedNormalJacobian;
  expectedNormalJacobian << 1.0, -1.0;
  EXPECT_TRUE(row.normalJacobian.isApprox(expectedNormalJacobian, 1e-12));
  EXPECT_GT(row.normalDenominator, 0.0);
  EXPECT_DOUBLE_EQ(
      row.normalDenominator,
      row.normalJacobian.dot(problem.inverseMass * row.normalJacobian));

  const double approachingVelocity = row.normalJacobian.dot(nextVelocity);
  EXPECT_DOUBLE_EQ(approachingVelocity, -0.75);
  EXPECT_DOUBLE_EQ(row.bias, 0.0);
  EXPECT_DOUBLE_EQ(row.restitutionTarget, 0.0);
  EXPECT_DOUBLE_EQ(row.normalRhs, 0.75);
}

//==============================================================================
TEST(MultibodyLinkContact, CouplesDynamicRigidObstacleRow)
{
  PrismaticLegWorld scene;

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.mass = 1.0;
  obstacleOptions.inertia = Eigen::Matrix3d::Identity();
  obstacleOptions.position = Eigen::Vector3d(0.1, 0.2, -0.3);
  auto obstacle = scene.world.addRigidBody("obstacle", obstacleOptions);

  sx::compute::LinkContact contact;
  contact.link = scene.link;
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.point = Eigen::Vector3d::Zero();
  contact.depth = 0.0; // no penetration: isolate the two-sided coupling
  contact.friction = 0.75;
  contact.restitution = 0.0;
  contact.otherBody
      = dart::simulation::detail::toRegistryEntity(obstacle.getEntity());

  Eigen::VectorXd nextVelocity(1);
  nextVelocity << -0.4;

  const std::vector<sx::compute::LinkContact> contacts{contact};
  const double timeStep = 0.01;

  const auto problem = sx::compute::assembleMultibodyLinkContactProblem(
      dart::simulation::detail::registryOf(scene.world),
      scene.structure(),
      nextVelocity,
      timeStep,
      contacts);
  const auto repeated = sx::compute::assembleMultibodyLinkContactProblem(
      dart::simulation::detail::registryOf(scene.world),
      scene.structure(),
      nextVelocity,
      timeStep,
      contacts);

  ASSERT_EQ(problem.rows.size(), 1u);
  const auto& row = problem.rows[0];
  EXPECT_TRUE(row.active);

  // The dynamic obstacle is coupled in: unit mass and identity inertia, with
  // the arm measured from the obstacle's body origin to the contact point.
  EXPECT_EQ(
      row.otherBody,
      dart::simulation::detail::toRegistryEntity(obstacle.getEntity()));
  EXPECT_DOUBLE_EQ(row.otherInvMass, 1.0);
  EXPECT_TRUE(row.otherInvInertia.isApprox(Eigen::Matrix3d::Identity(), 1e-12));
  EXPECT_TRUE(
      row.otherArm.isApprox(contact.point - obstacleOptions.position, 1e-12));

  EXPECT_DOUBLE_EQ(row.bias, 0.0); // depth == 0
  EXPECT_DOUBLE_EQ(row.friction, 0.75);

  // The denominators include the obstacle compliance and lever-arm terms.
  EXPECT_DOUBLE_EQ(
      row.normalDenominator,
      expectedDenominator(
          row, problem.inverseMass, row.normalJacobian, row.normal));
  EXPECT_DOUBLE_EQ(
      row.tangentDenominator1,
      expectedDenominator(
          row, problem.inverseMass, row.tangentJacobian1, row.tangent1));
  EXPECT_DOUBLE_EQ(
      row.tangentDenominator2,
      expectedDenominator(
          row, problem.inverseMass, row.tangentJacobian2, row.tangent2));
  // The obstacle compliance strictly increases the normal denominator over the
  // link-only term.
  EXPECT_GT(
      row.normalDenominator,
      row.normalJacobian.dot(problem.inverseMass * row.normalJacobian));

  ASSERT_EQ(repeated.rows.size(), 1u);
  expectRowsExactlyEqual(row, repeated.rows[0]);
}

//==============================================================================
TEST(MultibodyLinkContact, TreatsKinematicRigidObstacleAsOneSided)
{
  PrismaticLegWorld scene;

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.mass = 1.0;
  obstacleOptions.inertia = Eigen::Matrix3d::Identity();
  obstacleOptions.position = Eigen::Vector3d(0.1, 0.2, -0.3);
  obstacleOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, 3.0);
  auto obstacle = scene.world.addRigidBody("obstacle", obstacleOptions);
  obstacle.setKinematic(true);

  sx::compute::LinkContact contact;
  contact.link = scene.link;
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.point = Eigen::Vector3d::Zero();
  contact.depth = 0.0;
  contact.friction = 0.75;
  contact.restitution = 0.0;
  contact.otherBody = sx::detail::toRegistryEntity(obstacle.getEntity());

  Eigen::VectorXd nextVelocity(1);
  nextVelocity << -0.4;

  const std::vector<sx::compute::LinkContact> contacts{contact};
  const auto problem = sx::compute::assembleMultibodyLinkContactProblem(
      dart::simulation::detail::registryOf(scene.world),
      scene.structure(),
      nextVelocity,
      0.01,
      contacts);

  ASSERT_EQ(problem.rows.size(), 1u);
  const auto& row = problem.rows[0];
  EXPECT_TRUE(row.active);
  EXPECT_TRUE(row.otherBody == entt::null);
  EXPECT_DOUBLE_EQ(row.otherInvMass, 0.0);
  EXPECT_TRUE(row.otherInvInertia.isZero(0.0));
  EXPECT_TRUE(row.otherArm.isZero(0.0));

  EXPECT_DOUBLE_EQ(
      row.normalDenominator,
      row.normalJacobian.dot(problem.inverseMass * row.normalJacobian));
  EXPECT_DOUBLE_EQ(row.normalRhs, 0.4);
  EXPECT_DOUBLE_EQ(row.tangentRhs1, -row.tangentJacobian1.dot(nextVelocity));
  EXPECT_DOUBLE_EQ(row.tangentRhs2, -row.tangentJacobian2.dot(nextVelocity));
}

//==============================================================================
TEST(MultibodyLinkContact, RejectsWrongVelocityDimension)
{
  PrismaticLegWorld scene;

  sx::compute::LinkContact contact;
  contact.link = scene.link;
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.point = Eigen::Vector3d::Zero();
  contact.depth = 0.01;

  const std::vector<sx::compute::LinkContact> contacts{contact};
  const Eigen::VectorXd wrongSize = Eigen::VectorXd::Zero(2); // DOF is 1

  EXPECT_THROW(
      (void)sx::compute::assembleMultibodyLinkContactProblem(
          dart::simulation::detail::registryOf(scene.world),
          scene.structure(),
          wrongSize,
          0.01,
          contacts),
      sx::InvalidArgumentException);
}
