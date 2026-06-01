/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

#include <dart/simulation/experimental/body/contact.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/comps/multibody.hpp>
#include <dart/simulation/experimental/compute/multibody_dynamics.hpp>
#include <dart/simulation/experimental/compute/rigid_body_constraint.hpp>
#include <dart/simulation/experimental/compute/unified_constraint.hpp>
#include <dart/simulation/experimental/multibody/joint.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <entt/entt.hpp>
#include <gtest/gtest.h>

#include <span>
#include <vector>

#include <cmath>

namespace {

namespace sx = dart::simulation::experimental;

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
      world.getRegistry(), contacts);
}

} // namespace

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

  sx::compute::LinkContact near;
  near.link = arm.getEntity();
  near.normal = Eigen::Vector3d::UnitY();
  near.point = Eigen::Vector3d(1.0, 0.0, 0.0);
  near.depth = 0.01;
  near.friction = 0.5;

  sx::compute::LinkContact far;
  far.link = arm.getEntity();
  far.normal = Eigen::Vector3d::UnitY();
  far.point = Eigen::Vector3d(2.0, 0.0, 0.0);
  far.depth = 0.0;
  far.friction = 0.5;

  Eigen::VectorXd nextVelocity(1);
  nextVelocity << -0.3;

  const std::vector<sx::compute::LinkContact> linkContacts{near, far};
  auto& registry = world.getRegistry();
  const auto& structure
      = registry.get<sx::comps::MultibodyStructure>(robot.getEntity());
  auto linkProblem = sx::compute::assembleMultibodyLinkContactProblem(
      registry, structure, nextVelocity, 0.01, linkContacts);

  std::vector<sx::compute::UnifiedMultibodyContact> multibodyContacts;
  multibodyContacts.push_back({robot.getEntity(), linkProblem});

  sx::compute::RigidBodyContactProblem emptyRigid; // no rigid contacts
  const auto unified = sx::compute::assembleUnifiedConstraintProblem(
      emptyRigid, multibodyContacts);

  // Two active link contacts -> 6 rows, all link, one block at base 0.
  ASSERT_EQ(unified.delassus.rows(), 6);
  EXPECT_TRUE(unified.rigidConstraints.empty());
  ASSERT_EQ(unified.multibodyBlocks.size(), 1u);
  const auto& block = unified.multibodyBlocks[0];
  EXPECT_EQ(block.multibody, robot.getEntity());
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
  contact.link = leg.getEntity();
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.point = Eigen::Vector3d::Zero();
  contact.depth = 0.01;
  contact.friction = 0.4;

  Eigen::VectorXd nextVelocity(1);
  nextVelocity << -0.5;
  const std::vector<sx::compute::LinkContact> linkContacts{contact};
  auto& registry = linkWorld.getRegistry();
  const auto& structure
      = registry.get<sx::comps::MultibodyStructure>(robot.getEntity());
  auto linkProblem = sx::compute::assembleMultibodyLinkContactProblem(
      registry, structure, nextVelocity, 0.01, linkContacts);

  std::vector<sx::compute::UnifiedMultibodyContact> multibodyContacts;
  multibodyContacts.push_back({robot.getEntity(), linkProblem});
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
  active.link = leg.getEntity();
  active.normal = Eigen::Vector3d::UnitZ(); // along the slide axis -> active
  active.point = Eigen::Vector3d::Zero();
  active.depth = 0.01;
  active.friction = 0.5;

  sx::compute::LinkContact inactive;
  inactive.link = leg.getEntity();
  inactive.normal = Eigen::Vector3d::UnitX(); // orthogonal to slide -> inactive
  inactive.point = Eigen::Vector3d::Zero();
  inactive.depth = 0.01;
  inactive.friction = 0.5;

  Eigen::VectorXd nextVelocity(1);
  nextVelocity << -0.5;
  const std::vector<sx::compute::LinkContact> linkContacts{active, inactive};
  auto& registry = world.getRegistry();
  const auto& structure
      = registry.get<sx::comps::MultibodyStructure>(robot.getEntity());
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
  multibodyContacts.push_back({robot.getEntity(), linkProblem});
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
      world.getRegistry(), rigidContacts);
  ASSERT_EQ(rigid.constraints.size(), 1u);
  ASSERT_EQ(rigid.constraints[0].bodyB, obstacle.getEntity());

  // Link contact: the prismatic leg pushes on the dynamic obstacle R.
  sx::compute::LinkContact legObstacle;
  legObstacle.link = leg.getEntity();
  legObstacle.normal = Eigen::Vector3d::UnitZ();
  legObstacle.point = Eigen::Vector3d(0.05, -0.1, 0.5);
  legObstacle.depth = 0.0;
  legObstacle.friction = 0.5;
  legObstacle.otherBody = obstacle.getEntity();

  Eigen::VectorXd nextVelocity(1);
  nextVelocity << -0.4;
  const std::vector<sx::compute::LinkContact> linkContacts{legObstacle};
  auto& registry = world.getRegistry();
  const auto& structure
      = registry.get<sx::comps::MultibodyStructure>(robot.getEntity());
  auto linkProblem = sx::compute::assembleMultibodyLinkContactProblem(
      registry, structure, nextVelocity, 0.01, linkContacts);
  ASSERT_EQ(linkProblem.rows.size(), 1u);
  ASSERT_TRUE(linkProblem.rows[0].active);

  std::vector<sx::compute::UnifiedMultibodyContact> multibodyContacts;
  multibodyContacts.push_back({robot.getEntity(), linkProblem});
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
