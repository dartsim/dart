/*
 * Copyright (c) 2011-2026, The DART development contributors
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
#include "dart/dynamics/ContactInverseDynamics.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/math/Random.hpp"

#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <memory>
#include <vector>

using namespace dart;
using dynamics::BodyNode;
using dynamics::ContactInverseDynamics;
using dynamics::Skeleton;
using dynamics::SkeletonPtr;

namespace {

const double kGravity = 9.81;

//==============================================================================
BodyNode* addBox(
    const SkeletonPtr& skel,
    BodyNode* parent,
    const std::string& name,
    const Eigen::Vector3d& size,
    double mass,
    const Eigen::Isometry3d& parentToJoint,
    const Eigen::Isometry3d& childToJoint,
    const Eigen::Vector3d& axis)
{
  dynamics::RevoluteJoint::Properties joint;
  joint.mName = name + "_joint";
  joint.mAxis = axis;
  joint.mT_ParentBodyToJoint = parentToJoint;
  joint.mT_ChildBodyToJoint = childToJoint;

  dynamics::BodyNode::Properties body;
  body.mName = name;

  auto pair = skel->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
      parent, joint, body);
  BodyNode* bn = pair.second;

  auto shape = std::make_shared<dynamics::BoxShape>(size);
  bn->createShapeNodeWith<dynamics::VisualAspect>(shape);
  dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  bn->setInertia(inertia);

  return bn;
}

//==============================================================================
// A single floating rigid box: 6 DOFs, no actuated joints.
SkeletonPtr createFloatingBox(
    double mass = 2.0, const Eigen::Vector3d& size = {0.4, 0.4, 0.2})
{
  auto skel = Skeleton::create("floating_box");

  dynamics::FreeJoint::Properties joint;
  joint.mName = "root";
  dynamics::BodyNode::Properties body;
  body.mName = "box";

  auto pair = skel->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr, joint, body);
  BodyNode* bn = pair.second;

  auto shape = std::make_shared<dynamics::BoxShape>(size);
  bn->createShapeNodeWith<dynamics::VisualAspect>(shape);
  dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  bn->setInertia(inertia);

  return skel;
}

//==============================================================================
// Fixed-base three-link arm with revolute joints about the y-axis.
SkeletonPtr createFixedArm()
{
  auto skel = Skeleton::create("fixed_arm");

  BodyNode* parent = nullptr;
  Eigen::Isometry3d parentToJoint = Eigen::Isometry3d::Identity();
  for (int i = 0; i < 3; ++i) {
    Eigen::Isometry3d childToJoint = Eigen::Isometry3d::Identity();
    childToJoint.translation() = Eigen::Vector3d(0.0, 0.0, 0.2);

    parent = addBox(
        skel,
        parent,
        "link" + std::to_string(i),
        Eigen::Vector3d(0.05, 0.05, 0.4),
        1.0,
        parentToJoint,
        childToJoint,
        Eigen::Vector3d::UnitY());

    parentToJoint = Eigen::Isometry3d::Identity();
    parentToJoint.translation() = Eigen::Vector3d(0.0, 0.0, -0.2);
  }

  return skel;
}

//==============================================================================
// Floating-base biped: free-joint pelvis, and per leg a hip/knee/ankle pitch
// chain ending in a foot box. 12 DOFs total. Standing pose: feet flat below
// the pelvis, foot soles at the bottom face of the foot boxes.
struct Biped
{
  SkeletonPtr skel;
  BodyNode* leftFoot;
  BodyNode* rightFoot;
};

Biped createFloatingBiped()
{
  auto skel = Skeleton::create("biped");

  dynamics::FreeJoint::Properties rootJoint;
  rootJoint.mName = "root";
  dynamics::BodyNode::Properties pelvisProps;
  pelvisProps.mName = "pelvis";
  auto pair = skel->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr, rootJoint, pelvisProps);
  BodyNode* pelvis = pair.second;
  {
    auto shape = std::make_shared<dynamics::BoxShape>(
        Eigen::Vector3d(0.25, 0.3, 0.15));
    pelvis->createShapeNodeWith<dynamics::VisualAspect>(shape);
    dynamics::Inertia inertia;
    inertia.setMass(10.0);
    inertia.setMoment(shape->computeInertia(10.0));
    pelvis->setInertia(inertia);
  }

  Biped biped;
  biped.skel = skel;

  for (const double side : {0.1, -0.1}) {
    const std::string prefix = side > 0.0 ? "left" : "right";

    Eigen::Isometry3d hipParentToJoint = Eigen::Isometry3d::Identity();
    hipParentToJoint.translation() = Eigen::Vector3d(0.0, side, -0.075);
    Eigen::Isometry3d hipChildToJoint = Eigen::Isometry3d::Identity();
    hipChildToJoint.translation() = Eigen::Vector3d(0.0, 0.0, 0.15);
    BodyNode* thigh = addBox(
        skel,
        pelvis,
        prefix + "_thigh",
        Eigen::Vector3d(0.06, 0.06, 0.3),
        3.0,
        hipParentToJoint,
        hipChildToJoint,
        Eigen::Vector3d::UnitY());

    Eigen::Isometry3d kneeParentToJoint = Eigen::Isometry3d::Identity();
    kneeParentToJoint.translation() = Eigen::Vector3d(0.0, 0.0, -0.15);
    Eigen::Isometry3d kneeChildToJoint = Eigen::Isometry3d::Identity();
    kneeChildToJoint.translation() = Eigen::Vector3d(0.0, 0.0, 0.15);
    BodyNode* shank = addBox(
        skel,
        thigh,
        prefix + "_shank",
        Eigen::Vector3d(0.05, 0.05, 0.3),
        2.0,
        kneeParentToJoint,
        kneeChildToJoint,
        Eigen::Vector3d::UnitY());

    Eigen::Isometry3d ankleParentToJoint = Eigen::Isometry3d::Identity();
    ankleParentToJoint.translation() = Eigen::Vector3d(0.0, 0.0, -0.15);
    Eigen::Isometry3d ankleChildToJoint = Eigen::Isometry3d::Identity();
    ankleChildToJoint.translation() = Eigen::Vector3d(-0.03, 0.0, 0.04);
    BodyNode* foot = addBox(
        skel,
        shank,
        prefix + "_foot",
        Eigen::Vector3d(0.2, 0.08, 0.04),
        1.0,
        ankleParentToJoint,
        ankleChildToJoint,
        Eigen::Vector3d::UnitY());

    if (side > 0.0) {
      biped.leftFoot = foot;
    } else {
      biped.rightFoot = foot;
    }
  }

  return biped;
}

//==============================================================================
std::vector<ContactInverseDynamics::Contact> createFootContacts(
    const Biped& biped, double frictionCoeff)
{
  std::vector<ContactInverseDynamics::Contact> contacts;
  for (BodyNode* foot : {biped.leftFoot, biped.rightFoot}) {
    for (const double xCorner : {0.1, -0.1}) {
      ContactInverseDynamics::Contact contact;
      contact.bodyNode = foot;
      contact.localOffset = Eigen::Vector3d(xCorner, 0.0, -0.02);
      contact.normal = Eigen::Vector3d::UnitZ();
      contact.frictionCoeff = frictionCoeff;
      contacts.push_back(contact);
    }
  }
  return contacts;
}

//==============================================================================
std::vector<ContactInverseDynamics::Contact> createBoxBottomContacts(
    const SkeletonPtr& box, double frictionCoeff, std::size_t numBasis = 4)
{
  std::vector<ContactInverseDynamics::Contact> contacts;
  for (const double x : {0.2, -0.2}) {
    for (const double y : {0.2, -0.2}) {
      ContactInverseDynamics::Contact contact;
      contact.bodyNode = box->getBodyNode(0);
      contact.localOffset = Eigen::Vector3d(x, y, -0.1);
      contact.normal = Eigen::Vector3d::UnitZ();
      contact.frictionCoeff = frictionCoeff;
      contact.numBasis = numBasis;
      contacts.push_back(contact);
    }
  }
  return contacts;
}

//==============================================================================
Eigen::Vector3d sumForces(const std::vector<Eigen::Vector3d>& forces)
{
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto& force : forces) {
    sum += force;
  }
  return sum;
}

} // namespace

//==============================================================================
TEST(ContactInverseDynamics, NoContactsMatchesComputeInverseDynamics)
{
  auto skel = createFixedArm();
  math::Random::setSeed(11);
  skel->setPositions(math::Random::uniform<Eigen::VectorXd>(3, -1.0, 1.0));
  skel->setVelocities(math::Random::uniform<Eigen::VectorXd>(3, -1.0, 1.0));
  skel->setAccelerations(math::Random::uniform<Eigen::VectorXd>(3, -1.0, 1.0));

  skel->computeInverseDynamics();
  const Eigen::VectorXd expected = skel->getForces();

  ContactInverseDynamics solver(skel);
  const auto result = solver.compute();

  EXPECT_TRUE(result.feasible);
  ASSERT_EQ(result.jointForces.size(), expected.size());
  EXPECT_TRUE(result.jointForces.isApprox(expected, 1e-10));
  EXPECT_TRUE(result.contactForces.empty());
}

//==============================================================================
TEST(ContactInverseDynamics, ComputeDoesNotModifyJointForces)
{
  auto skel = createFixedArm();
  const Eigen::Vector3d originalForces(0.1, 0.2, 0.3);
  skel->setForces(originalForces);

  ContactInverseDynamics solver(skel);
  solver.compute();

  EXPECT_TRUE(skel->getForces().isApprox(originalForces, 1e-12));
}

//==============================================================================
TEST(ContactInverseDynamics, AccessorsAndTuningValidation)
{
  auto box = createFloatingBox();
  ContactInverseDynamics solver(box);

  EXPECT_EQ(solver.getSkeleton(), box);
  EXPECT_DOUBLE_EQ(solver.getRegularization(), 1e-8);
  EXPECT_DOUBLE_EQ(solver.getResidualTolerance(), 1e-6);

  const auto contacts = createBoxBottomContacts(box, 0.6);
  solver.setContacts(contacts);
  ASSERT_EQ(solver.getContacts().size(), contacts.size());
  EXPECT_EQ(solver.getContacts()[0].bodyNode, contacts[0].bodyNode);

  solver.setRegularization(-1.0);
  EXPECT_DOUBLE_EQ(solver.getRegularization(), 1e-8);
  solver.setRegularization(1e-4);
  EXPECT_DOUBLE_EQ(solver.getRegularization(), 1e-4);

  solver.setResidualTolerance(0.0);
  EXPECT_DOUBLE_EQ(solver.getResidualTolerance(), 1e-6);
  solver.setResidualTolerance(1e-5);
  EXPECT_DOUBLE_EQ(solver.getResidualTolerance(), 1e-5);

  EXPECT_TRUE(solver.compute().feasible);
}

//==============================================================================
TEST(ContactInverseDynamics, DetectsFreeRootJointAsUnactuated)
{
  const auto biped = createFloatingBiped();

  ContactInverseDynamics solver(biped.skel);
  const auto unactuated = solver.getUnactuatedDofs();

  ASSERT_EQ(unactuated.size(), 6u);
  for (std::size_t i = 0; i < 6u; ++i) {
    EXPECT_EQ(unactuated[i], i);
  }
}

//==============================================================================
TEST(ContactInverseDynamics, StaticBoxContactForcesBalanceWeight)
{
  const double mass = 2.0;
  auto box = createFloatingBox(mass);

  ContactInverseDynamics solver(box);
  solver.setContacts(createBoxBottomContacts(box, 0.6));
  const auto result = solver.compute();

  ASSERT_TRUE(result.feasible);
  ASSERT_EQ(result.contactForces.size(), 4u);

  // The documented contract bounds the residual by tolerance * |target|_inf.
  const double bound = solver.getResidualTolerance() * mass * kGravity;
  const Eigen::Vector3d total = sumForces(result.contactForces);
  EXPECT_NEAR(total.x(), 0.0, bound);
  EXPECT_NEAR(total.y(), 0.0, bound);
  EXPECT_NEAR(total.z(), mass * kGravity, bound);

  for (const auto& force : result.contactForces) {
    EXPECT_GE(force.z(), -1e-9);
  }

  EXPECT_LE(result.unactuatedResidual.lpNorm<Eigen::Infinity>(), bound);
}

//==============================================================================
TEST(ContactInverseDynamics, FeasibleTangentialAcceleration)
{
  const double mass = 2.0;
  const double frictionCoeff = 0.5;
  auto box = createFloatingBox(mass);

  // Horizontal acceleration below the friction limit: |ax| < mu * g.
  const double ax = 0.3 * kGravity;
  Eigen::VectorXd accelerations = Eigen::VectorXd::Zero(6);
  accelerations[3] = ax; // FreeJoint linear x acceleration
  box->setAccelerations(accelerations);

  ContactInverseDynamics solver(box);
  solver.setContacts(createBoxBottomContacts(box, frictionCoeff));
  const auto result = solver.compute();

  ASSERT_TRUE(result.feasible);
  const double bound = solver.getResidualTolerance() * mass * kGravity;
  const Eigen::Vector3d total = sumForces(result.contactForces);
  EXPECT_NEAR(total.x(), mass * ax, bound);
  EXPECT_NEAR(total.z(), mass * kGravity, bound);

  // Aggregate friction-cone consistency.
  EXPECT_LE(std::hypot(total.x(), total.y()), frictionCoeff * total.z() + 1e-6);
}

//==============================================================================
TEST(ContactInverseDynamics, InfeasibleTangentialAccelerationReported)
{
  const double mass = 2.0;
  const double frictionCoeff = 0.2;
  auto box = createFloatingBox(mass);

  // Horizontal acceleration far above the friction limit.
  Eigen::VectorXd accelerations = Eigen::VectorXd::Zero(6);
  accelerations[3] = 3.0 * frictionCoeff * kGravity;
  box->setAccelerations(accelerations);

  ContactInverseDynamics solver(box);
  solver.setContacts(createBoxBottomContacts(box, frictionCoeff));
  const auto result = solver.compute();

  EXPECT_FALSE(result.feasible);
  EXPECT_GT(result.unactuatedResidual.lpNorm<Eigen::Infinity>(), 1e-2);
}

//==============================================================================
TEST(ContactInverseDynamics, ZeroFrictionForcesAlongNormal)
{
  const double mass = 2.0;
  auto box = createFloatingBox(mass);

  ContactInverseDynamics solver(box);
  solver.setContacts(createBoxBottomContacts(box, 0.0));
  const auto result = solver.compute();

  ASSERT_TRUE(result.feasible);
  for (const auto& force : result.contactForces) {
    EXPECT_NEAR(force.x(), 0.0, 1e-9);
    EXPECT_NEAR(force.y(), 0.0, 1e-9);
    EXPECT_GE(force.z(), -1e-9);
  }
  EXPECT_NEAR(
      sumForces(result.contactForces).z(),
      mass * kGravity,
      solver.getResidualTolerance() * mass * kGravity);
}

//==============================================================================
TEST(ContactInverseDynamics, RoundTripWithForwardDynamics)
{
  const auto biped = createFloatingBiped();
  const auto& skel = biped.skel;

  // Slightly bent standing pose.
  Eigen::VectorXd positions = Eigen::VectorXd::Zero(skel->getNumDofs());
  positions[skel->getDof("left_thigh_joint")->getIndexInSkeleton()] = 0.2;
  positions[skel->getDof("left_shank_joint")->getIndexInSkeleton()] = -0.4;
  positions[skel->getDof("left_foot_joint")->getIndexInSkeleton()] = 0.2;
  positions[skel->getDof("right_thigh_joint")->getIndexInSkeleton()] = 0.2;
  positions[skel->getDof("right_shank_joint")->getIndexInSkeleton()] = -0.4;
  positions[skel->getDof("right_foot_joint")->getIndexInSkeleton()] = 0.2;
  skel->setPositions(positions);
  skel->setVelocities(Eigen::VectorXd::Zero(skel->getNumDofs()));

  const auto contacts = createFootContacts(biped, 0.8);

  std::vector<Eigen::VectorXd> testAccelerations;
  testAccelerations.push_back(Eigen::VectorXd::Zero(skel->getNumDofs()));
  {
    // Vertical pelvis acceleration (squat-like motion).
    Eigen::VectorXd ddq = Eigen::VectorXd::Zero(skel->getNumDofs());
    ddq[5] = -1.0;
    testAccelerations.push_back(ddq);
  }
  {
    // Mild joint-space acceleration.
    math::Random::setSeed(21);
    testAccelerations.push_back(
        math::Random::uniform<Eigen::VectorXd>(skel->getNumDofs(), -0.3, 0.3));
  }

  for (std::size_t i = 0; i < testAccelerations.size(); ++i) {
    skel->setAccelerations(testAccelerations[i]);

    ContactInverseDynamics solver(skel);
    solver.setContacts(contacts);
    const auto result = solver.compute();

    ASSERT_TRUE(result.feasible) << "case " << i;

    // Apply the solution: joint forces plus contact forces as external
    // forces, then verify forward dynamics reproduces the accelerations.
    skel->setForces(result.jointForces);
    for (std::size_t k = 0; k < contacts.size(); ++k) {
      contacts[k].bodyNode->addExtForce(
          result.contactForces[k], contacts[k].localOffset, false, true);
    }

    skel->computeForwardDynamics();
    const Eigen::VectorXd recovered = skel->getAccelerations();

    EXPECT_LT(
        (recovered - testAccelerations[i]).lpNorm<Eigen::Infinity>(), 1e-6)
        << "case " << i << "\nexpected: " << testAccelerations[i].transpose()
        << "\nactual:   " << recovered.transpose();

    skel->clearExternalForces();
    skel->resetGeneralizedForces();
  }
}

//==============================================================================
// Same round trip with a rotated and tilted floating base: FreeJoint
// generalized forces are expressed in exponential coordinates, so this
// catches any frame inconsistency between the inverse dynamics result and
// the contact Jacobians.
TEST(ContactInverseDynamics, RoundTripWithRotatedRoot)
{
  const auto biped = createFloatingBiped();
  const auto& skel = biped.skel;

  Eigen::VectorXd positions = Eigen::VectorXd::Zero(skel->getNumDofs());
  const Eigen::Matrix3d rotation
      = (Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
  positions.head<3>() = dart::math::logMap(rotation);
  positions[skel->getDof("left_thigh_joint")->getIndexInSkeleton()] = 0.2;
  positions[skel->getDof("left_shank_joint")->getIndexInSkeleton()] = -0.4;
  positions[skel->getDof("left_foot_joint")->getIndexInSkeleton()] = 0.2;
  positions[skel->getDof("right_thigh_joint")->getIndexInSkeleton()] = 0.2;
  positions[skel->getDof("right_shank_joint")->getIndexInSkeleton()] = -0.4;
  positions[skel->getDof("right_foot_joint")->getIndexInSkeleton()] = 0.2;
  skel->setPositions(positions);
  skel->setVelocities(Eigen::VectorXd::Zero(skel->getNumDofs()));
  skel->setAccelerations(Eigen::VectorXd::Zero(skel->getNumDofs()));

  const auto contacts = createFootContacts(biped, 0.8);

  ContactInverseDynamics solver(skel);
  solver.setContacts(contacts);
  const auto result = solver.compute();

  ASSERT_TRUE(result.feasible);

  skel->setForces(result.jointForces);
  for (std::size_t k = 0; k < contacts.size(); ++k) {
    contacts[k].bodyNode->addExtForce(
        result.contactForces[k], contacts[k].localOffset, false, true);
  }

  skel->computeForwardDynamics();
  EXPECT_LT(skel->getAccelerations().lpNorm<Eigen::Infinity>(), 1e-6);

  skel->clearExternalForces();
  skel->resetGeneralizedForces();
}

//==============================================================================
TEST(ContactInverseDynamics, CustomUnactuatedDofs)
{
  auto skel = createFixedArm();
  skel->setPositions(Eigen::Vector3d(0.4, -0.3, 0.2));

  // Declare the first joint unactuated and let a contact on the last link
  // balance it. Pick a contact normal whose friction cone can supply torque
  // about the first joint's axis (world y) with the required sign.
  skel->computeInverseDynamics();
  const double requiredTorque = skel->getForces()[0];
  skel->resetGeneralizedForces();

  ContactInverseDynamics solver(skel);
  solver.setUnactuatedDofs({0});

  ContactInverseDynamics::Contact contact;
  contact.bodyNode = skel->getBodyNode("link2");
  contact.localOffset = Eigen::Vector3d(0.0, 0.0, 0.2);
  const Eigen::Vector3d contactPoint
      = contact.bodyNode->getTransform() * contact.localOffset;
  const double torqueSign = requiredTorque >= 0.0 ? 1.0 : -1.0;
  contact.normal
      = torqueSign * Eigen::Vector3d::UnitY().cross(contactPoint).normalized();
  contact.frictionCoeff = 0.5;
  solver.setContacts({contact});

  const auto result = solver.compute();

  ASSERT_EQ(result.jointForces.size(), 3);
  EXPECT_LT(std::abs(result.jointForces[0]), 1e-6);
}

//==============================================================================
TEST(ContactInverseDynamics, RejectsBodyNodeFromOtherSkeleton)
{
  auto box = createFloatingBox();
  auto other = createFloatingBox();

  ContactInverseDynamics solver(box);
  ContactInverseDynamics::Contact contact;
  contact.bodyNode = other->getBodyNode(0);
  solver.setContacts({contact});

  const auto result = solver.compute();
  EXPECT_FALSE(result.feasible);
}

//==============================================================================
TEST(ContactInverseDynamics, RejectsDegenerateContactSpecs)
{
  auto box = createFloatingBox();
  ContactInverseDynamics solver(box);

  {
    ContactInverseDynamics::Contact contact;
    contact.bodyNode = box->getBodyNode(0);
    contact.normal = Eigen::Vector3d::Zero();
    solver.setContacts({contact});
    EXPECT_FALSE(solver.compute().feasible);
  }

  {
    ContactInverseDynamics::Contact contact;
    contact.bodyNode = box->getBodyNode(0);
    contact.frictionCoeff = -0.5;
    solver.setContacts({contact});
    EXPECT_FALSE(solver.compute().feasible);
  }

  {
    ContactInverseDynamics::Contact contact;
    contact.bodyNode = box->getBodyNode(0);
    contact.numBasis = 2;
    solver.setContacts({contact});
    EXPECT_FALSE(solver.compute().feasible);
  }
}

//==============================================================================
TEST(ContactInverseDynamics, RejectsNonFiniteContactSpecs)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  auto box = createFloatingBox();
  ContactInverseDynamics solver(box);

  {
    // A NaN friction coefficient must be rejected cleanly: it would
    // otherwise be counted as frictionless when sizing the system but take
    // the frictional generator path, writing out of bounds.
    ContactInverseDynamics::Contact contact;
    contact.bodyNode = box->getBodyNode(0);
    contact.frictionCoeff = nan;
    solver.setContacts({contact});
    EXPECT_FALSE(solver.compute().feasible);
  }

  {
    ContactInverseDynamics::Contact contact;
    contact.bodyNode = box->getBodyNode(0);
    contact.normal = Eigen::Vector3d(0.0, nan, 1.0);
    solver.setContacts({contact});
    EXPECT_FALSE(solver.compute().feasible);
  }

  {
    ContactInverseDynamics::Contact contact;
    contact.bodyNode = box->getBodyNode(0);
    contact.localOffset = Eigen::Vector3d(nan, 0.0, 0.0);
    solver.setContacts({contact});
    EXPECT_FALSE(solver.compute().feasible);
  }
}

//==============================================================================
TEST(ContactInverseDynamics, RejectsNullSkeleton)
{
  ContactInverseDynamics solver(nullptr);
  EXPECT_EQ(solver.getSkeleton(), nullptr);
  EXPECT_TRUE(solver.getUnactuatedDofs().empty());

  const auto result = solver.compute();
  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.jointForces.size(), 0);
}

//==============================================================================
TEST(ContactInverseDynamics, ReportsInfeasibleForNonFiniteRegularization)
{
  auto box = createFloatingBox();
  ContactInverseDynamics solver(box);
  solver.setRegularization(std::numeric_limits<double>::infinity());
  solver.setContacts(createBoxBottomContacts(box, 0.6));
  const auto result = solver.compute();

  EXPECT_FALSE(result.feasible);
}

//==============================================================================
TEST(ContactInverseDynamics, ComputeDoesNotModifyJointCommands)
{
  auto skel = createFixedArm();
  // Order matters: setForces() overwrites the commands of FORCE-actuated
  // joints, so set the commands afterwards to give them a distinct value.
  skel->setForces(Eigen::Vector3d(0.1, 0.2, 0.3));
  const Eigen::Vector3d commands(0.4, 0.5, 0.6);
  skel->setCommands(commands);

  ContactInverseDynamics solver(skel);
  solver.compute();

  EXPECT_TRUE(skel->getCommands().isApprox(commands, 1e-12));
}

//==============================================================================
TEST(ContactInverseDynamics, MultiTreeSkeletonDetectionAndForceSplit)
{
  // One Skeleton holding two floating boxes (two trees).
  auto skel = Skeleton::create("two_boxes");
  std::array<BodyNode*, 2> boxes;
  const std::array<double, 2> masses = {2.0, 5.0};
  for (int tree = 0; tree < 2; ++tree) {
    dynamics::FreeJoint::Properties joint;
    joint.mName = "root" + std::to_string(tree);
    dynamics::BodyNode::Properties body;
    body.mName = "box" + std::to_string(tree);
    auto pair = skel->createJointAndBodyNodePair<dynamics::FreeJoint>(
        nullptr, joint, body);
    boxes[tree] = pair.second;
    auto shape
        = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.4, 0.4, 0.2));
    boxes[tree]->createShapeNodeWith<dynamics::VisualAspect>(shape);
    dynamics::Inertia inertia;
    inertia.setMass(masses[tree]);
    inertia.setMoment(shape->computeInertia(masses[tree]));
    boxes[tree]->setInertia(inertia);
  }
  Eigen::VectorXd positions = Eigen::VectorXd::Zero(12);
  positions[9] = 1.0; // translate the second tree along x
  skel->setPositions(positions);

  ContactInverseDynamics solver(skel);
  const auto unactuated = solver.getUnactuatedDofs();
  ASSERT_EQ(unactuated.size(), 12u);
  for (std::size_t i = 0; i < 12u; ++i) {
    EXPECT_EQ(unactuated[i], i);
  }

  std::vector<ContactInverseDynamics::Contact> contacts;
  for (int tree = 0; tree < 2; ++tree) {
    for (const double x : {0.2, -0.2}) {
      for (const double y : {0.2, -0.2}) {
        ContactInverseDynamics::Contact contact;
        contact.bodyNode = boxes[tree];
        contact.localOffset = Eigen::Vector3d(x, y, -0.1);
        contact.normal = Eigen::Vector3d::UnitZ();
        contact.frictionCoeff = 0.6;
        contacts.push_back(contact);
      }
    }
  }
  solver.setContacts(contacts);
  const auto result = solver.compute();

  ASSERT_TRUE(result.feasible);
  // Contacts on one tree cannot balance the other tree's root rows, so each
  // tree's contacts carry exactly that tree's weight.
  for (int tree = 0; tree < 2; ++tree) {
    double treeVertical = 0.0;
    for (int k = 0; k < 4; ++k) {
      treeVertical += result.contactForces[4 * tree + k].z();
    }
    EXPECT_NEAR(
        treeVertical,
        masses[tree] * kGravity,
        solver.getResidualTolerance() * masses[1] * kGravity * 10.0);
  }
}

//==============================================================================
TEST(ContactInverseDynamics, FloatingBaseWithoutContactsIsInfeasible)
{
  auto box = createFloatingBox();
  ContactInverseDynamics solver(box);
  const auto result = solver.compute();

  EXPECT_FALSE(result.feasible);
  EXPECT_TRUE(result.contactForces.empty());
  // The residual is the full gravity wrench on the root.
  EXPECT_NEAR(
      result.unactuatedResidual.lpNorm<Eigen::Infinity>(),
      2.0 * kGravity,
      1e-9);
}

//==============================================================================
TEST(ContactInverseDynamics, RejectsOutOfRangeUnactuatedIndex)
{
  auto box = createFloatingBox();
  ContactInverseDynamics solver(box);
  solver.setUnactuatedDofs({99});
  const auto result = solver.compute();
  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.jointForces.size(), 0);
}

//==============================================================================
TEST(ContactInverseDynamics, SolverReuseAcrossContactChanges)
{
  const double mass = 2.0;
  auto box = createFloatingBox(mass);

  ContactInverseDynamics solver(box);

  // 4 contacts, then 1 contact, then 4 again; each result must match a
  // fresh solver (workspace reuse must not leak state).
  const auto fourContacts = createBoxBottomContacts(box, 0.6);
  const std::vector<ContactInverseDynamics::Contact> oneContact
      = {fourContacts[0]};

  for (const auto* contacts : {&fourContacts, &oneContact, &fourContacts}) {
    solver.setContacts(*contacts);
    const auto reused = solver.compute();

    ContactInverseDynamics fresh(box);
    fresh.setContacts(*contacts);
    const auto reference = fresh.compute();

    EXPECT_EQ(reused.feasible, reference.feasible);
    ASSERT_EQ(reused.contactForces.size(), reference.contactForces.size());
    EXPECT_TRUE(reused.jointForces.isApprox(reference.jointForces, 1e-12));
  }
}

//==============================================================================
TEST(ContactInverseDynamics, DampingFlagIsForwarded)
{
  auto skel = createFixedArm();
  for (std::size_t i = 0; i < skel->getNumDofs(); ++i) {
    skel->getDof(i)->setDampingCoefficient(2.5);
  }
  skel->setPositions(Eigen::Vector3d(0.3, -0.2, 0.1));
  skel->setVelocities(Eigen::Vector3d(0.5, -0.4, 0.3));
  skel->setAccelerations(Eigen::Vector3d(0.1, 0.2, -0.3));

  skel->computeInverseDynamics(false, true, false);
  const Eigen::VectorXd expected = skel->getForces();
  skel->resetGeneralizedForces();

  ContactInverseDynamics solver(skel);
  const auto withDamping = solver.compute(false, true, false);
  const auto withoutDamping = solver.compute(false, false, false);

  EXPECT_TRUE(withDamping.jointForces.isApprox(expected, 1e-10));
  EXPECT_FALSE(
      withDamping.jointForces.isApprox(withoutDamping.jointForces, 1e-10));
}

//==============================================================================
TEST(ContactInverseDynamics, MinimumBasisCountWorks)
{
  const double mass = 2.0;
  auto box = createFloatingBox(mass);

  ContactInverseDynamics solver(box);
  solver.setContacts(createBoxBottomContacts(box, 0.6, 3));
  const auto result = solver.compute();

  ASSERT_TRUE(result.feasible);
  EXPECT_NEAR(
      sumForces(result.contactForces).z(),
      mass * kGravity,
      solver.getResidualTolerance() * mass * kGravity);
}

//==============================================================================
TEST(ContactInverseDynamics, NumBasisIgnoredForFrictionlessContacts)
{
  const double mass = 2.0;
  auto box = createFloatingBox(mass);

  // numBasis below the frictional minimum must be accepted when mu == 0.
  ContactInverseDynamics solver(box);
  solver.setContacts(createBoxBottomContacts(box, 0.0, 2));
  const auto result = solver.compute();

  ASSERT_TRUE(result.feasible);
  EXPECT_NEAR(
      sumForces(result.contactForces).z(),
      mass * kGravity,
      solver.getResidualTolerance() * mass * kGravity);
}

//==============================================================================
TEST(ContactInverseDynamics, NormalLengthDoesNotChangeResult)
{
  auto box = createFloatingBox();

  ContactInverseDynamics unitSolver(box);
  unitSolver.setContacts(createBoxBottomContacts(box, 0.6));
  const auto unitResult = unitSolver.compute();

  auto scaledContacts = createBoxBottomContacts(box, 0.6);
  for (auto& contact : scaledContacts) {
    contact.normal *= 2.0;
  }
  ContactInverseDynamics scaledSolver(box);
  scaledSolver.setContacts(scaledContacts);
  const auto scaledResult = scaledSolver.compute();

  ASSERT_TRUE(unitResult.feasible);
  ASSERT_TRUE(scaledResult.feasible);
  for (std::size_t k = 0; k < unitResult.contactForces.size(); ++k) {
    EXPECT_TRUE(unitResult.contactForces[k].isApprox(
        scaledResult.contactForces[k], 1e-9));
  }
}
