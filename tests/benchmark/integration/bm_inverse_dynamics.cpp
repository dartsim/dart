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

#include <benchmark/benchmark.h>

#include <memory>
#include <vector>

#include <cmath>

using namespace dart;

namespace {

//==============================================================================
dynamics::BodyNode* addBox(
    const dynamics::SkeletonPtr& skel,
    dynamics::BodyNode* parent,
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
  dynamics::BodyNode* bn = pair.second;

  auto shape = std::make_shared<dynamics::BoxShape>(size);
  bn->createShapeNodeWith<dynamics::VisualAspect>(shape);
  dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  bn->setInertia(inertia);

  return bn;
}

//==============================================================================
dynamics::SkeletonPtr createChain(int numLinks)
{
  auto skel = dynamics::Skeleton::create("chain");

  dynamics::BodyNode* parent = nullptr;
  Eigen::Isometry3d parentToJoint = Eigen::Isometry3d::Identity();
  for (int i = 0; i < numLinks; ++i) {
    Eigen::Isometry3d childToJoint = Eigen::Isometry3d::Identity();
    childToJoint.translation() = Eigen::Vector3d(0.0, 0.0, 0.15);

    parent = addBox(
        skel,
        parent,
        "link" + std::to_string(i),
        Eigen::Vector3d(0.05, 0.05, 0.3),
        1.0,
        parentToJoint,
        childToJoint,
        i % 2 == 0 ? Eigen::Vector3d::UnitY() : Eigen::Vector3d::UnitX());

    parentToJoint = Eigen::Isometry3d::Identity();
    parentToJoint.translation() = Eigen::Vector3d(0.0, 0.0, -0.15);
  }

  return skel;
}

//==============================================================================
struct Biped
{
  dynamics::SkeletonPtr skel;
  dynamics::BodyNode* leftFoot;
  dynamics::BodyNode* rightFoot;
};

Biped createFloatingBiped()
{
  auto skel = dynamics::Skeleton::create("biped");

  dynamics::FreeJoint::Properties rootJoint;
  rootJoint.mName = "root";
  dynamics::BodyNode::Properties pelvisProps;
  pelvisProps.mName = "pelvis";
  auto pair = skel->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr, rootJoint, pelvisProps);
  dynamics::BodyNode* pelvis = pair.second;
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
    dynamics::BodyNode* thigh = addBox(
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
    dynamics::BodyNode* shank = addBox(
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
    dynamics::BodyNode* foot = addBox(
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
std::vector<dynamics::ContactInverseDynamics::Contact> createFootContacts(
    const Biped& biped, std::size_t numContacts, std::size_t numBasis)
{
  std::vector<dynamics::ContactInverseDynamics::Contact> contacts;
  std::size_t added = 0;
  for (const double x : {0.1, -0.1, 0.05, -0.05}) {
    for (dynamics::BodyNode* foot : {biped.leftFoot, biped.rightFoot}) {
      if (added >= numContacts) {
        break;
      }
      dynamics::ContactInverseDynamics::Contact contact;
      contact.bodyNode = foot;
      contact.localOffset = Eigen::Vector3d(x, 0.0, -0.02);
      contact.normal = Eigen::Vector3d::UnitZ();
      contact.frictionCoeff = 0.8;
      contact.numBasis = numBasis;
      contacts.push_back(contact);
      ++added;
    }
  }
  return contacts;
}

//==============================================================================
void setBentPose(const dynamics::SkeletonPtr& skel)
{
  Eigen::VectorXd positions = Eigen::VectorXd::Zero(skel->getNumDofs());
  for (const char* side : {"left", "right"}) {
    positions[skel->getDof(std::string(side) + "_thigh_joint")
                  ->getIndexInSkeleton()]
        = 0.2;
    positions[skel->getDof(std::string(side) + "_shank_joint")
                  ->getIndexInSkeleton()]
        = -0.4;
    positions[skel->getDof(std::string(side) + "_foot_joint")
                  ->getIndexInSkeleton()]
        = 0.2;
  }
  skel->setPositions(positions);
}

} // namespace

//==============================================================================
// Recursive Newton-Euler inverse dynamics on a fixed-base chain. The state is
// perturbed every iteration to defeat the dirty-flag caches.
static void BM_InverseDynamics(benchmark::State& state)
{
  const int numLinks = static_cast<int>(state.range(0));
  auto skel = createChain(numLinks);
  const int numDofs = static_cast<int>(skel->getNumDofs());

  Eigen::VectorXd positions = Eigen::VectorXd::Constant(numDofs, 0.1);
  skel->setVelocities(Eigen::VectorXd::Constant(numDofs, 0.2));
  skel->setAccelerations(Eigen::VectorXd::Constant(numDofs, 0.3));

  double perturbation = 0.0;
  for (auto _ : state) {
    perturbation += 1e-6;
    positions[0] = 0.1 + perturbation;
    skel->setPositions(positions);
    skel->computeInverseDynamics();
    benchmark::DoNotOptimize(skel->getForces());
  }
}
BENCHMARK(BM_InverseDynamics)
    ->Arg(10)
    ->Arg(20)
    ->Arg(40)
    ->Unit(benchmark::kMicrosecond);

//==============================================================================
// Equivalent generalized forces assembled from the dense equation of motion
// (mass matrix plus Coriolis/gravity vector). Comparison baseline that shows
// why the recursive path is preferred for inverse dynamics.
static void BM_InverseDynamicsViaMassMatrix(benchmark::State& state)
{
  const int numLinks = static_cast<int>(state.range(0));
  auto skel = createChain(numLinks);
  const int numDofs = static_cast<int>(skel->getNumDofs());

  Eigen::VectorXd positions = Eigen::VectorXd::Constant(numDofs, 0.1);
  skel->setVelocities(Eigen::VectorXd::Constant(numDofs, 0.2));
  const Eigen::VectorXd accelerations = Eigen::VectorXd::Constant(numDofs, 0.3);
  skel->setAccelerations(accelerations);

  double perturbation = 0.0;
  Eigen::VectorXd forces(numDofs);
  for (auto _ : state) {
    perturbation += 1e-6;
    positions[0] = 0.1 + perturbation;
    skel->setPositions(positions);
    forces.noalias() = skel->getMassMatrix() * accelerations;
    forces += skel->getCoriolisAndGravityForces();
    benchmark::DoNotOptimize(forces);
  }
}
BENCHMARK(BM_InverseDynamicsViaMassMatrix)
    ->Arg(10)
    ->Arg(20)
    ->Arg(40)
    ->Unit(benchmark::kMicrosecond);

//==============================================================================
// Contact-aware inverse dynamics on a 12-DOF floating-base biped while the
// number of contact points grows.
static void BM_ContactInverseDynamics(benchmark::State& state)
{
  const std::size_t numContacts = static_cast<std::size_t>(state.range(0));
  Biped biped = createFloatingBiped();
  setBentPose(biped.skel);
  biped.skel->setVelocities(Eigen::VectorXd::Zero(biped.skel->getNumDofs()));

  dynamics::ContactInverseDynamics solver(biped.skel);
  solver.setContacts(createFootContacts(biped, numContacts, 4));

  // Dirty the positions every iteration (bounded wobble) so transform and
  // Jacobian caches are recomputed per call, like in a real playback loop.
  Eigen::VectorXd positions = biped.skel->getPositions();
  const double basePosition = positions[6];
  Eigen::VectorXd accelerations
      = Eigen::VectorXd::Zero(biped.skel->getNumDofs());
  int iteration = 0;
  for (auto _ : state) {
    ++iteration;
    positions[6] = basePosition + 1e-6 * (iteration % 1000);
    biped.skel->setPositions(positions);
    accelerations[5] = 1e-6 * (iteration % 1000);
    biped.skel->setAccelerations(accelerations);
    auto result = solver.compute();
    benchmark::DoNotOptimize(result.jointForces);
  }
}
BENCHMARK(BM_ContactInverseDynamics)
    ->Arg(2)
    ->Arg(4)
    ->Arg(8)
    ->Unit(benchmark::kMicrosecond);

//==============================================================================
// Contact-aware inverse dynamics while the friction-cone resolution grows
// (four contacts).
static void BM_ContactInverseDynamicsBasis(benchmark::State& state)
{
  const std::size_t numBasis = static_cast<std::size_t>(state.range(0));
  Biped biped = createFloatingBiped();
  setBentPose(biped.skel);
  biped.skel->setVelocities(Eigen::VectorXd::Zero(biped.skel->getNumDofs()));

  dynamics::ContactInverseDynamics solver(biped.skel);
  solver.setContacts(createFootContacts(biped, 4, numBasis));

  // Dirty the positions every iteration (bounded wobble) so transform and
  // Jacobian caches are recomputed per call, like in a real playback loop.
  Eigen::VectorXd positions = biped.skel->getPositions();
  const double basePosition = positions[6];
  Eigen::VectorXd accelerations
      = Eigen::VectorXd::Zero(biped.skel->getNumDofs());
  int iteration = 0;
  for (auto _ : state) {
    ++iteration;
    positions[6] = basePosition + 1e-6 * (iteration % 1000);
    biped.skel->setPositions(positions);
    accelerations[5] = 1e-6 * (iteration % 1000);
    biped.skel->setAccelerations(accelerations);
    auto result = solver.compute();
    benchmark::DoNotOptimize(result.jointForces);
  }
}
BENCHMARK(BM_ContactInverseDynamicsBasis)
    ->Arg(4)
    ->Arg(8)
    ->Arg(16)
    ->Unit(benchmark::kMicrosecond);
