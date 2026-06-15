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

// Headless driver for contact-aware inverse dynamics, mirroring boxes_headless:
// it times only the solve loop (no rendering), prints a high-precision checksum
// of the joint forces as a determinism/correctness oracle, and dumps the
// built-in text profiler. Drives a 12-DOF floating-base biped in a bent pose
// with a configurable number of foot contacts / friction-cone basis directions
// (the BM_ContactInverseDynamics scenario), perturbing the state each iteration
// to defeat the dirty-flag caches as a real playback loop would.
//
// Usage: id_headless [numContacts=8] [numBasis=8] [iters=200000]

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/ContactInverseDynamics.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <dart/common/Profile.hpp>

#include <chrono>
#include <string>
#include <vector>

#include <cerrno>
#include <cstdio>
#include <cstdlib>

using namespace dart;

namespace {

constexpr std::size_t kMinConeBasis = 3u;
constexpr std::size_t kMaxFootContacts = 8u;

bool parseSizeArg(const char* name, const char* value, std::size_t& output)
{
  errno = 0;
  char* end = nullptr;
  const long long parsed = std::strtoll(value, &end, 10);
  if (value[0] == '\0' || end == value || *end != '\0' || errno == ERANGE
      || parsed < 0) {
    std::fprintf(
        stderr,
        "id_headless expected non-negative integer %s; got [%s]\n",
        name,
        value);
    return false;
  }

  output = static_cast<std::size_t>(parsed);
  return true;
}

dynamics::BodyNode* addBox(
    const dynamics::SkeletonPtr& skel,
    dynamics::BodyNode* parent,
    const std::string& name,
    const Eigen::Vector3d& size,
    double mass,
    const Eigen::Isometry3d& parentToJoint,
    const Eigen::Isometry3d& childToJoint)
{
  dynamics::RevoluteJoint::Properties joint;
  joint.mName = name + "_joint";
  joint.mAxis = Eigen::Vector3d::UnitY();
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
        hipChildToJoint);

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
        kneeChildToJoint);

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
        ankleChildToJoint);

    if (side > 0.0)
      biped.leftFoot = foot;
    else
      biped.rightFoot = foot;
  }
  return biped;
}

std::vector<dynamics::ContactInverseDynamics::Contact> createFootContacts(
    const Biped& biped, std::size_t numContacts, std::size_t numBasis)
{
  std::vector<dynamics::ContactInverseDynamics::Contact> contacts;
  std::size_t added = 0;
  for (const double x : {0.1, -0.1, 0.05, -0.05}) {
    for (dynamics::BodyNode* foot : {biped.leftFoot, biped.rightFoot}) {
      if (added >= numContacts)
        break;
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

int main(int argc, char** argv)
{
  std::size_t numContacts = 8u;
  std::size_t numBasis = 8u;
  std::size_t iters = 200000u;

  if ((argc > 1 && !parseSizeArg("numContacts", argv[1], numContacts))
      || (argc > 2 && !parseSizeArg("numBasis", argv[2], numBasis))
      || (argc > 3 && !parseSizeArg("iters", argv[3], iters))) {
    return 1;
  }

  if (numContacts > kMaxFootContacts) {
    std::fprintf(
        stderr,
        "id_headless supports at most %zu foot contacts; requested %zu\n",
        kMaxFootContacts,
        numContacts);
    return 1;
  }

  if (numBasis < kMinConeBasis) {
    std::fprintf(
        stderr,
        "id_headless requires numBasis >= %zu; requested %zu\n",
        kMinConeBasis,
        numBasis);
    return 1;
  }

  Biped biped = createFloatingBiped();
  setBentPose(biped.skel);
  biped.skel->setVelocities(Eigen::VectorXd::Zero(biped.skel->getNumDofs()));

  dynamics::ContactInverseDynamics solver(biped.skel);
  solver.setContacts(createFootContacts(biped, numContacts, numBasis));

  Eigen::VectorXd positions = biped.skel->getPositions();
  const double basePosition = positions[6];
  Eigen::VectorXd accelerations
      = Eigen::VectorXd::Zero(biped.skel->getNumDofs());

  std::printf(
      "# id_headless contacts=%zu basis=%zu dofs=%d iters=%zu\n",
      numContacts,
      numBasis,
      static_cast<int>(biped.skel->getNumDofs()),
      iters);

  double checksum = 0.0;
  const auto t0 = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < iters; ++i) {
    positions[6] = basePosition + 1e-6 * static_cast<double>(i % 1000);
    biped.skel->setPositions(positions);
    accelerations[5] = 1e-6 * static_cast<double>(i % 1000);
    biped.skel->setAccelerations(accelerations);
    auto result = solver.compute();
    checksum += result.jointForces.sum();
  }
  const auto t1 = std::chrono::steady_clock::now();
  const double elapsedMs
      = std::chrono::duration<double, std::milli>(t1 - t0).count();

  std::printf("checksum %.17g\n", checksum);
  std::printf(
      "elapsed_ms %.3f  solves_per_s %.1f\n",
      elapsedMs,
      elapsedMs > 0.0 ? (1000.0 * static_cast<double>(iters) / elapsedMs)
                      : 0.0);

  DART_PROFILE_TEXT_DUMP();
  return 0;
}
