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
 *     copyright notice, this list of conditions and the following disclaimer
 *     in the documentation and/or other materials provided with the
 *     distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// GUI-free numerical oracle for the soft-foot SIMBICON biped (Jain/Liu 2011
// "biped push recovery" parity row). The reusable model + the reused
// atlas_simbicon SIMBICON controller stay GUI-free so the paper's soft-vs-rigid
// contact claim can be exercised directly here.
//
// The three gates below reproduce the paper's row:
//  1. The soft-foot biped stays finite and standing under the controller and is
//     bit-for-bit deterministic across runs.
//  2. Soft feet maintain at least as many ground contact points as rigid feet
//     over a settled window (the paper's headline: "maintaining more ground
//     contact points") -- here a large, robust margin.
//  3. Recoverable pelvis push is measured for both foot types. The paper's
//     "withstands larger perturbations" does NOT reproduce here -- soft
//     recovers 4000 N against the matched rigid control's 12000 N -- so this
//     numbers rather than asserting the comparison. See the gate for the
//     stiffness sweep that locates the cause.
//
// A fourth gate covers the scene's Reset action, which has to restore the soft
// feet's independent point-mass state and the gait phase, not just the
// skeleton's generalized coordinates.

#include "examples/demos/scenes/atlas_simbicon/Controller.hpp"
#include "examples/demos/scenes/atlas_simbicon/State.hpp"
#include "examples/demos/scenes/atlas_simbicon/StateMachine.hpp"
#include "examples/demos/scenes/soft_foot_simbicon/SoftFootSimbiconModel.hpp"

#include <dart/common/Macros.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>

#include <cstddef>

namespace sfs = dart_demos::soft_foot_simbicon_model;

namespace {

/// Total mass of `skeleton`, counting soft point masses.
///
/// `BodyNode::getMass()` is not virtual and `Skeleton::getMass()` accumulates
/// through it, so neither sees the point masses a SoftBodyNode carries; only
/// `SoftBodyNode::getMass()` adds them.
double totalMassIncludingPointMasses(
    const dart::dynamics::SkeletonPtr& skeleton)
{
  double total = 0.0;
  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    auto* body = skeleton->getBodyNode(i);
    auto* soft = dynamic_cast<dart::dynamics::SoftBodyNode*>(body);
    total += soft ? soft->getMass() : body->getMass();
  }
  return total;
}

/// Whether `body` reads as colliding the way BodyContactCondition reads it: for
/// a SoftBodyNode the flag lives on the point masses, not on the body.
bool readsAsColliding(dart::dynamics::BodyNode* body)
{
  auto* soft = dynamic_cast<dart::dynamics::SoftBodyNode*>(body);
  if (soft) {
    for (std::size_t i = 0; i < soft->getNumPointMasses(); ++i) {
      if (soft->getPointMass(i)->isColliding())
        return true;
    }
  }
  DART_SUPPRESS_DEPRECATED_BEGIN
  const bool colliding = body->isColliding();
  DART_SUPPRESS_DEPRECATED_END
  return colliding;
}

/// Name of the gait state the controller is currently in. stateVector() covers
/// the physical state only, so the gait phase has to be checked separately.
std::string gaitStateName(const sfs::Model& model)
{
  return model.controller->getCurrentState()->getCurrentState()->getName();
}

constexpr std::size_t kStandSteps = 1000;
constexpr std::size_t kContactWindowStart = 500;

struct StandOutcome
{
  bool finiteThroughout = true;
  bool uprightThroughout = true;
  std::size_t failedStep = 0;
  double minUprightGap = 1e9;
  double finalPelvisY = 0.0;
  Eigen::VectorXd state;
  std::size_t contactSum = 0;
  std::size_t contactMin = 1000000;
  std::size_t contactSamples = 0;
};

StandOutcome runStand(sfs::Feet feet, std::size_t steps)
{
  sfs::Model model = sfs::createModel(feet);
  StandOutcome out;
  for (std::size_t s = 1; s <= steps; ++s) {
    sfs::step(model);
    if (!sfs::isFinite(model)) {
      out.finiteThroughout = false;
      out.failedStep = s;
      return out;
    }
    const double gap = sfs::pelvisHeightY(model) - sfs::meanFootHeightY(model);
    out.minUprightGap = std::min(out.minUprightGap, gap);
    if (!sfs::isUpright(model)) {
      out.uprightThroughout = false;
      if (out.failedStep == 0)
        out.failedStep = s;
    }
    if (s >= kContactWindowStart) {
      const std::size_t c = sfs::footContactCount(model);
      out.contactSum += c;
      out.contactMin = std::min(out.contactMin, c);
      ++out.contactSamples;
    }
  }
  out.finalPelvisY = sfs::pelvisHeightY(model);
  out.state = sfs::stateVector(model);
  return out;
}

double contactAvg(const StandOutcome& o)
{
  return o.contactSamples ? static_cast<double>(o.contactSum) / o.contactSamples
                          : 0.0;
}

} // namespace

//==============================================================================
// Gate 1: the soft-foot biped stays finite + standing under the controller for
// kStandSteps, and two runs are bit-for-bit identical.
TEST(SoftFootSimbiconModelTest, SoftFootBipedStandsAndIsDeterministic)
{
  const StandOutcome first = runStand(sfs::Feet::Soft, kStandSteps);
  ASSERT_TRUE(first.finiteThroughout)
      << "soft biped became non-finite at step " << first.failedStep;
  ASSERT_TRUE(first.uprightThroughout)
      << "soft biped stopped being upright at step " << first.failedStep
      << " (min pelvis-above-feet gap " << first.minUprightGap << " m)";

  const StandOutcome second = runStand(sfs::Feet::Soft, kStandSteps);
  ASSERT_TRUE(second.finiteThroughout);

  // Exact comparison of the whole state -- skeleton positions and velocities
  // plus every soft point mass's positions and velocities. Reducing this to a
  // scalar would let two genuinely different states compare equal.
  ASSERT_EQ(second.state.size(), first.state.size());
  ASSERT_TRUE((second.state.array() == first.state.array()).all())
      << "soft biped run was not deterministic; largest component difference "
      << (second.state - first.state).cwiseAbs().maxCoeff();
  ASSERT_EQ(second.finalPelvisY, first.finalPelvisY);

  std::cout << std::setprecision(6)
            << "soft_foot_simbicon steps=" << kStandSteps
            << " min_upright_gap=" << first.minUprightGap
            << " final_pelvisY=" << first.finalPelvisY
            << " state_components=" << first.state.size()
            << " deterministic=true\n";
}

//==============================================================================
// Gate 1b: Reset restores the complete state, not just the skeleton's
// generalized coordinates. The soft feet carry deformation and momentum in
// independent point masses, so a reset that skipped them would resume from a
// nominal pose with squashed, still-moving feet.
TEST(SoftFootSimbiconModelTest, ResetRestoresTheFullStartingState)
{
  sfs::Model fresh = sfs::createModel(sfs::Feet::Soft);
  const Eigen::VectorXd startState = sfs::stateVector(fresh);
  const std::string startGait = gaitStateName(fresh);

  sfs::Model model = sfs::createModel(sfs::Feet::Soft);
  sfs::applyPush(model, Eigen::Vector3d(0.0, 0.0, 400.0), 100);

  // Stop with the *right* foot in contact. The walking-in-place machine starts
  // in state 1, whose terminal condition is a BodyContactCondition on the right
  // foot, so this is the state in which stale contact bookkeeping does damage:
  // the first controller update after the reset would see a stance foot from
  // before the reset and transition the gait immediately.
  constexpr int kSettleBeforeReset = 200;
  bool stoppedInRightFootContact = false;
  for (int s = 0; s < 600; ++s) {
    sfs::step(model);
    if (s >= kSettleBeforeReset && readsAsColliding(model.rightFoot)) {
      stoppedInRightFootContact = true;
      break;
    }
  }
  ASSERT_TRUE(stoppedInRightFootContact)
      << "never reached a right-foot stance to reset from, so this gate would "
         "not exercise the stale-contact path";

  // The run has to have actually disturbed both the physical state and the
  // gait phase, or the reset below would be trivially satisfied.
  const Eigen::VectorXd disturbed = sfs::stateVector(model);
  const std::string disturbedGait = gaitStateName(model);
  ASSERT_EQ(disturbed.size(), startState.size());
  ASSERT_GT((disturbed - startState).cwiseAbs().maxCoeff(), 1e-6);
  ASSERT_NE(disturbedGait, startGait)
      << "the run never left the initial gait state, so this gate would not "
         "detect a reset that skips the gait phase";

  sfs::resetModel(model);

  const Eigen::VectorXd resetState = sfs::stateVector(model);
  EXPECT_LT((resetState - startState).cwiseAbs().maxCoeff(), 1e-12)
      << "reset did not restore the starting state";
  // The walking machines start at state 1, not state 0, so resetting to
  // "the first state" would land in the wrong half of the gait cycle.
  EXPECT_EQ(gaitStateName(model), startGait)
      << "reset did not restore the initial gait state";
  EXPECT_EQ(model.forceDuration, 0);
  EXPECT_EQ(model.externalForce, Eigen::Vector3d::Zero());

  // Restoring the state is necessary but not sufficient. Contact bookkeeping --
  // the colliding flags, the last collision result, the constraint impulses --
  // survives a state restore, as do the world clock, the island resting
  // snapshots, and the collision detector's incremental broadphase. All of it
  // steers the next step, and prepareStep() runs the controller before the next
  // solve refreshes any of it. Only stepping both models shows the reset is
  // equivalent to a fresh one.
  //
  // The requirement is exact equality, because two fresh runs are already
  // bit-for-bit identical (see the determinism gate above); anything less would
  // mean state was left behind. 300 steps carries this well past the two points
  // where an incomplete reset showed up historically: a gait transition landing
  // one step apart at step 5, and a second at step 65.
  constexpr int kLockstepSteps = 300;
  for (int s = 1; s <= kLockstepSteps; ++s) {
    sfs::step(fresh);
    sfs::step(model);
    const double divergence
        = (sfs::stateVector(fresh) - sfs::stateVector(model))
              .cwiseAbs()
              .maxCoeff();
    ASSERT_EQ(divergence, 0.0)
        << "a reset model diverged from a fresh model at step " << s;
    ASSERT_EQ(gaitStateName(model), gaitStateName(fresh))
        << "gait state diverged at step " << s;
  }

  std::cout << "soft_foot_simbicon reset  disturbed_by="
            << (disturbed - startState).cwiseAbs().maxCoeff() << " gait "
            << startGait << "->" << disturbedGait << "->" << startGait
            << "  residual_after_reset="
            << (resetState - startState).cwiseAbs().maxCoeff()
            << "  lockstep_divergence_over_" << kLockstepSteps << "_steps="
            << (sfs::stateVector(fresh) - sfs::stateVector(model))
                   .cwiseAbs()
                   .maxCoeff()
            << "\n";
}

//==============================================================================
// Gate 1c: the two bipeds are comparable -- same mass, same number of foot
// collision surfaces. A soft foot's point masses are added
// on top of its link mass, so leaving the link at the rigid mass would make the
// soft biped a kilogram heavier -- and extra ground load raises contact counts
// and changes push recovery by itself, which would make gates 2 and 3 unable to
// attribute anything to deformability.
TEST(SoftFootSimbiconModelTest, SoftAndRigidBipedsAreComparable)
{
  sfs::Model rigid = sfs::createModel(sfs::Feet::Rigid);
  sfs::Model soft = sfs::createModel(sfs::Feet::Soft);

  const double rigidMass = totalMassIncludingPointMasses(rigid.atlas);
  const double softMass = totalMassIncludingPointMasses(soft.atlas);
  EXPECT_NEAR(softMass, rigidMass, 1e-9)
      << "the soft biped does not weigh what the rigid one does, so the "
         "contact and push gates cannot isolate deformability";

  // Per foot as well, not just in total.
  for (const auto& pair :
       {std::make_pair(rigid.leftFoot, soft.leftFoot),
        std::make_pair(rigid.rightFoot, soft.rightFoot)}) {
    auto* softFoot = dynamic_cast<dart::dynamics::SoftBodyNode*>(pair.second);
    ASSERT_NE(softFoot, nullptr) << "soft model's foot is not a SoftBodyNode";
    EXPECT_NEAR(softFoot->getMass(), pair.first->getMass(), 1e-9)
        << "soft foot total mass differs from the rigid foot it replaces";
  }

  // And one collision surface per foot on each side. Parsing `<soft_shape>`
  // adds a collidable SoftMeshShape while the link's original rigid STL
  // collision mesh is still there, so without intervention a soft foot presents
  // two overlapping surfaces against the rigid foot's one and could win on
  // contact count by duplication rather than deformation.
  //
  // Counted from the constraint solver's collision group, not from the body's
  // CollisionAspects. Those are what the solver actually collides, and they can
  // disagree: removing an aspect after the skeleton joins the world does not
  // retire the collision object the group already created for it, so an
  // aspect-based count would report a surface gone while it is still live.
  const auto collisionSurfaces = [](const sfs::Model& model,
                                    dart::dynamics::BodyNode* body) {
    const auto group = model.world->getConstraintSolver()->getCollisionGroup();
    std::size_t count = 0;
    std::size_t softSurfaces = 0;
    for (std::size_t i = 0; i < group->getNumShapeFrames(); ++i) {
      const auto* frame = group->getShapeFrame(i);
      const auto* shapeNode = frame ? frame->asShapeNode() : nullptr;
      if (!shapeNode || shapeNode->getBodyNodePtr() != body)
        continue;
      ++count;
      if (std::dynamic_pointer_cast<const dart::dynamics::SoftMeshShape>(
              shapeNode->getShape())) {
        ++softSurfaces;
      }
    }
    return std::make_pair(count, softSurfaces);
  };

  for (const auto& pair :
       {std::make_pair(rigid.leftFoot, soft.leftFoot),
        std::make_pair(rigid.rightFoot, soft.rightFoot)}) {
    const auto rigidSurfaces = collisionSurfaces(rigid, pair.first);
    const auto softSurfaces = collisionSurfaces(soft, pair.second);
    EXPECT_EQ(softSurfaces.first, rigidSurfaces.first)
        << "the soft foot does not present the same number of collision "
           "surfaces as the rigid foot it replaces";
    EXPECT_EQ(softSurfaces.second, softSurfaces.first)
        << "a soft foot's collision surfaces should all be the soft mesh";
    EXPECT_EQ(rigidSurfaces.second, 0u);
  }

  // Both models must run the FCL detector, where MeshShape and SoftMeshShape
  // are both BVH meshes against the ground primitive -- the same narrow phase.
  // The native `dart` detector dispatches the two shape types to different
  // collide functions with different contact placement, so under it the
  // control would no longer isolate deformation and every count below would be
  // representation again.
  for (const sfs::Model* model : {&rigid, &soft}) {
    ASSERT_EQ(
        model->world->getConstraintSolver()->getCollisionDetector()->getType(),
        "fcl")
        << "the rigid-vs-soft comparison is only valid under FCL";
  }

  // And the same rest-pose collision geometry, vertex for vertex. The rigid
  // control collides as a rigid triangle mesh generated by the same helper
  // that generates the soft surface, so its vertices must equal the soft
  // foot's point-mass rest positions -- measured from the point masses
  // themselves, not from a second copy of the SDF numbers -- and its triangle
  // count must match the soft face count. Anything less than the same
  // tessellation lets the contact counts measure collision representation
  // (a box manifold caps at a few corners; a mesh emits per-vertex contacts)
  // instead of compliance.
  const auto soleCollisionSurface
      = [](const sfs::Model& model,
           dart::dynamics::BodyNode* body) -> const dart::dynamics::ShapeNode* {
    const auto group = model.world->getConstraintSolver()->getCollisionGroup();
    for (std::size_t i = 0; i < group->getNumShapeFrames(); ++i) {
      const auto* frame = group->getShapeFrame(i);
      const auto* shapeNode = frame ? frame->asShapeNode() : nullptr;
      if (shapeNode && shapeNode->getBodyNodePtr() == body)
        return shapeNode;
    }
    return nullptr;
  };
  for (const auto& pair :
       {std::make_pair(rigid.leftFoot, soft.leftFoot),
        std::make_pair(rigid.rightFoot, soft.rightFoot)}) {
    auto* softFoot = dynamic_cast<dart::dynamics::SoftBodyNode*>(pair.second);
    ASSERT_NE(softFoot, nullptr);

    const auto* meshNode = soleCollisionSurface(rigid, pair.first);
    ASSERT_NE(meshNode, nullptr);
    const auto mesh
        = std::dynamic_pointer_cast<const dart::dynamics::MeshShape>(
            meshNode->getShape());
    ASSERT_NE(mesh, nullptr)
        << "the rigid control's foot does not collide as a triangle mesh";
    const auto triMesh = mesh->getTriMesh();
    ASSERT_NE(triMesh, nullptr);

    ASSERT_EQ(triMesh->getVertices().size(), softFoot->getNumPointMasses())
        << "rigid control mesh does not have the soft rest tessellation";
    const Eigen::Isometry3d meshTf = meshNode->getRelativeTransform();
    for (std::size_t i = 0; i < softFoot->getNumPointMasses(); ++i) {
      const Eigen::Vector3d vertex = meshTf * triMesh->getVertices()[i];
      const Eigen::Vector3d& rest
          = softFoot->getPointMass(i)->getRestingPosition();
      ASSERT_LT((vertex - rest).cwiseAbs().maxCoeff(), 1e-9)
          << "rigid control mesh vertex " << i
          << " differs from the soft rest position";
    }
    EXPECT_EQ(triMesh->getTriangles().size(), softFoot->getNumFaces())
        << "rigid control mesh triangulation differs from the soft surface";
  }

  std::cout << "soft_foot_simbicon mass  rigid=" << rigidMass
            << " kg  soft=" << softMass
            << " kg  collision_surfaces_per_foot rigid="
            << collisionSurfaces(rigid, rigid.leftFoot).first
            << " soft=" << collisionSurfaces(soft, soft.leftFoot).first << "\n";
}

//==============================================================================
// Gate 2 (paper: soft maintains more ground contact points): soft feet keep at
// least as many foot contacts as rigid feet over the settled window.
TEST(SoftFootSimbiconModelTest, SoftMaintainsAtLeastRigidFootContacts)
{
  const StandOutcome rigid = runStand(sfs::Feet::Rigid, kStandSteps);
  const StandOutcome soft = runStand(sfs::Feet::Soft, kStandSteps);
  ASSERT_TRUE(rigid.finiteThroughout);
  ASSERT_TRUE(soft.finiteThroughout);

  // Both bipeds must still be standing. A toppled rigid biped also reports few
  // foot contacts, so without this the comparison below could "pass" by
  // measuring a fall rather than the contact spreading the paper describes.
  ASSERT_TRUE(rigid.uprightThroughout) << "rigid biped fell; contact totals "
                                          "would not be comparable";
  ASSERT_TRUE(soft.uprightThroughout)
      << "soft biped fell; contact totals would "
         "not be comparable";

  const double rigidAvg = contactAvg(rigid);
  const double softAvg = contactAvg(soft);

  std::cout << std::setprecision(4) << "foot_contacts window=["
            << kContactWindowStart << "," << kStandSteps
            << "]  rigid avg=" << rigidAvg << " min=" << rigid.contactMin
            << "  soft avg=" << softAvg << " min=" << soft.contactMin << "\n";

  EXPECT_GE(soft.contactSum, rigid.contactSum)
      << "soft total foot contacts " << soft.contactSum << " < rigid total "
      << rigid.contactSum;

  // The margin is asserted, not just the ordering, because the demo panel and
  // docs advertise a multiple. Measured with the same-tessellation control:
  // soft 51.98 vs rigid 18.45, a 2.8x spread; the enforced 1.5x leaves
  // headroom for platform jitter while still failing if the central result
  // regresses to a trivial difference.
  constexpr double kAdvertisedContactRatio = 1.5;
  EXPECT_GE(softAvg, kAdvertisedContactRatio * rigidAvg)
      << "the soft feet no longer spread meaningfully more contacts than the "
         "matched rigid control; the demo panel's claim would be stale";
}

//==============================================================================
// Gate 3 (paper: soft withstands larger perturbations): measures the largest
// recoverable lateral pelvis push for each foot type.
//
// This gate records rather than asserts the comparison, because the paper's
// push-recovery advantage does NOT currently reproduce with this asset. With
// the models made genuinely comparable -- equal mass, one collision surface
// per foot, the same rest tessellation -- the soft biped recovers 4000 N
// against the mesh-footed rigid control's 12000 N.
//
// It is a stiffness gap, not a contact-model limitation. Sweeping the soft
// feet's vertex stiffness and damping (`atlas_v3_no_head_soft_feet.sdf` ships
// kv = 5e4, damp = 1e3) gives:
//
//     kv 5e4  damp 1e3 -> 4000 N        kv 5e4  damp 5e3 -> 4000 N
//     kv 2e5  damp 1e3 -> 4000 N        kv 2e5  damp 5e3 -> 4000 N
//     kv 1e6  damp 1e3 -> 6000 N        kv 1e6  damp 5e3 -> 12000 N
//
// so stiffening the feet closes the gap -- kv 1e6 / damp 5e3 matches the
// control's measured 12000 N, though it no longer exceeds it -- and the
// shipped asset's feet are simply too compliant for a 147 kg Atlas. Retuning a
// shared `dart://sample` asset, or overriding its stiffness scene-locally,
// changes what the demo models and is a maintainer decision, so this gate
// publishes the numbers and the contact-spreading gate above carries the
// paper claim that does reproduce.
TEST(SoftFootSimbiconModelTest, MeasuresRecoverablePushForBothFeet)
{
  const double rigidPush = sfs::maxRecoverablePush(sfs::Feet::Rigid);
  const double softPush = sfs::maxRecoverablePush(sfs::Feet::Soft);

  std::cout << "max_recoverable_push  rigid=" << rigidPush
            << " N  soft=" << softPush << " N"
            << (softPush >= rigidPush
                    ? "  (soft >= rigid)"
                    : "  (soft < rigid: paper's push-recovery "
                      "claim does not reproduce at the "
                      "asset's foot stiffness)")
            << "\n";

  // What is asserted is only that the sweep measured a real threshold for each
  // foot type: both recover something and neither saturates the sweep ceiling.
  // A saturated result would not be a measurement.
  ASSERT_GT(rigidPush, 0.0)
      << "rigid biped recovered no push in the sweep -- sweep range too high";
  ASSERT_GT(softPush, 0.0)
      << "soft biped recovered no push in the sweep -- sweep range too high";
  EXPECT_LT(rigidPush, sfs::kPushSweepEnd)
      << "rigid threshold saturated the sweep ceiling, so it is a floor on the "
         "real threshold rather than a measurement";
  EXPECT_LT(softPush, sfs::kPushSweepEnd)
      << "soft threshold saturated the sweep ceiling, so it is a floor on the "
         "real threshold rather than a measurement";
}

//==============================================================================
// Gate 1d: the rigid control and the *undeformed* soft feet must produce the
// same foot-ground contact manifold. This is the empirical form of the
// comparability argument: the control shares the soft feet's tessellation and
// the pinned FCL narrow phase, so at an identical pose, before any deformation
// exists, the two representations must be indistinguishable to collision. If
// this holds, whatever the walking gates then measure between the two models
// is deformation and its dynamics, not collision bookkeeping.
TEST(SoftFootSimbiconModelTest, ControlMatchesUndeformedSoftManifold)
{
  sfs::Model rigid = sfs::createModel(sfs::Feet::Rigid);
  sfs::Model soft = sfs::createModel(sfs::Feet::Soft);

  // Identical configurations with a guaranteed slight ground penetration, and
  // no stepping at all: the soft point masses are still exactly at rest.
  for (sfs::Model* model : {&rigid, &soft}) {
    Eigen::VectorXd positions = model->atlas->getPositions();
    positions[4] -= 0.002; // root translation, world-Y (vertical)
    model->atlas->setPositions(positions);
  }

  const auto footGroundContacts = [](const sfs::Model& model) {
    auto group = model.world->getConstraintSolver()->getCollisionGroup();
    dart::collision::CollisionOption option;
    option.maxNumContacts = 10000u;
    dart::collision::CollisionResult result;
    group->collide(option, &result);

    std::vector<Eigen::Vector3d> points;
    for (const auto& contact : result.getContacts()) {
      const auto* body1 = contact.getBodyNodePtr1().get();
      const auto* body2 = contact.getBodyNodePtr2().get();
      const bool footInvolved
          = body1 == model.leftFoot || body1 == model.rightFoot
            || body2 == model.leftFoot || body2 == model.rightFoot;
      // Only contacts against the ground: raw group->collide() has no
      // adjacent-body filter, and foot-vs-shin pairs may legitimately differ
      // between the two foot representations.
      const bool otherIsGround
          = (body1 && body1->getSkeleton() != model.atlas)
            || (body2 && body2->getSkeleton() != model.atlas);
      if (footInvolved && otherIsGround)
        points.push_back(contact.point);
    }
    std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) {
      return std::lexicographical_compare(
          a.data(), a.data() + 3, b.data(), b.data() + 3);
    });
    return points;
  };

  const auto rigidPoints = footGroundContacts(rigid);
  const auto softPoints = footGroundContacts(soft);

  ASSERT_GT(rigidPoints.size(), 0u)
      << "the penetrating pose produced no rigid foot-ground contacts, so "
         "this gate would be vacuous";
  ASSERT_EQ(rigidPoints.size(), softPoints.size())
      << "the rigid control and the undeformed soft feet produce different "
         "foot-ground manifolds, so the representations are not equivalent to "
         "collision and the walking comparison would not isolate the soft "
         "contact pipeline";

  // Matched as sets, one to one, rather than by sorted index: SoftMeshShape
  // stores its vertices in assimp's single-precision buffers, so the soft
  // manifold sits ~1e-8 m off the control's double-precision points -- far
  // below physical relevance but enough to reorder a lexicographic sort
  // within clusters of nearly-tied coordinates. The tolerance covers float32
  // rounding at these coordinate magnitudes and nothing else; a genuinely
  // different contact (the grid spacing is ~0.04 m) cannot hide under it.
  constexpr double kFloatRoundingTolerance = 1e-6;
  std::vector<bool> matched(softPoints.size(), false);
  double worstPairDistance = 0.0;
  for (std::size_t i = 0; i < rigidPoints.size(); ++i) {
    double best = 1e9;
    std::size_t bestIndex = softPoints.size();
    for (std::size_t j = 0; j < softPoints.size(); ++j) {
      if (matched[j])
        continue;
      const double distance = (rigidPoints[i] - softPoints[j]).norm();
      if (distance < best) {
        best = distance;
        bestIndex = j;
      }
    }
    ASSERT_LT(best, kFloatRoundingTolerance)
        << "control manifold point " << i << " at "
        << rigidPoints[i].transpose()
        << " has no counterpart in the undeformed soft manifold (nearest is "
        << best << " m away)";
    matched[bestIndex] = true;
    worstPairDistance = std::max(worstPairDistance, best);
  }

  std::cout << "soft_foot_simbicon manifold  rigid_points="
            << rigidPoints.size() << "  soft_points=" << softPoints.size()
            << "  worst_pair_distance=" << worstPairDistance
            << " m  (undeformed, identical pose)\n";
}
