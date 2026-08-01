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

#include "SoftFootSimbiconModel.hpp"

#include "../atlas_simbicon/Controller.hpp"
#include "../atlas_simbicon/StateMachine.hpp"

#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include <algorithm>
#include <stdexcept>
#include <string>

namespace dart_demos {
namespace soft_foot_simbicon_model {

namespace {

// Steps to let the biped settle into its walking-in-place gait before a push.
constexpr int kSettleSteps = 500;

// Steps to observe recovery after a push window closes, and the settled tail
// (final steps) over which the biped must stay upright to count as recovered.
constexpr int kRecoverySteps = 800;
constexpr int kSettledTailSteps = 250;

// Lateral (coronal, world +Z) push-magnitude sweep. Lateral perturbations are
// the harder push-recovery axis (narrow coronal base of support) and are the
// perturbation the Jain/Liu row highlights. The range brackets the biped's
// actual topple threshold: both foot types comfortably recover the lower
// magnitudes and topple at the top of the range, so the returned value is a
// real measured threshold, not a saturated sweep ceiling.
const Eigen::Vector3d kPushDirection = Eigen::Vector3d::UnitZ();

//==============================================================================
std::string sdfUriFor(Feet feet)
{
  return feet == Feet::Soft
             ? "dart://sample/sdf/atlas/atlas_v3_no_head_soft_feet.sdf"
             : "dart://sample/sdf/atlas/atlas_v3_no_head.sdf";
}

// Fixed seeds for the deterministic perturbation streams. Both streams go
// through splitmix64 below rather than <random>, because the standard
// distributions are implementation-defined: the same seed draws different
// sequences under libstdc++, libc++, and MSVC, and these draws are part of
// gated, cross-platform measurements.
constexpr std::uint64_t kMotorNoiseSeed = 0x5EED0F00D0C0FFEEull;
constexpr std::uint64_t kFloorSeed = 0x5EED0F100D7113E5ull;

//==============================================================================
std::uint64_t splitmix64(std::uint64_t& state)
{
  state += 0x9E3779B97F4A7C15ull;
  std::uint64_t z = state;
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
  return z ^ (z >> 31);
}

//==============================================================================
/// Next uniform draw in [0, 1), from the top 53 bits (a double's mantissa).
double nextUniform(std::uint64_t& state)
{
  return static_cast<double>(splitmix64(state) >> 11)
         * (1.0 / 9007199254740992.0);
}

//==============================================================================
/// Next uniform draw in [-1, 1).
double nextSymmetricUniform(std::uint64_t& state)
{
  return 2.0 * nextUniform(state) - 1.0;
}

//==============================================================================
/// Settles an already-built model for `settleSteps`, applies a single
/// lateral push of the given magnitude, and reports whether the biped stays
/// finite and upright throughout the settled tail of the recovery window.
/// Requiring uprightness across the whole tail (not just the final instant)
/// rejects a biped that is still toppling or momentarily passes through an
/// upright pose while tumbling.
bool settlesPushesAndRecovers(Model& model, double magnitude, int settleSteps)
{
  for (int i = 0; i < settleSteps; ++i) {
    step(model);
    if (!isFinite(model))
      return false;
  }

  applyPush(model, kPushDirection * magnitude, kPushSteps);

  const int tailStart = kRecoverySteps - kSettledTailSteps;
  for (int i = 0; i < kRecoverySteps; ++i) {
    step(model);
    if (!isFinite(model))
      return false;
    if (i >= tailStart && !isUpright(model))
      return false;
  }

  return true;
}

//==============================================================================
/// One ensemble replica: build the requested configuration, enable the held
/// motor noise, offset the root's X translation by the replica index (see
/// kReplicaOffset), then settle, push, and observe recovery.
bool replicaRecovers(
    Feet feet,
    double motorNoise,
    double floorAmplitude,
    double magnitude,
    int replica)
{
  Model model = createModel(feet, floorAmplitude);
  if (motorNoise > 0.0)
    setMotorNoise(model, motorNoise);
  // Root free joint: dofs 0-2 are rotation, 3-5 translation; dof 3 is world X.
  model.atlas->setPosition(
      3, model.atlas->getPosition(3) + replica * kReplicaOffset);
  // Each replica also settles longer before the push lands, so the ensemble
  // samples distinct gait phases, not five copies of one phase (see the
  // header's ensemble note).
  return settlesPushesAndRecovers(
      model, magnitude, kSettleSteps + replica * kReplicaPhaseStride);
}

//==============================================================================
/// Rest-pose surface of the soft feet, mirrored from the `<soft_shape>`
/// element of `atlas_v3_no_head_soft_feet.sdf`. The rigid control's feet
/// collide as a rigid triangle mesh with exactly this tessellation (see
/// normalizeRigidFoot), so the two models differ only in whether the surface
/// deforms; the comparability gate checks the control's mesh vertex-for-vertex
/// against the soft feet's actual point-mass rest positions rather than
/// trusting a second copy of the numbers.
const Eigen::Vector3d kSoftFootRestBoxSize(0.275, 0.15, 0.075);
const Eigen::Vector3d kSoftFootRestBoxOffset(0.05, 0.0, -0.06);
const Eigen::Vector3i kSoftFootRestBoxFrags(3, 3, 3);
// <soft_shape><total_mass>: the generator distributes it over the point
// masses, and the control's combined-inertia computation needs the same
// distribution the parser produced. The comparability gate cross-checks the
// result against the live soft foot, so a drift from the asset fails there.
constexpr double kSoftFootPointMassTotal = 0.5;

//==============================================================================
/// Makes a loaded soft foot directly comparable with the rigid foot it
/// replaces, by changing the loaded instance rather than the shared
/// `dart://sample` asset, which other callers and `test_SdfParser` also load.
///
/// Two things differ once `<soft_shape>` is parsed, and both would let the
/// comparison measure something other than deformability:
///
///  - The generated `SoftMeshShape` is collidable *in addition to* the link's
///    original rigid STL collision mesh, so a soft foot would present two
///    overlapping collision surfaces against the rigid foot's one, and could
///    win on contact count by duplication alone.
///  - `<total_mass>` is added on top of the link's mass rather than carved out
///    of it, so a soft foot would be 0.5 kg heavier, and extra ground load
///    raises contact counts and changes push recovery by itself.
void normalizeSoftFoot(dart::dynamics::BodyNode* foot)
{
  auto* soft = dynamic_cast<dart::dynamics::SoftBodyNode*>(foot);
  if (!soft)
    throw std::runtime_error(
        std::string(foot->getName()) + ": expected a SoftBodyNode");

  // Leave only the soft surface collidable.
  soft->eachShapeNodeWith<dart::dynamics::CollisionAspect>(
      [](dart::dynamics::ShapeNode* shapeNode) {
        const auto& shape = shapeNode->getShape();
        if (!std::dynamic_pointer_cast<dart::dynamics::SoftMeshShape>(shape))
          shapeNode->removeCollisionAspect();
      });

  // SoftBodyNode::getMass() is link mass plus point masses; BodyNode::getMass()
  // is the link alone. Taking the point-mass total out of the link leaves the
  // foot weighing exactly what the SDF declared for the link, which is what the
  // rigid foot weighs.
  const double linkMass = soft->dart::dynamics::BodyNode::getMass();
  const double pointMass = soft->getMass() - linkMass;
  const double normalizedLinkMass = linkMass - pointMass;
  if (!(normalizedLinkMass > 0.0))
    throw std::runtime_error(
        std::string(foot->getName())
        + ": soft point masses exceed the link mass");

  // The moment of inertia was authored for the full link mass, so scale it with
  // the mass that remains. The point masses contribute their own rotational
  // inertia through their offsets, as separate degrees of freedom.
  dart::dynamics::Inertia inertia = soft->getInertia();
  inertia.setMoment(inertia.getMoment() * (normalizedLinkMass / linkMass));
  inertia.setMass(normalizedLinkMass);
  soft->setInertia(inertia);
}

//==============================================================================
/// Makes the rigid control comparable with the soft feet it is measured
/// against, again on the loaded instance.
///
/// The rigid SDF's feet collide with `l_foot.stl` / `r_foot.stl` while the
/// soft feet collide with the box surface generated from `<soft_shape>`. A
/// triangle mesh and a box produce inherently different contact manifolds, so
/// leaving the STL in the control would let the contact-count comparison
/// measure collision representation rather than compliance. The control's
/// feet collide as the soft rest box instead -- same size, same offset -- and
/// keep the STL for visuals only.
void normalizeRigidFoot(dart::dynamics::BodyNode* foot)
{
  // The friction coefficient rides along from the surface being replaced, so
  // the swap changes geometry and nothing else. The Atlas SDFs declare no
  // custom <surface>, so today this copies the default; the copy is here so
  // that stays true if the asset ever gains one.
  double frictionCoeff = 1.0;
  bool sawCollision = false;
  foot->eachShapeNodeWith<dart::dynamics::CollisionAspect>(
      [&](dart::dynamics::ShapeNode* shapeNode) {
        if (!sawCollision && shapeNode->getDynamicsAspect() != nullptr) {
          frictionCoeff = shapeNode->getDynamicsAspect()->getFrictionCoeff();
          sawCollision = true;
        }
        shapeNode->removeCollisionAspect();
      });
  if (!sawCollision)
    throw std::runtime_error(
        std::string(foot->getName()) + ": no collision surface to replace");

  // The control collides as a rigid triangle mesh with the soft feet's exact
  // rest tessellation, generated by the same helper that generates the soft
  // surface. An analytic BoxShape would not be comparable either: the box
  // manifold is capped at a handful of corner contacts while a mesh emits one
  // contact per touching vertex, so the contact-count comparison would still
  // be measuring the representation. With both feet colliding as the same
  // mesh, one deforming and one not, the counts count the same thing.
  const auto softProperties
      = dart::dynamics::SoftBodyNodeHelper::makeBoxProperties(
          kSoftFootRestBoxSize,
          Eigen::Translation3d(kSoftFootRestBoxOffset)
              * Eigen::Isometry3d::Identity(),
          kSoftFootRestBoxFrags,
          kSoftFootPointMassTotal);

  auto restMesh = std::make_shared<dart::math::TriMesh<double>>();
  restMesh->reserveVertices(softProperties.mPointProps.size());
  for (const auto& pointProperties : softProperties.mPointProps)
    restMesh->addVertex(pointProperties.mX0);
  for (const auto& face : softProperties.mFaces) {
    restMesh->addTriangle(
        static_cast<std::size_t>(face[0]),
        static_cast<std::size_t>(face[1]),
        static_cast<std::size_t>(face[2]));
  }

  auto* meshNode = foot->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::MeshShape>(
          Eigen::Vector3d::Ones(), std::move(restMesh)));
  meshNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  // The control is "the soft foot with deformation frozen", so its rigid-body
  // inertia is the soft foot's combined rest-pose inertia: the down-scaled
  // link (normalizeSoftFoot leaves the soft link at linkMass - pointMass with
  // moments scaled by the same ratio) plus every point mass at its rest
  // position, via the parallel-axis theorem. Without this the control keeps
  // the SDF link's compact tensor while the soft foot carries its mass out at
  // the surface -- measured at 1.8-2.0x the principal moments and a 5.9 mm
  // center-of-mass shift, which changes gait and push response on its own.
  //
  // Matching in this direction is always realizable; the reverse (thinning
  // the soft foot's link to cancel the point masses' spread) would need
  // negative link inertia.
  double pointMassTotal = 0.0;
  for (const auto& pointProperties : softProperties.mPointProps)
    pointMassTotal += pointProperties.mMass;

  const dart::dynamics::Inertia sdfInertia = foot->getInertia();
  const double sdfMass = sdfInertia.getMass();
  const double linkMass = sdfMass - pointMassTotal;
  if (!(linkMass > 0.0))
    throw std::runtime_error(
        std::string(foot->getName())
        + ": soft point masses exceed the link mass");
  const Eigen::Vector3d linkCom = sdfInertia.getLocalCOM();
  const Eigen::Matrix3d linkMomentAboutCom
      = sdfInertia.getMoment() * (linkMass / sdfMass);

  const auto shiftToOrigin
      = [](double mass, const Eigen::Vector3d& com) -> Eigen::Matrix3d {
    return mass
           * (com.squaredNorm() * Eigen::Matrix3d::Identity()
              - com * com.transpose());
  };

  Eigen::Matrix3d momentAboutOrigin
      = linkMomentAboutCom + shiftToOrigin(linkMass, linkCom);
  Eigen::Vector3d weightedCom = linkMass * linkCom;
  for (const auto& pointProperties : softProperties.mPointProps) {
    momentAboutOrigin
        += shiftToOrigin(pointProperties.mMass, pointProperties.mX0);
    weightedCom += pointProperties.mMass * pointProperties.mX0;
  }
  const Eigen::Vector3d combinedCom = weightedCom / sdfMass;
  const Eigen::Matrix3d combinedMomentAboutCom
      = momentAboutOrigin - shiftToOrigin(sdfMass, combinedCom);

  dart::dynamics::Inertia combinedInertia;
  combinedInertia.setMass(sdfMass);
  combinedInertia.setLocalCOM(combinedCom);
  combinedInertia.setMoment(combinedMomentAboutCom);
  foot->setInertia(combinedInertia);
}

//==============================================================================
/// Top of the flat ground plane: ground.urdf's box is centered at y = -0.95
/// with a height of 0.05. The tile floor references this so switching floors
/// never moves the surface the biped spawns above.
constexpr double kGroundTopY = -0.925;
constexpr double kFloorTileThickness = 0.05;

//==============================================================================
/// Builds the seeded noisy tile floor: a single immobile skeleton of
/// kFloorTilesX * kFloorTilesZ welded 5x5 cm box tiles whose tops are dug 0 to
/// `amplitude` meters below the flat ground plane (see the header note on why
/// down rather than up). The heights come from the fixed kFloorSeed stream in
/// row-major tile order, so every build at a given amplitude is identical on
/// every platform.
dart::dynamics::SkeletonPtr buildNoisyTileFloor(double amplitude)
{
  auto floor = dart::dynamics::Skeleton::create("noisy_tile_floor");
  std::uint64_t state = kFloorSeed;

  for (int i = 0; i < kFloorTilesX; ++i) {
    for (int j = 0; j < kFloorTilesZ; ++j) {
      const double drop = amplitude * nextUniform(state);

      dart::dynamics::WeldJoint::Properties jointProperties;
      jointProperties.mName
          = "tile_joint_" + std::to_string(i) + "_" + std::to_string(j);
      dart::dynamics::BodyNode::Properties bodyProperties;
      bodyProperties.mName
          = "tile_" + std::to_string(i) + "_" + std::to_string(j);

      auto pair = floor->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(
          nullptr, jointProperties, bodyProperties);

      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.translation() = Eigen::Vector3d(
          (i - 0.5 * (kFloorTilesX - 1)) * kFloorTileSize,
          kGroundTopY - drop - 0.5 * kFloorTileThickness,
          kFloorPatchMinZ + (j + 0.5) * kFloorTileSize);
      pair.first->setTransformFromParentBodyNode(tf);

      auto* shapeNode = pair.second->createShapeNodeWith<
          dart::dynamics::VisualAspect,
          dart::dynamics::CollisionAspect,
          dart::dynamics::DynamicsAspect>(
          std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(
              kFloorTileSize, kFloorTileThickness, kFloorTileSize)));

      // Checkerboard shading so captures show tiles rather than a flat plane.
      const double shade = ((i + j) % 2 == 0) ? 0.55 : 0.45;
      shapeNode->getVisualAspect()->setColor(
          Eigen::Vector4d(shade, shade, shade + 0.05, 1.0));
    }
  }

  floor->setMobile(false);
  return floor;
}

} // namespace

//==============================================================================
Model createModel(Feet feet, double floorAmplitude)
{
  Model model;
  model.feet = feet;
  model.floorAmplitude = floorAmplitude;
  model.noiseState = kMotorNoiseSeed;

  auto world = dart::simulation::World::create();
  world->setTimeStep(kTimeStep);
  // Single-threaded stepping so two runs of the same model are bit-identical.
  world->setNumSimulationThreads(1);

  // The comparison is only valid under FCL, so the detector is pinned rather
  // than inherited from the solver default. Under FCL both foot types become
  // BVH meshes (MeshShape via createMesh, SoftMeshShape via createSoftMesh)
  // colliding against the ground primitive through the same narrow phase; the
  // native `dart` detector instead dispatches MeshShape and SoftMeshShape to
  // different collide functions with different contact placement, which would
  // reintroduce a representation difference between the control and the soft
  // feet. The comparability gate asserts this configuration.
  world->getConstraintSolver()->setCollisionDetector(
      dart::collision::FCLCollisionDetector::create());

  dart::dynamics::SkeletonPtr ground;
  if (floorAmplitude > 0.0) {
    ground = buildNoisyTileFloor(floorAmplitude);
  } else {
    dart::utils::DartLoader urdfLoader;
    ground = urdfLoader.parseSkeleton("dart://sample/sdf/atlas/ground.urdf");
    if (!ground)
      throw std::runtime_error(
          "failed to load dart://sample/sdf/atlas/ground.urdf");
  }

  auto atlas = dart::utils::SdfParser::readSkeleton(sdfUriFor(feet));
  if (!atlas)
    throw std::runtime_error("failed to load " + sdfUriFor(feet));

  auto* pelvis = atlas->getBodyNode("pelvis");
  auto* leftFoot = atlas->getBodyNode("l_foot");
  auto* rightFoot = atlas->getBodyNode("r_foot");
  if (!pelvis || !leftFoot || !rightFoot)
    throw std::runtime_error(
        sdfUriFor(feet) + ": missing 'pelvis', 'l_foot', or 'r_foot'");

  // Normalize before the skeleton joins the world. ConstraintSolver subscribes
  // its collision group in addSkeleton(), creating collision objects for every
  // surface present at that moment; removing a CollisionAspect afterwards
  // raises the body's shape signal but does not advance the version
  // CollisionGroup::updateSkeletonSource() checks, so the rigid foot meshes
  // would stay live in the group and the comparison would keep running on
  // duplicate surfaces.
  if (feet == Feet::Soft) {
    normalizeSoftFoot(leftFoot);
    normalizeSoftFoot(rightFoot);
  } else {
    normalizeRigidFoot(leftFoot);
    normalizeRigidFoot(rightFoot);
  }

  world->addSkeleton(ground);
  world->addSkeleton(atlas);

  // Y-up, deliberately not reoriented: State::getCOMFrame() and
  // Controller::isAllowingControl() hardcode world-Y as vertical (see the
  // AtlasSimbiconScene file comment). The root free-joint spin stands Atlas up.
  atlas->setPosition(0, -0.5 * dart::math::constantsd::pi());
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  model.world = world;
  model.atlas = atlas;
  model.pelvis = pelvis;
  model.leftFoot = leftFoot;
  model.rightFoot = rightFoot;
  model.controller = std::make_shared<atlas_simbicon::Controller>(
      atlas, world->getConstraintSolver());

  return model;
}

//==============================================================================
void applyPush(Model& model, const Eigen::Vector3d& force, int steps)
{
  model.externalForce = force;
  model.forceDuration = steps;
}

//==============================================================================
void prepareStep(Model& model)
{
  // Order matches AtlasSimbiconScene::preStep: apply the pending pelvis force,
  // then run one controller update.
  //
  // The force is applied only while the window is open. Applying it before
  // testing the counter would push for one step longer than requested, and
  // would turn a zero-step request into a one-step impulse, which would make
  // the recovery sweep's reported push window wrong.
  if (model.forceDuration > 0) {
    model.pelvis->addExtForce(model.externalForce);
    if (--model.forceDuration == 0)
      model.externalForce.setZero();
  }

  model.controller->update();

  // Motor noise perturbs what the controller just wrote, and nothing else:
  // Controller::update() ends in Skeleton::setForces, so scaling the
  // generalized forces here corrupts exactly the motor torques while the
  // scheduled pelvis push above travels separately through addExtForce. The
  // multiplication also keeps the unactuated root's six forces at zero. One
  // draw per degree of freedom per redraw keeps the stream aligned between
  // the rigid and soft arms, whose skeletons expose the same generalized
  // coordinates. The factors are held for kMotorNoiseHoldSteps between
  // redraws (see the header on why per-step redraws would be vacuous).
  if (model.motorNoise > 0.0) {
    Eigen::VectorXd forces = model.atlas->getForces();
    if (model.noiseHoldCounter <= 0
        || model.noiseFactors.size() != forces.size()) {
      model.noiseFactors.resize(forces.size());
      for (Eigen::Index i = 0; i < forces.size(); ++i) {
        model.noiseFactors[i]
            = 1.0 + model.motorNoise * nextSymmetricUniform(model.noiseState);
      }
      model.noiseHoldCounter = kMotorNoiseHoldSteps;
    }
    --model.noiseHoldCounter;
    forces.array() *= model.noiseFactors.array();
    model.atlas->setForces(forces);
  }
}

//==============================================================================
void step(Model& model)
{
  prepareStep(model);
  model.world->step();
}

//==============================================================================
double pelvisHeightY(const Model& model)
{
  return model.pelvis->getTransform().translation().y();
}

//==============================================================================
double meanFootHeightY(const Model& model)
{
  return 0.5
         * (model.leftFoot->getTransform().translation().y()
            + model.rightFoot->getTransform().translation().y());
}

//==============================================================================
bool isUpright(const Model& model)
{
  // The absolute bound catches a biped falling off the world: free fall
  // preserves the pelvis-above-feet gap, so the relative test alone would
  // report a plummeting biped as standing (see kFellBelowWorldY).
  return pelvisHeightY(model) > kFellBelowWorldY
         && (pelvisHeightY(model) - meanFootHeightY(model)) > kUprightGap;
}

//==============================================================================
std::size_t footContactCount(const Model& model)
{
  const auto& result = model.world->getLastCollisionResult();
  std::size_t count = 0;
  for (const auto& contact : result.getContacts()) {
    const auto* body1 = contact.getBodyNodePtr1().get();
    const auto* body2 = contact.getBodyNodePtr2().get();
    if (body1 == model.leftFoot || body1 == model.rightFoot
        || body2 == model.leftFoot || body2 == model.rightFoot)
      ++count;
  }
  return count;
}

//==============================================================================
bool isFinite(const Model& model)
{
  const auto& atlas = model.atlas;
  if (!atlas->getPositions().allFinite() || !atlas->getVelocities().allFinite())
    return false;

  for (std::size_t i = 0; i < atlas->getNumSoftBodyNodes(); ++i) {
    const auto* softBody = atlas->getSoftBodyNode(i);
    if (!softBody->getTransform().matrix().allFinite())
      return false;
    for (std::size_t j = 0; j < softBody->getNumPointMasses(); ++j) {
      const auto* point = softBody->getPointMass(j);
      if (!point->getPositions().allFinite()
          || !point->getVelocities().allFinite()
          || !point->getWorldPosition().allFinite()) {
        return false;
      }
    }
  }
  return true;
}

//==============================================================================
Eigen::VectorXd stateVector(const Model& model)
{
  const Eigen::VectorXd positions = model.atlas->getPositions();
  const Eigen::VectorXd velocities = model.atlas->getVelocities();

  std::size_t numPoints = 0;
  for (std::size_t i = 0; i < model.atlas->getNumSoftBodyNodes(); ++i)
    numPoints += model.atlas->getSoftBodyNode(i)->getNumPointMasses();

  Eigen::VectorXd state(positions.size() + velocities.size() + 6 * numPoints);

  // Assigned by segment rather than with a comma initializer: the vector is
  // longer than these two pieces, and a comma initializer that does not fill
  // the whole vector aborts under Eigen's assertions, which the Debug test
  // configurations enable.
  state.head(positions.size()) = positions;
  state.segment(positions.size(), velocities.size()) = velocities;

  Eigen::Index offset = positions.size() + velocities.size();
  for (std::size_t i = 0; i < model.atlas->getNumSoftBodyNodes(); ++i) {
    const auto* softBody = model.atlas->getSoftBodyNode(i);
    for (std::size_t j = 0; j < softBody->getNumPointMasses(); ++j) {
      const auto* point = softBody->getPointMass(j);
      state.segment<3>(offset) = point->getPositions();
      state.segment<3>(offset + 3) = point->getVelocities();
      offset += 6;
    }
  }
  return state;
}

//==============================================================================
void resetModel(Model& model)
{
  model.controller->resetRobot();

  // resetRobot() restores the skeleton's generalized coordinates only. The
  // soft feet hold their deformation and momentum in independent point-mass
  // state, so without this the biped would resume from a nominal pose with
  // squashed, still-moving feet.
  for (std::size_t i = 0; i < model.atlas->getNumSoftBodyNodes(); ++i) {
    auto* softBody = model.atlas->getSoftBodyNode(i);
    for (std::size_t j = 0; j < softBody->getNumPointMasses(); ++j) {
      auto* point = softBody->getPointMass(j);
      point->resetPositions();
      point->resetVelocities();
      point->resetAccelerations();
      point->resetForces();
    }
  }

  // Contact bookkeeping and world-level state outlive everything above. The
  // colliding flags are per body and per point mass; the constraint impulses,
  // last collision result, clock, frame counter and island resting/deactivation
  // snapshots live on the world. prepareStep() runs the controller before the
  // next solve refreshes any of it, so the initial gait state's
  // BodyContactCondition would fire on a stance foot that no longer exists.
  //
  // The colliding flags are themselves deprecated in DART 6, but the SIMBICON
  // terminal conditions this demo reuses still read them, so the reset has to
  // clear them the same way.
  DART_SUPPRESS_DEPRECATED_BEGIN
  model.atlas->clearCollidingBodies();
  DART_SUPPRESS_DEPRECATED_END
  model.atlas->clearExternalForces();
  model.atlas->clearInternalForces();
  model.world->reset();

  // The collision detector keeps an incremental broadphase built up over every
  // step so far. Two worlds holding the same bodies but different update
  // histories can report the same contacts in a different order, which reorders
  // the LCP and changes the result in the last bits. That was the whole of the
  // residual difference from a fresh model: rebuilding the detector takes a
  // reset from "agrees to 5e-15, then diverges once a gait transition lands one
  // step apart" to bit-for-bit identical.
  auto* solver = model.world->getConstraintSolver();
  solver->setCollisionDetector(
      solver->getCollisionDetector()->cloneWithoutCollisionObjects());

  // After world->reset() the clock reads zero, so the gait restarts from the
  // same phase origin a fresh model does. State 0 is not the initial state of
  // the walking machines -- they start mid-cycle -- so ask the machine.
  auto* stateMachine = model.controller->getCurrentState();
  const double now = model.world->getTime();
  stateMachine->transiteTo(stateMachine->getInitialState(), now);
  stateMachine->begin(now);

  model.externalForce.setZero();
  model.forceDuration = 0;

  // The noise level is configuration and survives a reset, but the stream
  // and the hold rewind with the rest of the state so a restarted noisy run
  // repeats the original draw-for-draw.
  model.noiseState = kMotorNoiseSeed;
  model.noiseHoldCounter = 0;
}

//==============================================================================
double robustRecoverablePush(
    Feet feet,
    double motorNoise,
    double floorAmplitude,
    std::string* fractionLog)
{
  // Prefix semantics: the threshold is the largest magnitude such that every
  // magnitude up to it clears the ensemble majority. A bare max-sustained
  // rule would re-admit isolated high-magnitude resonance pockets (the old
  // single-trajectory sweep's 18000 N island still clears 4/5 replicas while
  // everything from 6000 N up to it topples); "withstands pushes up to X"
  // only means something if the interval below X is survivable. The full
  // response curve is still measured and logged past the break for evidence.
  double threshold = 0.0;
  bool prefixIntact = true;
  for (double magnitude = kPushSweepStart; magnitude <= kPushSweepEnd + 1e-9;
       magnitude += kPushSweepStep) {
    int recovered = 0;
    for (int replica = 0; replica < kReplicaCount; ++replica) {
      if (replicaRecovers(feet, motorNoise, floorAmplitude, magnitude, replica))
        ++recovered;
    }
    if (recovered < kReplicasRequired)
      prefixIntact = false;
    else if (prefixIntact)
      threshold = magnitude;
    if (fractionLog != nullptr) {
      *fractionLog += "  push=" + std::to_string(static_cast<int>(magnitude))
                      + "  " + std::to_string(recovered) + "/"
                      + std::to_string(kReplicaCount) + "\n";
    }
  }
  return threshold;
}

//==============================================================================
double maxRecoverablePush(Feet feet)
{
  return robustRecoverablePush(feet, 0.0, 0.0);
}

//==============================================================================
void setMotorNoise(Model& model, double level)
{
  model.motorNoise = std::max(0.0, level);
}

} // namespace soft_foot_simbicon_model
} // namespace dart_demos
