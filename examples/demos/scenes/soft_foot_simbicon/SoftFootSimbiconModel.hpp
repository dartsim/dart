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

#ifndef DART_EXAMPLES_DEMOS_SCENES_SOFT_FOOT_SIMBICON_SOFTFOOTSIMBICONMODEL_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_SOFT_FOOT_SIMBICON_SOFTFOOTSIMBICONMODEL_HPP_

#include <dart/dart.hpp>

#include <memory>
#include <string>

#include <cstddef>
#include <cstdint>

namespace dart_demos {

namespace atlas_simbicon {
class Controller;
} // namespace atlas_simbicon

namespace soft_foot_simbicon_model {

inline constexpr double kTimeStep = 0.001;

/// Duration (steps) of a scheduled pelvis push, matching the AtlasSimbicon
/// scene's own 100-step push window.
inline constexpr int kPushSteps = 100;

/// Default push magnitude (N) the interactive scene's push keys apply.
inline constexpr double kDefaultPushMagnitude = 500.0;

/// Minimum pelvis-above-feet height (m) for the biped to count as still
/// standing. Measured as pelvisY - mean(footY) rather than an absolute pelvis Y
/// because a hard push slides the whole biped sideways while it stays vertical,
/// so absolute pelvis height alone cannot tell "standing but displaced" from
/// "collapsed". A stable stance holds this near ~0.8 m; a toppled biped drops
/// it toward or below zero.
inline constexpr double kUprightGap = 0.40;

/// Which foot geometry the Atlas biped is loaded with.
enum class Feet
{
  Rigid, ///< rigid box feet (atlas_v3_no_head.sdf)
  Soft,  ///< SoftBodyNode feet (atlas_v3_no_head_soft_feet.sdf)
};

/// Noisy tile floor, per the Jain/Liu noisy-floor experiment: 5x5 cm tiles
/// whose heights vary by up to the chosen amplitude. Tile tops are dug *down*
/// from the flat ground plane (spanning [-amplitude, 0] relative to it) so a
/// freshly spawned biped can never start intersecting a raised tile; the paper
/// only constrains the offsets' spread, not their sign.
inline constexpr double kFloorTileSize = 0.05;
/// The amplitude the paper's noisy-floor experiment uses (0-2 cm offsets).
inline constexpr double kFloorPaperAmplitude = 0.02;
/// Patch extent in tiles. X (sagittal) only needs stance drift, but Z must
/// cover the whole slide a large +Z push produces: a 1.2 m square patch let
/// a pushed biped skate off the edge and free-fall, which reads as a topple
/// (or worse, as upright -- see kFellBelowWorldY) and contaminates the
/// measurement with patch size. The patch spans 0.6 m of -Z and 3.0 m of +Z.
inline constexpr int kFloorTilesX = 24;
inline constexpr int kFloorTilesZ = 72;
/// World-Z where the tile patch begins (see kFloorTilesZ).
inline constexpr double kFloorPatchMinZ = -0.6;

/// GUI-free state for the soft-foot SIMBICON biped: the Atlas-on-ground world,
/// its SIMBICON controller, and the handles the free-functions below drive and
/// measure. The controller math hardcodes world-Y as vertical, so the world is
/// built Y-up (see createModel).
struct Model
{
  dart::simulation::WorldPtr world;
  dart::dynamics::SkeletonPtr atlas;
  std::shared_ptr<atlas_simbicon::Controller> controller;
  dart::dynamics::BodyNode* pelvis = nullptr;
  dart::dynamics::BodyNode* leftFoot = nullptr;
  dart::dynamics::BodyNode* rightFoot = nullptr;
  Eigen::Vector3d externalForce = Eigen::Vector3d::Zero();
  int forceDuration = 0;
  Feet feet = Feet::Rigid;
  /// Multiplicative motor-noise level (see setMotorNoise); 0 disables.
  double motorNoise = 0.0;
  /// State of the deterministic noise stream; reset by resetModel().
  std::uint64_t noiseState = 0;
  /// Per-DOF noise factors currently held (see kMotorNoiseHoldSteps).
  Eigen::VectorXd noiseFactors;
  /// Steps until the held factors are redrawn.
  int noiseHoldCounter = 0;
  /// Height amplitude (m) of the noisy tile floor; 0 means the flat ground.
  double floorAmplitude = 0.0;
};

/// Builds the Atlas-on-ground SIMBICON world for the requested foot geometry.
/// A positive floorAmplitude replaces the flat ground with the seeded noisy
/// tile floor of that height amplitude (see kFloorTileSize).
Model createModel(Feet feet, double floorAmplitude = 0.0);

/// Schedules a pelvis push: `force` (N, world frame) applied for `steps` steps.
void applyPush(Model& model, const Eigen::Vector3d& force, int steps);

/// Applies the control the biped needs immediately before World::step(): the
/// pending pelvis push, one SIMBICON controller update, and the push-window
/// countdown. Used by the interactive scene's preStep (which steps the world
/// itself).
void prepareStep(Model& model);

/// Advances the model one step: prepareStep() followed by World::step().
void step(Model& model);

/// World-frame Y (vertical) coordinate of the pelvis.
double pelvisHeightY(const Model& model);

/// Mean world-frame Y of the two feet.
double meanFootHeightY(const Model& model);

/// Whether the biped is still standing: pelvis at least kUprightGap above the
/// feet (see kUprightGap). Robust to the biped sliding sideways under a push.
bool isUpright(const Model& model);

/// Number of contact points on the left or right foot in the last step.
std::size_t footContactCount(const Model& model);

/// Whether all skeleton and soft-point state remains finite.
bool isFinite(const Model& model);

/// The model's full simulation state, laid out as skeleton positions, skeleton
/// velocities, then each soft point mass's positions and velocities.
///
/// Returned as a vector rather than reduced to a scalar so a determinism check
/// can compare states exactly: any summary that folds the components together
/// admits collisions between genuinely different states.
Eigen::VectorXd stateVector(const Model& model);

/// Resets the biped to its starting state: skeleton configuration, soft-foot
/// point-mass state, gait phase, and any pending push.
void resetModel(Model& model);

/// Bounds of the lateral push sweep maxRecoverablePush() searches. Exported so
/// a caller can tell a measured threshold from one that saturated the ceiling.
constexpr double kPushSweepStart = 2000.0;
constexpr double kPushSweepEnd = 24000.0;
constexpr double kPushSweepStep = 2000.0;

/// Ensemble controls for every sweep verdict in this model. A single
/// trajectory is not a measurement here: near the recovery boundary the soft
/// arm's outcome flips with a 10 um initial offset, with the binary hosting
/// the same objects, and with gait phase (measured 2026-08-01 -- the
/// previous single-trajectory sweep reported an isolated 18000 N resonance
/// island as the soft threshold while 6000-16000 N all toppled). Each
/// magnitude is therefore decided by kReplicaCount replicas; replica k
/// starts with its root X translation offset by k * kReplicaOffset
/// (deterministic, physically negligible, decorrelates microscopic chaos)
/// AND settles k * kReplicaPhaseStride extra steps before the push lands
/// (sampling distinct gait phases -- a push the biped only survives at one
/// arrival phase is not "withstood"). A magnitude counts as sustained only
/// when at least kReplicasRequired replicas recover.
inline constexpr int kReplicaCount = 5;
inline constexpr int kReplicasRequired = 4;
inline constexpr double kReplicaOffset = 1e-5;
inline constexpr int kReplicaPhaseStride = 60;

/// Free fall preserves the pelvis-above-feet gap, so a biped that skates
/// off the edge of the world (or through it) could read as upright forever.
/// isUpright() therefore also requires the pelvis above this absolute
/// world-Y (the ground plane tops out at -0.925).
inline constexpr double kFellBelowWorldY = -2.0;

/// Robust push-recovery threshold (N): the largest magnitude such that every
/// swept magnitude up to it clears the replica-ensemble majority above
/// (prefix semantics -- an isolated high-magnitude resonance pocket above a
/// toppling interval is not a threshold), with optional sustained
/// perturbations active from the first step: `motorNoise` (see
/// setMotorNoise) and `floorAmplitude` (noisy tile floor; 0 selects the
/// flat ground). When `fractionLog` is non-null, one
/// "  push=<N>  <recovered>/<replicas>" line per swept magnitude is
/// appended, so a gate's output records the whole response curve rather
/// than a bare threshold.
double robustRecoverablePush(
    Feet feet,
    double motorNoise,
    double floorAmplitude,
    std::string* fractionLog = nullptr);

/// Flat-ground, unperturbed robustRecoverablePush().
double maxRecoverablePush(Feet feet);

/// Enables multiplicative motor noise: after each SIMBICON update, every
/// generalized force is scaled by (1 + level * u) with u drawn uniformly from
/// [-1, 1) by a deterministic, platform-independent generator owned by the
/// model. Proportional actuation error is the standard motor-noise model, and
/// scaling preserves the unactuated root's zero forces, so the noise perturbs
/// exactly the motor torques the controller wrote.
///
/// The per-DOF factors are held for kMotorNoiseHoldSteps before being
/// redrawn. Redrawing every 1 ms step would make the sweep vacuous: zero-mean
/// noise refreshed far above the biped's mechanical bandwidth averages out,
/// and both foot types then shrug off even 100% torque error (measured:
/// the whole 0.1-1.0 sweep saturated). Held factors act on gait timescales,
/// which is both discriminating and the more honest model of real actuation
/// error, whose bias components persist across control cycles.
///
/// resetModel() rewinds the stream and the hold with everything else,
/// keeping noisy runs deterministic and repeatable.
void setMotorNoise(Model& model, double level);

/// Steps each drawn set of per-DOF noise factors is held before redrawing
/// (50 ms at the model timestep, i.e. a 20 Hz refresh -- the scale of the
/// gait's own control cadence, not of the integrator).
inline constexpr int kMotorNoiseHoldSteps = 50;

/// Operating points for the perturbed push-recovery gates. Sustained
/// perturbations alone do not discriminate -- measured, both foot types
/// idle in place through 100% held torque noise and through the whole
/// 0.4-3.2 cm tile-amplitude range, because the in-place gait is a
/// trivially easy task. What the paper claims is robust *control*, so the
/// perturbed gates instead measure each arm's robust push-recovery
/// threshold (robustRecoverablePush) with the perturbation active from the
/// first step: 20% held actuation error, and the paper's own 2 cm noisy
/// floor.
inline constexpr double kMotorNoiseGateLevel = 0.2;
inline constexpr double kFloorGateAmplitude = kFloorPaperAmplitude;

} // namespace soft_foot_simbicon_model
} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_SCENES_SOFT_FOOT_SIMBICON_SOFTFOOTSIMBICONMODEL_HPP_
