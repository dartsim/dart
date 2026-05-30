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

#pragma once

#include <dart/simulation/experimental/fwd.hpp>

#include <dart/simulation/experimental/body/deformable_body_options.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/constraint/loop_closure.hpp>
#include <dart/simulation/experimental/multibody/multibody_options.hpp>
#include <dart/simulation/experimental/world_sync_stage.hpp>

#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <iosfwd>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <cstddef>

namespace dart::simulation::experimental {

/// Solver family used for free rigid-body dynamics in the default experimental
/// World step pipeline.
enum class RigidBodySolver
{
  SequentialImpulse,
  Ipc,
};

class DART_EXPERIMENTAL_API World
{
public:
  World();
  ~World();

  World(const World&) = delete;
  World& operator=(const World&) = delete;
  World(World&&) = delete;
  World& operator=(World&&) = delete;

  //--------------------------------------------------------------------------
  // Frame creation
  //--------------------------------------------------------------------------
  FreeFrame addFreeFrame();
  FreeFrame addFreeFrame(std::string_view name);
  FreeFrame addFreeFrame(std::string_view name, const Frame& parent);

  FixedFrame addFixedFrame(std::string_view name, const Frame& parent);
  FixedFrame addFixedFrame(
      std::string_view name,
      const Frame& parent,
      const Eigen::Isometry3d& offset);

  //--------------------------------------------------------------------------
  // Multibody management
  //--------------------------------------------------------------------------
  Multibody addMultibody(std::string_view name);
  std::optional<Multibody> getMultibody(std::string_view name);
  bool hasMultibody(std::string_view name) const;
  std::size_t getMultibodyCount() const;

  //--------------------------------------------------------------------------
  // Loop-closure management
  //--------------------------------------------------------------------------
  LoopClosure addLoopClosure(
      std::string_view name, const LoopClosureSpec& spec);
  std::optional<LoopClosure> getLoopClosure(std::string_view name);
  bool hasLoopClosure(std::string_view name) const;
  std::size_t getLoopClosureCount() const;

  //--------------------------------------------------------------------------
  // Rigid body management
  //--------------------------------------------------------------------------
  RigidBody addRigidBody(
      std::string_view name,
      const RigidBodyOptions& options = RigidBodyOptions{});
  std::optional<RigidBody> getRigidBody(std::string_view name);
  bool hasRigidBody(std::string_view name) const;
  std::size_t getRigidBodyCount() const;

  //--------------------------------------------------------------------------
  // Deformable body management
  //--------------------------------------------------------------------------
  DeformableBody addDeformableBody(
      std::string_view name, const DeformableBodyOptions& options);
  std::optional<DeformableBody> getDeformableBody(std::string_view name);
  bool hasDeformableBody(std::string_view name) const;
  std::size_t getDeformableBodyCount() const;

  //--------------------------------------------------------------------------
  // Simulation control
  //--------------------------------------------------------------------------
  [[nodiscard]] bool isSimulationMode() const
  {
    return m_simulationMode;
  }

  void enterSimulationMode();

  /// Set the uniform gravitational acceleration applied to dynamic bodies.
  ///
  /// Gravity is applied as an acceleration during rigid-body integration
  /// (`a = force / mass + gravity`); it is not stored in any per-body force
  /// accumulator. Bodies with non-positive or non-finite mass are unaffected.
  /// The value must contain only finite coordinates. Defaults to
  /// `(0, 0, -9.81)`.
  void setGravity(const Eigen::Vector3d& gravity);

  /// Get the uniform gravitational acceleration applied to dynamic bodies.
  [[nodiscard]] const Eigen::Vector3d& getGravity() const noexcept;

  /// Select the solver family used by the default rigid-body step pipeline.
  ///
  /// The default remains SequentialImpulse. Ipc is experimental and currently
  /// handles free mesh-like rigid bodies through the internal rigid IPC stage.
  void setRigidBodySolver(RigidBodySolver solver);

  /// Get the solver family used by the default rigid-body step pipeline.
  [[nodiscard]] RigidBodySolver getRigidBodySolver() const noexcept;

  void setTimeStep(double timeStep);
  [[nodiscard]] double getTimeStep() const noexcept;
  void setTime(double time);
  [[nodiscard]] double getTime() const noexcept;
  [[nodiscard]] std::size_t getFrame() const noexcept;
  void sync(WorldSyncStage stage = WorldSyncStage::Kinematics);
  void sync(WorldSyncStage stage, compute::ComputeExecutor& executor);
  void updateKinematics();
  void updateKinematics(compute::ComputeExecutor& executor);
  void step();
  void step(std::size_t count);
  void step(compute::ComputeExecutor& executor);
  void step(std::size_t count, compute::ComputeExecutor& executor);
  void step(compute::ComputeExecutor& executor, compute::WorldStepStage& stage);
  void step(
      std::size_t count,
      compute::ComputeExecutor& executor,
      compute::WorldStepStage& stage);
  void step(
      compute::ComputeExecutor& executor, compute::WorldStepPipeline& pipeline);
  void step(
      std::size_t count,
      compute::ComputeExecutor& executor,
      compute::WorldStepPipeline& pipeline);

  //--------------------------------------------------------------------------
  // Multibody solver configuration
  //--------------------------------------------------------------------------

  /// Set the multibody solver/integration configuration as a whole (see
  /// `MultibodyOptions`). Configuration is by documented method-family name, so
  /// new capabilities are added as `MultibodyOptions` fields rather than as new
  /// World methods, and no solver/stage types are exposed. Throws
  /// InvalidArgumentException for an unknown `integrationFamily`. Selection is
  /// parsed to an internal representation here, so the per-step path carries no
  /// configuration-parsing cost.
  void setMultibodyOptions(const MultibodyOptions& options);

  /// The current multibody solver/integration configuration. The
  /// `integrationFamily` defaults to `"semi-implicit"`.
  [[nodiscard]] MultibodyOptions getMultibodyOptions() const;

  //--------------------------------------------------------------------------
  // Registry access
  //--------------------------------------------------------------------------
  /// @internal
  /// DART 7 implementation escape hatch for tests and subsystem bring-up.
  /// This is not part of the DART 8 promotion target for the public World
  /// facade; prefer public handles and accessors for user-facing code.
  entt::registry& getRegistry();
  /// @internal
  /// See the non-const overload.
  const entt::registry& getRegistry() const;

  //--------------------------------------------------------------------------
  // Collision queries
  //--------------------------------------------------------------------------

  /// Run a collision query over all rigid bodies that have a collision shape.
  ///
  /// This is a query, not a solver: it reports contact points (position,
  /// world-frame normal from bodyA toward bodyB, and penetration depth) using
  /// the current body world transforms. It does not modify body state. Bodies
  /// without a collision shape are ignored.
  [[nodiscard]] std::vector<Contact> collide();

  //--------------------------------------------------------------------------
  // Serialization
  //--------------------------------------------------------------------------
  void saveBinary(std::ostream& output) const;
  void loadBinary(std::istream& input);

  //--------------------------------------------------------------------------
  // Utilities
  //--------------------------------------------------------------------------
  void clear();

private:
  friend class Frame;
  friend class FreeFrame;
  friend class FixedFrame;
  friend class Link;
  friend class Joint;
  friend class LoopClosure;
  friend class Multibody;
  friend class RigidBody;
  friend class DeformableBody;

  Frame resolveParentFrame(const Frame& parent) const;
  entt::entity createFrameEntity(
      std::string_view name,
      const Frame& parentFrame,
      const Eigen::Isometry3d& localTransform,
      std::size_t* autoNameCounter,
      std::string_view autoNamePrefix,
      bool isFixedFrame,
      std::string& outName);

  void ensureDesignMode() const;
  void resetCountersFromRegistry();

  entt::registry m_registry;
  bool m_simulationMode{false};
  Eigen::Vector3d m_gravity{0.0, 0.0, -9.81};
  RigidBodySolver m_rigidBodySolver{RigidBodySolver::SequentialImpulse};
  double m_timeStep{0.001};
  double m_time{0.0};
  enum class MultibodyIntegrationMethod
  {
    SemiImplicit,
    Variational
  };
  MultibodyIntegrationMethod m_multibodyIntegrationMethod{
      MultibodyIntegrationMethod::SemiImplicit};
  std::size_t m_frame{0};

  std::size_t m_freeFrameCounter{0};
  std::size_t m_fixedFrameCounter{0};
  std::size_t m_multibodyCounter{0};
  std::size_t m_loopClosureCounter{0};
  std::size_t m_rigidBodyCounter{0};
  std::size_t m_deformableBodyCounter{0};
  std::size_t m_linkCounter{0};
  std::size_t m_jointCounter{0};
};

} // namespace dart::simulation::experimental
