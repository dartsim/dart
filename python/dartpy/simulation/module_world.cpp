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

#include "simulation/module_detail.hpp"

namespace dart::python_nb {

// World class and backend-neutral deformable acceleration controls
void defSimPartWorld(nb::module_& m)
{
  nb::class_<sim::World>(m, "World")
      .def(
          "__init__",
          [](sim::World* self,
             double timeStep,
             const nb::handle& gravity,
             bool differentiable,
             sim::RigidBodySolver rigidBodySolver,
             const sim::MultibodyOptions& multibodyOptions,
             sim::ContactSolverMethod contactSolverMethod,
             sim::ContactGradientMode contactGradientMode,
             sim::ComputeAcceleratorPolicy computeAcceleratorPolicy,
             const sim::DeactivationOptions& deactivationOptions) {
            sim::WorldOptions options;
            options.timeStep = timeStep;
            if (!gravity.is_none()) {
              options.gravity = toVector3(gravity);
            }
            options.differentiable = differentiable;
            options.rigidBodySolver = rigidBodySolver;
            options.multibodyOptions = multibodyOptions;
            options.contactSolverMethod = contactSolverMethod;
            options.contactGradientMode = contactGradientMode;
            options.computeAcceleratorPolicy = computeAcceleratorPolicy;
            options.deactivationOptions = deactivationOptions;
            new (self) sim::World(options);
          },
          nb::arg("time_step") = 0.001,
          nb::kw_only(),
          nb::arg("gravity") = nb::none(),
          nb::arg("differentiable") = false,
          nb::arg("rigid_body_solver")
          = sim::RigidBodySolver::SequentialImpulse,
          nb::arg("multibody_options") = sim::MultibodyOptions{},
          nb::arg("contact_solver_method")
          = sim::ContactSolverMethod::SequentialImpulse,
          nb::arg("contact_gradient_mode") = sim::ContactGradientMode::Analytic,
          nb::arg("compute_accelerator_policy")
          = sim::ComputeAcceleratorPolicy::CpuOnly,
          nb::arg("deactivation_options") = sim::DeactivationOptions{})
      .def(
          "add_free_frame",
          [](sim::World& self,
             const std::string& name,
             const nb::handle& parent) {
            if (parent.is_none()) {
              return self.addFreeFrame(name);
            }
            return self.addFreeFrame(name, toFrameHandle(parent));
          },
          nb::arg("name") = "",
          nb::kw_only(),
          nb::arg("parent") = nb::none(),
          nb::keep_alive<0, 1>())
      .def(
          "add_fixed_frame",
          [](sim::World& self,
             const std::string& name,
             const nb::handle& parent,
             const nb::handle& offset) {
            if (offset.is_none()) {
              return self.addFixedFrame(name, toFrameHandle(parent));
            }
            return self.addFixedFrame(
                name, toFrameHandle(parent), toIsometry(offset));
          },
          nb::arg("name"),
          nb::arg("parent"),
          nb::arg("offset") = nb::none(),
          nb::keep_alive<0, 1>())
      .def(
          "add_multibody",
          [](sim::World& self, const std::string& name) {
            return self.addMultibody(name);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "get_multibody",
          [](sim::World& self, const std::string& name) -> nb::object {
            auto multibody = self.getMultibody(name);
            if (!multibody.has_value()) {
              return nb::none();
            }
            return nb::cast(*multibody, nb::rv_policy::move);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "has_multibody",
          [](const sim::World& self, const std::string& name) {
            return self.hasMultibody(name);
          },
          nb::arg("name"))
      .def(
          "add_deformable_body",
          [](sim::World& self,
             const std::string& name,
             const sim::DeformableBodyOptions& options) {
            return self.addDeformableBody(name, options);
          },
          nb::arg("name"),
          nb::arg("options"),
          nb::keep_alive<0, 1>())
      .def(
          "get_deformable_body",
          [](sim::World& self, const std::string& name) -> nb::object {
            auto body = self.getDeformableBody(name);
            if (!body.has_value()) {
              return nb::none();
            }
            return nb::cast(*body, nb::rv_policy::move);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "has_deformable_body",
          [](const sim::World& self, const std::string& name) {
            return self.hasDeformableBody(name);
          },
          nb::arg("name"))
      .def_prop_ro(
          "num_deformable_bodies",
          [](const sim::World& self) { return self.getDeformableBodyCount(); })
      .def(
          "configure_deformable_solver",
          [](sim::World& self,
             const std::string& name,
             const sim::DeformableSolverOptions& options) {
            self.configureDeformableSolver(name, options);
          },
          nb::arg("name"),
          nb::arg("options"))
      .def(
          "add_loop_closure",
          [](sim::World& self,
             const sim::LoopClosureSpec& spec,
             const nb::handle& name) {
            return self.addLoopClosure(toOptionalName(name), spec);
          },
          nb::arg("spec"),
          nb::kw_only(),
          nb::arg("name") = nb::none(),
          nb::keep_alive<0, 1>())
      .def(
          "add_loop_closure",
          [](sim::World& self,
             const nb::handle& name,
             const sim::LoopClosureSpec& spec) {
            return self.addLoopClosure(toOptionalName(name), spec);
          },
          nb::arg("name"),
          nb::arg("spec"),
          nb::keep_alive<0, 1>())
      .def(
          "add_loop_closure",
          [](sim::World& self,
             const nb::handle& name,
             const nb::handle& frameA,
             const nb::handle& frameB,
             sim::LoopClosureFamily family,
             const nb::handle& offsetA,
             const nb::handle& offsetB,
             double distance) {
            return self.addLoopClosure(
                toOptionalName(name),
                makeLoopClosureSpec(
                    frameA, frameB, family, offsetA, offsetB, distance));
          },
          nb::arg("name") = nb::none(),
          nb::kw_only(),
          nb::arg("frame_a"),
          nb::arg("frame_b"),
          nb::arg("family") = sim::LoopClosureFamily::Rigid,
          nb::arg("offset_a") = nb::none(),
          nb::arg("offset_b") = nb::none(),
          nb::arg("distance") = 0.0,
          nb::keep_alive<0, 1>())
      .def(
          "has_loop_closure",
          [](const sim::World& self, const std::string& name) {
            return self.hasLoopClosure(name);
          },
          nb::arg("name"))
      .def(
          "get_loop_closure",
          [](sim::World& self, const std::string& name) -> nb::object {
            auto closure = self.getLoopClosure(name);
            if (!closure.has_value()) {
              return nb::none();
            }
            return nb::cast(*closure, nb::rv_policy::move);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "add_joint",
          [](sim::World& self,
             const nb::handle& parent,
             const nb::handle& child,
             const sim::JointSpec& spec) {
            return self.addJoint(
                toFrameHandle(parent), toFrameHandle(child), spec);
          },
          nb::arg("parent"),
          nb::arg("child"),
          nb::arg("spec"),
          nb::keep_alive<0, 1>())
      .def(
          "add_joint",
          [](sim::World& self,
             const nb::handle& child,
             const sim::JointSpec& spec) {
            return self.addJoint(toFrameHandle(child), spec);
          },
          nb::arg("child"),
          nb::arg("spec"),
          nb::keep_alive<0, 1>())
      .def(
          "get_joint",
          [](sim::World& self, const std::string& name) -> nb::object {
            auto joint = self.getJoint(name);
            if (!joint.has_value()) {
              return nb::none();
            }
            return nb::cast(*joint, nb::rv_policy::move);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "has_joint",
          [](const sim::World& self, const std::string& name) {
            return self.hasJoint(name);
          },
          nb::arg("name"))
      .def_prop_ro(
          "joints",
          [](const nb::object& worldObject) {
            auto& self = nb::cast<sim::World&>(worldObject);
            return castJointsKeepingWorldAlive(self.getJoints(), worldObject);
          })
      .def(
          "add_rigid_body",
          [](sim::World& self,
             const std::string& name,
             const sim::RigidBodyOptions& options,
             const nb::handle& mass,
             const nb::handle& position,
             const nb::handle& orientation,
             const nb::handle& linearVelocity,
             const nb::handle& angularVelocity,
             const nb::handle& inertia) {
            return self.addRigidBody(
                name,
                mergeRigidBodyOptions(
                    options,
                    mass,
                    position,
                    orientation,
                    linearVelocity,
                    angularVelocity,
                    inertia));
          },
          nb::arg("name"),
          nb::arg("options") = sim::RigidBodyOptions{},
          nb::kw_only(),
          nb::arg("mass") = nb::none(),
          nb::arg("position") = nb::none(),
          nb::arg("orientation") = nb::none(),
          nb::arg("linear_velocity") = nb::none(),
          nb::arg("angular_velocity") = nb::none(),
          nb::arg("inertia") = nb::none(),
          nb::keep_alive<0, 1>())
      .def(
          "add_rigid_body_distance_spring",
          [](sim::World& self,
             const std::string& name,
             const sim::RigidBody& parent,
             const sim::RigidBody& child,
             double restLength,
             double stiffness) {
            self.addRigidBodyDistanceSpring(
                name, parent, child, restLength, stiffness);
          },
          nb::arg("name"),
          nb::arg("parent"),
          nb::arg("child"),
          nb::arg("rest_length"),
          nb::arg("stiffness"),
          nb::keep_alive<0, 1>())
      .def(
          "add_rigid_body_distance_spring",
          [](sim::World& self,
             const std::string& name,
             const sim::RigidBody& parent,
             const sim::RigidBody& child,
             double restLength,
             double stiffness,
             const nb::handle& parentAnchor,
             const nb::handle& childAnchor) {
            self.addRigidBodyDistanceSpring(
                name,
                parent,
                child,
                restLength,
                stiffness,
                toVector3(parentAnchor),
                toVector3(childAnchor));
          },
          nb::arg("name"),
          nb::arg("parent"),
          nb::arg("child"),
          nb::arg("rest_length"),
          nb::arg("stiffness"),
          nb::kw_only(),
          nb::arg("parent_anchor"),
          nb::arg("child_anchor"),
          nb::keep_alive<0, 1>())
      .def(
          "has_rigid_body_distance_spring",
          [](const sim::World& self, const std::string& name) {
            return self.hasRigidBodyDistanceSpring(name);
          },
          nb::arg("name"))
      .def(
          "get_rigid_body_distance_spring_parameters",
          [](const sim::World& self, const std::string& name) {
            const auto parameters
                = self.getRigidBodyDistanceSpringParameters(name);
            return nb::make_tuple(parameters.first, parameters.second);
          },
          nb::arg("name"))
      .def(
          "set_rigid_body_distance_spring_parameters",
          [](sim::World& self,
             const std::string& name,
             double restLength,
             double stiffness) {
            self.setRigidBodyDistanceSpringParameters(
                name, restLength, stiffness);
          },
          nb::arg("name"),
          nb::arg("rest_length"),
          nb::arg("stiffness"))
      .def(
          "has_rigid_body",
          [](const sim::World& self, const std::string& name) {
            return self.hasRigidBody(name);
          },
          nb::arg("name"))
      .def(
          "get_rigid_body",
          [](sim::World& self, const std::string& name) -> nb::object {
            auto body = self.getRigidBody(name);
            if (!body.has_value()) {
              return nb::none();
            }
            return nb::cast(*body, nb::rv_policy::move);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "get_rigid_body_names",
          [](const sim::World& self) { return self.getRigidBodyNames(); },
          "Sorted names of all rigid bodies currently in the world.")
      .def_prop_ro("is_simulation_mode", &sim::World::isSimulationMode)
      .def("enter_simulation_mode", &sim::World::enterSimulationMode)
      .def(
          "save_binary",
          [](const sim::World& self, const std::filesystem::path& path) {
            std::ofstream output(path, std::ios::binary);
            DART_SIMULATION_THROW_T_IF(
                !output,
                sim::InvalidArgumentException,
                "World.save_binary(path) could not open path for writing");
            self.saveBinary(output);
          },
          nb::arg("path"),
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "load_binary",
          [](sim::World& self, const std::filesystem::path& path) {
            std::ifstream input(path, std::ios::binary);
            DART_SIMULATION_THROW_T_IF(
                !input,
                sim::InvalidArgumentException,
                "World.load_binary(path) could not open path for reading");
            self.loadBinary(input);
          },
          nb::arg("path"),
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "update_kinematics",
          [](sim::World& self) { self.updateKinematics(); },
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "update_kinematics",
          [](sim::World& self, sim::compute::ComputeExecutor& executor) {
            self.updateKinematics(executor);
          },
          nb::arg("executor"),
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "sync",
          [](sim::World& self, sim::WorldSyncStage stage) { self.sync(stage); },
          nb::arg("stage") = sim::WorldSyncStage::Kinematics,
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "sync",
          [](sim::World& self,
             sim::compute::ComputeExecutor& executor,
             sim::WorldSyncStage stage) { self.sync(stage, executor); },
          nb::arg("executor"),
          nb::arg("stage") = sim::WorldSyncStage::Kinematics,
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "step",
          [](sim::World& self, std::ptrdiff_t n) {
            DART_SIMULATION_THROW_T_IF(
                n < 0,
                sim::InvalidArgumentException,
                "World.step(n=...) requires a non-negative step count");
            if (n == 0) {
              return;
            }
            if (!self.isSimulationMode()) {
              self.enterSimulationMode();
            }
            self.step(static_cast<std::size_t>(n));
          },
          nb::arg("n") = 1,
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "step",
          [](sim::World& self,
             sim::compute::ComputeExecutor& executor,
             std::ptrdiff_t n) {
            DART_SIMULATION_THROW_T_IF(
                n < 0,
                sim::InvalidArgumentException,
                "World.step(n=...) requires a non-negative step count");
            if (n == 0) {
              return;
            }
            if (!self.isSimulationMode()) {
              self.enterSimulationMode();
            }
            self.step(static_cast<std::size_t>(n), executor);
          },
          nb::arg("executor"),
          nb::arg("n") = 1,
          nb::call_guard<nb::gil_scoped_release>())
      .def_prop_rw(
          "replay_recording_enabled",
          &sim::World::isReplayRecordingEnabled,
          &sim::World::setReplayRecordingEnabled,
          "Opt-in simulation replay recording. When enabled, the current "
          "state is frame zero and each completed timestep appends another "
          "restorable frame.")
      .def(
          "clear_replay_recording",
          &sim::World::clearReplayRecording,
          "Clear recorded replay frames. If recording is enabled, the current "
          "state is captured as the new frame zero.")
      .def_prop_ro("replay_frame_count", &sim::World::getReplayFrameCount)
      .def_prop_ro("replay_cursor", &sim::World::getReplayCursor)
      .def(
          "get_replay_frame_time",
          &sim::World::getReplayFrameTime,
          nb::arg("index"))
      .def(
          "get_replay_simulation_frame",
          &sim::World::getReplaySimulationFrame,
          nb::arg("index"))
      .def(
          "restore_replay_frame",
          [](sim::World& self, std::ptrdiff_t index) {
            DART_SIMULATION_THROW_T_IF(
                index < 0,
                sim::InvalidArgumentException,
                "World.restore_replay_frame(index) requires a non-negative "
                "index");
            self.restoreReplayFrame(static_cast<std::size_t>(index));
          },
          nb::arg("index"),
          nb::call_guard<nb::gil_scoped_release>())
      .def_prop_ro(
          "last_deformable_solver_diagnostics",
          &sim::World::getLastDeformableSolverDiagnostics,
          nb::rv_policy::reference_internal,
          "Curated diagnostics from the deformable solve on the most recent "
          "step that used the built-in pipeline (mesh sizes, projected-Newton "
          "convergence, self-contact activity, contact closest-approach).")
      .def(
          "get_last_contact_forces",
          [](const sim::World& self) { return self.getLastContactForces(); },
          "Per-contact reaction forces from the most recent built-in-pipeline "
          "step, for the rigid-body sequential-impulse and boxed-LCP paths. "
          "Each entry pairs a world-space contact point with the reaction "
          "force "
          "on body_b (the solved impulse divided by the time step). Empty "
          "after "
          "a reset or a step with no rigid contacts.")
      .def_prop_ro(
          "memory_diagnostics",
          [](const sim::World& self) { return self.getMemoryDiagnostics(); },
          nb::rv_policy::copy,
          "Summary of World-owned allocator and ECS storage diagnostics.")
      .def(
          "get_memory_diagnostics",
          [](const sim::World& self,
             const sim::WorldMemoryDiagnosticsOptions& options) {
            return self.getMemoryDiagnostics(options);
          },
          nb::arg("options"),
          nb::rv_policy::copy,
          "Collect World memory diagnostics with explicit detail options.")
      .def_prop_rw(
          "step_profiling_enabled",
          &sim::World::isStepProfilingEnabled,
          &sim::World::setStepProfilingEnabled,
          "Enable or disable per-stage step profiling. Requires a "
          "DART_BUILD_PROFILE=ON build; profile-off builds report false and "
          "leave last_step_profile empty without storing per-World profiling "
          "cache fields. Off by default; when off the step path is unchanged "
          "and adds no overhead. When on, each step records a per-stage "
          "wall-clock breakdown in last_step_profile.")
      .def_prop_ro(
          "last_step_profile",
          &sim::World::getLastStepProfile,
          nb::rv_policy::reference_internal,
          "Per-stage wall-clock profile of the most recent step taken while "
          "step profiling was enabled (empty otherwise). Call .summary() for a "
          "compact text breakdown of where the step spent its time.")
      .def(
          "compute_step_metrics",
          &sim::World::computeStepMetrics,
          "Read-only physical metrics for the World's current state.")
      .def_prop_rw(
          "time_step", &sim::World::getTimeStep, &sim::World::setTimeStep)
      .def_prop_rw("time", &sim::World::getTime, &sim::World::setTime)
      .def_prop_rw(
          "gravity",
          &sim::World::getGravity,
          [](sim::World& self, const nb::handle& gravity) {
            self.setGravity(toVector3(gravity));
          })
      .def_prop_rw(
          "rigid_body_solver",
          &sim::World::getRigidBodySolver,
          &sim::World::setRigidBodySolver)
      .def_prop_rw(
          "multibody_options",
          &sim::World::getMultibodyOptions,
          [](sim::World& self, const sim::MultibodyOptions& options) {
            self.setMultibodyOptions(options);
          })
      .def_prop_ro("frame", &sim::World::getFrame)
      .def_prop_ro("num_multibodies", &sim::World::getMultibodyCount)
      .def_prop_ro("num_loop_closures", &sim::World::getLoopClosureCount)
      .def_prop_ro("num_rigid_bodies", &sim::World::getRigidBodyCount)
      .def_prop_ro("num_joints", &sim::World::getJointCount)
      .def_prop_ro("is_differentiable", &sim::World::isDifferentiable)
      .def_prop_rw(
          "contact_solver_method",
          &sim::World::getContactSolverMethod,
          &sim::World::setContactSolverMethod)
      .def_prop_rw(
          "contact_gradient_mode",
          &sim::World::getContactGradientMode,
          &sim::World::setContactGradientMode)
      .def_prop_rw(
          "compute_accelerator_policy",
          &sim::World::getComputeAcceleratorPolicy,
          &sim::World::setComputeAcceleratorPolicy)
      .def_prop_rw(
          "deactivation_options",
          &sim::World::getDeactivationOptions,
          [](sim::World& self, const sim::DeactivationOptions& options) {
            self.setDeactivationOptions(options);
          })
      .def_prop_ro("deactivation_enabled", &sim::World::isDeactivationEnabled)
      .def_prop_ro("num_dofs", &sim::World::getNumDofs)
      .def_prop_ro("num_efforts", &sim::World::getNumEfforts)
      .def_prop_ro("num_rigid_body_dofs", &sim::World::getNumRigidBodyDofs)
      .def_prop_ro(
          "num_rigid_body_efforts", &sim::World::getNumRigidBodyEfforts)
      .def_prop_rw(
          "state_vector",
          &sim::World::getStateVector,
          [](sim::World& self, const nb::handle& state) {
            self.setStateVector(toVectorX(state));
          })
      .def_prop_rw(
          "control_vector",
          &sim::World::getControlVector,
          [](sim::World& self, const nb::handle& control) {
            self.setControlVector(toVectorX(control));
          })
      .def_prop_rw(
          "rigid_body_state_vector",
          &sim::World::getRigidBodyStateVector,
          [](sim::World& self, const nb::handle& state) {
            self.setRigidBodyStateVector(toVectorX(state));
          })
      .def_prop_rw(
          "rigid_body_control_vector",
          &sim::World::getRigidBodyControlVector,
          [](sim::World& self, const nb::handle& control) {
            self.setRigidBodyControlVector(toVectorX(control));
          })
      .def("get_step_derivatives", &sim::World::getStepDerivatives)
      .def(
          "apply_step_vjp",
          [](const sim::World& self, const nb::handle& dLossDNextState) {
            return self.applyStepVjp(toVectorX(dLossDNextState));
          },
          nb::arg("d_loss_d_next_state"))
      .def(
          "add_differentiable_parameter",
          [](sim::World& self,
             const sim::RigidBody& body,
             sim::PhysicalParameter parameter,
             const nb::handle& lowerBound,
             const nb::handle& upperBound) {
            sim::PhysicalParameterSelector selector(body, parameter);
            if (!lowerBound.is_none()) {
              selector.lowerBound = nb::cast<double>(lowerBound);
            }
            if (!upperBound.is_none()) {
              selector.upperBound = nb::cast<double>(upperBound);
            }
            self.addDifferentiableParameter(selector);
          },
          nb::arg("body"),
          nb::arg("parameter") = sim::PhysicalParameter::MASS,
          nb::kw_only(),
          nb::arg("lower_bound") = nb::none(),
          nb::arg("upper_bound") = nb::none())
      .def_prop_ro(
          "num_differentiable_parameters",
          &sim::World::getNumDifferentiableParameters)
      .def(
          "set_collision_pair_ignored",
          [](sim::World& self,
             const nb::handle& first,
             const nb::handle& second,
             bool ignored) {
            self.setCollisionPairIgnored(
                toFrameHandle(first), toFrameHandle(second), ignored);
          },
          nb::arg("first"),
          nb::arg("second"),
          nb::arg("ignored") = true)
      .def(
          "is_collision_pair_ignored",
          [](const sim::World& self,
             const nb::handle& first,
             const nb::handle& second) {
            return self.isCollisionPairIgnored(
                toFrameHandle(first), toFrameHandle(second));
          },
          nb::arg("first"),
          nb::arg("second"))
      .def(
          "clear_ignored_collision_pairs",
          &sim::World::clearIgnoredCollisionPairs)
      .def_prop_ro(
          "num_ignored_collision_pairs",
          &sim::World::getIgnoredCollisionPairCount)
      .def(
          "collide",
          [](sim::World& self, const sim::CollisionQueryOptions& options) {
            return self.collide(options);
          },
          nb::arg("options") = sim::CollisionQueryOptions{})
      .def("clear", &sim::World::clear)
      .def("__repr__", [](const sim::World& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back(
            "multibodies", std::to_string(self.getMultibodyCount()));
        fields.emplace_back(
            "loop_closures", std::to_string(self.getLoopClosureCount()));
        fields.emplace_back(
            "rigid_bodies", std::to_string(self.getRigidBodyCount()));
        fields.emplace_back("joints", std::to_string(self.getJointCount()));
        fields.emplace_back("time", repr_double(self.getTime()));
        fields.emplace_back("time_step", repr_double(self.getTimeStep()));
        fields.emplace_back(
            "gravity",
            "(" + repr_double(self.getGravity().x()) + ", "
                + repr_double(self.getGravity().y()) + ", "
                + repr_double(self.getGravity().z()) + ")");
        fields.emplace_back("frame", std::to_string(self.getFrame()));
        fields.emplace_back(
            "simulation_mode",
            self.isSimulationMode() ? std::string("True")
                                    : std::string("False"));
        return format_repr("World", fields);
      });

  // Backend-neutral acceleration control for direct deformable PSD backend
  // calls. World stepping uses each World's compute_accelerator_policy.
  m.def(
      "is_accelerated_deformable_solve_available",
      &acceleratedDeformableSolveAvailable,
      "Whether a deformable-solve accelerator (e.g. an experimental device "
      "backend) is registered and reports an available device at runtime.");
  m.def(
      "set_accelerated_deformable_solve",
      &setAcceleratedDeformableSolve,
      nb::arg("enabled"),
      "Enable or disable accelerated (device-offloaded) deformable PSD "
      "projection for direct backend calls. World stepping uses each World's "
      "compute_accelerator_policy. Returns the resulting enabled state (False "
      "when no accelerator is available, so the call is a safe no-op that "
      "stays on the CPU backend).");
  m.def(
      "is_accelerated_deformable_solve_enabled",
      &acceleratedDeformableSolveEnabled,
      "Whether the accelerated deformable PSD backend is active in the current "
      "thread or installed as the direct-call process default.");
}

} // namespace dart::python_nb
