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

// Frame/FreeFrame/FixedFrame/Joint/Link classes and loop-closure/multibody
// option structs
void defSimPartFrames(nb::module_& m)
{
  auto frameClass = nb::class_<sim::Frame>(m, "Frame");
  auto freeFrameClass = nb::class_<sim::FreeFrame, sim::Frame>(m, "FreeFrame");
  auto fixedFrameClass
      = nb::class_<sim::FixedFrame, sim::Frame>(m, "FixedFrame");
  auto jointClass = nb::class_<sim::Joint>(m, "Joint");
  auto linkClass = nb::class_<sim::Link, sim::Frame>(m, "Link");
  auto loopClosureClass = nb::class_<sim::LoopClosure>(m, "LoopClosure");
  auto rigidBodyClass = nb::class_<sim::RigidBody, sim::Frame>(m, "RigidBody");

  frameClass.def_static("world", &sim::Frame::world)
      .def(
          "relative_transform",
          [](const sim::Frame& self, const nb::handle& relativeTo) {
            return self.getTransform(toFrameHandle(relativeTo)).matrix();
          },
          nb::arg("relative_to"))
      .def(
          "relative_transform",
          [](const sim::Frame& self,
             const nb::handle& to,
             const nb::handle& expressedIn) {
            return self
                .getTransform(toFrameHandle(to), toFrameHandle(expressedIn))
                .matrix();
          },
          nb::arg("relative_to"),
          nb::arg("expressed_in"))
      .def(
          "is_same_instance_as",
          [](const sim::Frame& self, const nb::handle& other) {
            return self.isSameInstanceAs(toFrameHandle(other));
          },
          nb::arg("other"))
      .def_prop_ro(
          "name",
          [](const sim::Frame& self) { return std::string(self.getName()); })
      .def_prop_rw(
          "parent_frame",
          &sim::Frame::getParentFrame,
          [](sim::Frame& self, const nb::handle& parent) {
            self.setParentFrame(toFrameHandle(parent));
          })
      .def_prop_ro(
          "local_transform",
          [](const sim::Frame& self) {
            return self.getLocalTransform().matrix();
          })
      .def_prop_ro("translation", &sim::Frame::getTranslation)
      .def_prop_ro("rotation", &sim::Frame::getRotation)
      .def_prop_ro(
          "quaternion",
          [](const sim::Frame& self) { return toWxyz(self.getQuaternion()); })
      .def_prop_ro("transform", &sim::Frame::getTransformMatrix)
      .def_prop_ro("is_valid", &sim::Frame::isValid)
      .def_prop_ro("is_world", &sim::Frame::isWorld)
      .def(
          "__eq__",
          [](const sim::Frame& self, const nb::handle& other) {
            try {
              return self == toFrameHandle(other);
            } catch (const nb::cast_error&) {
              return false;
            }
          },
          nb::is_operator())
      .def(
          "__ne__",
          [](const sim::Frame& self, const nb::handle& other) {
            try {
              return self != toFrameHandle(other);
            } catch (const nb::cast_error&) {
              return true;
            }
          },
          nb::is_operator())
      .def("__repr__", [](const sim::Frame& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("valid", self.isValid() ? "True" : "False");
        if (self.isValid()) {
          fields.emplace_back("name", repr_string(std::string(self.getName())));
        }
        return format_repr("Frame", fields);
      });

  freeFrameClass
      .def_prop_rw(
          "local_transform",
          [](const sim::FreeFrame& self) {
            return self.getLocalTransform().matrix();
          },
          [](sim::FreeFrame& self, const nb::handle& transform) {
            self.setLocalTransform(toIsometry(transform));
          })
      .def("__repr__", [](const sim::FreeFrame& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("valid", self.isValid() ? "True" : "False");
        if (self.isValid()) {
          fields.emplace_back("name", repr_string(std::string(self.getName())));
        }
        return format_repr("FreeFrame", fields);
      });

  fixedFrameClass
      .def_prop_rw(
          "local_transform",
          [](const sim::FixedFrame& self) {
            return self.getLocalTransform().matrix();
          },
          [](sim::FixedFrame& self, const nb::handle& transform) {
            self.setLocalTransform(toIsometry(transform));
          })
      .def("__repr__", [](const sim::FixedFrame& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("valid", self.isValid() ? "True" : "False");
        if (self.isValid()) {
          fields.emplace_back("name", repr_string(std::string(self.getName())));
        }
        return format_repr("FixedFrame", fields);
      });

  jointClass
      .def_prop_ro(
          "name",
          [](const sim::Joint& self) { return std::string(self.getName()); })
      .def_prop_ro("type", &sim::Joint::getType)
      .def_prop_rw(
          "actuator_type",
          &sim::Joint::getActuatorType,
          &sim::Joint::setActuatorType)
      .def_prop_rw(
          "command_velocity",
          &sim::Joint::getCommandVelocity,
          [](sim::Joint& self, const nb::handle& value) {
            self.setCommandVelocity(toVectorX(value));
          })
      .def_prop_ro("axis", &sim::Joint::getAxis)
      .def_prop_ro("axis2", &sim::Joint::getAxis2)
      .def_prop_rw("pitch", &sim::Joint::getPitch, &sim::Joint::setPitch)
      .def_prop_ro("num_dofs", &sim::Joint::getDOFCount)
      .def_prop_rw(
          "position",
          &sim::Joint::getPosition,
          [](sim::Joint& self, const nb::handle& position) {
            self.setPosition(toVectorX(position));
          })
      .def_prop_rw(
          "velocity",
          &sim::Joint::getVelocity,
          [](sim::Joint& self, const nb::handle& velocity) {
            self.setVelocity(toVectorX(velocity));
          })
      .def_prop_rw(
          "force",
          &sim::Joint::getForce,
          [](sim::Joint& self, const nb::handle& force) {
            self.setForce(toVectorX(force));
          })
      .def_prop_ro("acceleration", &sim::Joint::getAcceleration)
      .def_prop_rw(
          "spring_stiffness",
          &sim::Joint::getSpringStiffness,
          [](sim::Joint& self, const nb::handle& value) {
            self.setSpringStiffness(toVectorX(value));
          })
      .def_prop_rw(
          "rest_position",
          &sim::Joint::getRestPosition,
          [](sim::Joint& self, const nb::handle& value) {
            self.setRestPosition(toVectorX(value));
          })
      .def_prop_rw(
          "damping_coefficient",
          &sim::Joint::getDampingCoefficient,
          [](sim::Joint& self, const nb::handle& value) {
            self.setDampingCoefficient(toVectorX(value));
          })
      .def_prop_rw(
          "armature",
          &sim::Joint::getArmature,
          [](sim::Joint& self, const nb::handle& value) {
            self.setArmature(toVectorX(value));
          })
      .def_prop_rw(
          "coulomb_friction",
          &sim::Joint::getCoulombFriction,
          [](sim::Joint& self, const nb::handle& value) {
            self.setCoulombFriction(toVectorX(value));
          })
      .def(
          "set_position_limits",
          [](sim::Joint& self,
             const nb::handle& lower,
             const nb::handle& upper) {
            self.setPositionLimits(toVectorX(lower), toVectorX(upper));
          },
          nb::arg("lower"),
          nb::arg("upper"))
      .def_prop_ro("position_lower_limits", &sim::Joint::getPositionLowerLimits)
      .def_prop_ro("position_upper_limits", &sim::Joint::getPositionUpperLimits)
      .def(
          "set_velocity_limits",
          [](sim::Joint& self,
             const nb::handle& lower,
             const nb::handle& upper) {
            self.setVelocityLimits(toVectorX(lower), toVectorX(upper));
          },
          nb::arg("lower"),
          nb::arg("upper"))
      .def_prop_ro("velocity_lower_limits", &sim::Joint::getVelocityLowerLimits)
      .def_prop_ro("velocity_upper_limits", &sim::Joint::getVelocityUpperLimits)
      .def(
          "set_effort_limits",
          [](sim::Joint& self,
             const nb::handle& lower,
             const nb::handle& upper) {
            self.setEffortLimits(toVectorX(lower), toVectorX(upper));
          },
          nb::arg("lower"),
          nb::arg("upper"))
      .def_prop_ro("effort_lower_limits", &sim::Joint::getEffortLowerLimits)
      .def_prop_ro("effort_upper_limits", &sim::Joint::getEffortUpperLimits)
      .def_prop_rw(
          "break_force", &sim::Joint::getBreakForce, &sim::Joint::setBreakForce)
      .def_prop_ro("is_broken", &sim::Joint::isBroken)
      .def("reset_breakage", &sim::Joint::resetBreakage)
      .def_prop_rw(
          "constraint_projection_policy",
          &getJointConstraintProjectionPolicy,
          &setJointConstraintProjectionPolicy,
          nb::keep_alive<0, 1>())
      .def_prop_ro("parent_link", &sim::Joint::getParentLink)
      .def_prop_ro("child_link", &sim::Joint::getChildLink)
      .def_prop_ro("parent_rigid_body", &sim::Joint::getParentRigidBody)
      .def_prop_ro("child_rigid_body", &sim::Joint::getChildRigidBody)
      .def_prop_ro("is_valid", &sim::Joint::isValid)
      .def("__repr__", [](const sim::Joint& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("valid", self.isValid() ? "True" : "False");
        if (self.isValid()) {
          fields.emplace_back("name", repr_string(std::string(self.getName())));
          fields.emplace_back(
              "type",
              nb::cast<std::string>(nb::repr(nb::cast(self.getType()))));
        }
        return format_repr("Joint", fields);
      });

  linkClass
      .def_prop_ro(
          "name",
          [](const sim::Link& self) { return std::string(self.getName()); })
      .def_prop_ro("parent_joint", &sim::Link::getParentJoint)
      .def_prop_rw("mass", &sim::Link::getMass, &sim::Link::setMass)
      .def_prop_rw(
          "inertia",
          &sim::Link::getInertia,
          [](sim::Link& self, const nb::handle& inertia) {
            self.setInertia(toMatrix3(inertia));
          })
      .def_prop_rw(
          "center_of_mass",
          &sim::Link::getCenterOfMass,
          [](sim::Link& self, const nb::handle& centerOfMass) {
            self.setCenterOfMass(toVector3(centerOfMass));
          })
      .def(
          "apply_force",
          [](sim::Link& self,
             const nb::handle& force,
             const nb::handle& point,
             bool forceInWorldFrame,
             bool pointInWorldFrame) {
            self.applyForce(
                toVector3(force),
                toVector3(point),
                forceInWorldFrame,
                pointInWorldFrame);
          },
          nb::arg("force"),
          nb::arg("point") = Eigen::Vector3d::Zero(),
          nb::arg("force_in_world_frame") = true,
          nb::arg("point_in_world_frame") = false)
      .def_prop_ro("translation", &sim::Link::getTranslation)
      .def_prop_ro("rotation", &sim::Link::getRotation)
      .def_prop_ro(
          "quaternion",
          [](const sim::Link& self) { return toWxyz(self.getQuaternion()); })
      .def_prop_ro("transform", &sim::Link::getTransformMatrix)
      .def(
          "set_collision_shape",
          &sim::Link::setCollisionShape,
          nb::arg("shape"))
      .def(
          "add_collision_shape",
          &sim::Link::addCollisionShape,
          nb::arg("shape"))
      .def_prop_ro("collision_shape", &sim::Link::getCollisionShape)
      .def_prop_ro("collision_shapes", &sim::Link::getCollisionShapes)
      .def_prop_ro("has_collision_shape", &sim::Link::hasCollisionShape)
      .def_prop_ro("is_valid", &sim::Link::isValid)
      .def("__repr__", [](const sim::Link& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("valid", self.isValid() ? "True" : "False");
        if (self.isValid()) {
          fields.emplace_back("name", repr_string(std::string(self.getName())));
        }
        return format_repr("Link", fields);
      });

  nb::class_<sim::LoopClosureSpec>(m, "LoopClosureSpec")
      .def(
          nb::new_([](const nb::handle& frameA,
                      const nb::handle& frameB,
                      sim::LoopClosureFamily family,
                      const nb::handle& offsetA,
                      const nb::handle& offsetB,
                      double distance) {
            return makeLoopClosureSpec(
                frameA, frameB, family, offsetA, offsetB, distance);
          }),
          nb::arg("frame_a"),
          nb::arg("frame_b"),
          nb::arg("family") = sim::LoopClosureFamily::Rigid,
          nb::kw_only(),
          nb::arg("offset_a") = nb::none(),
          nb::arg("offset_b") = nb::none(),
          nb::arg("distance") = 0.0)
      .def_prop_rw(
          "frame_a",
          [](const sim::LoopClosureSpec& self) { return self.frameA; },
          [](sim::LoopClosureSpec& self, const nb::handle& frame) {
            self.frameA = toFrameHandle(frame);
          })
      .def_prop_rw(
          "frame_b",
          [](const sim::LoopClosureSpec& self) { return self.frameB; },
          [](sim::LoopClosureSpec& self, const nb::handle& frame) {
            self.frameB = toFrameHandle(frame);
          })
      .def_rw("family", &sim::LoopClosureSpec::family)
      .def_rw("distance", &sim::LoopClosureSpec::distance)
      .def_prop_rw(
          "offset_a",
          [](const sim::LoopClosureSpec& self) {
            return self.offsetA.matrix();
          },
          [](sim::LoopClosureSpec& self, const nb::handle& offset) {
            self.offsetA = toIsometry(offset);
          })
      .def_prop_rw(
          "offset_b",
          [](const sim::LoopClosureSpec& self) {
            return self.offsetB.matrix();
          },
          [](sim::LoopClosureSpec& self, const nb::handle& offset) {
            self.offsetB = toIsometry(offset);
          })
      .def("__repr__", [](const sim::LoopClosureSpec& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back(
            "frame_a", nb::cast<std::string>(nb::repr(nb::cast(self.frameA))));
        fields.emplace_back(
            "frame_b", nb::cast<std::string>(nb::repr(nb::cast(self.frameB))));
        fields.emplace_back(
            "family", nb::cast<std::string>(nb::repr(nb::cast(self.family))));
        return format_repr("LoopClosureSpec", fields);
      });

  nb::class_<sim::LoopClosureRuntimePolicy>(m, "LoopClosureRuntimePolicy")
      .def(
          nb::new_([](bool enabled,
                      sim::ClosureKinematicsPolicy kinematics,
                      sim::ClosureDynamicsPolicy dynamics) {
            return sim::LoopClosureRuntimePolicy{
                .enabled = enabled,
                .kinematics = kinematics,
                .dynamics = dynamics};
          }),
          nb::arg("enabled") = true,
          nb::arg("kinematics") = sim::ClosureKinematicsPolicy::ResidualOnly,
          nb::arg("dynamics") = sim::ClosureDynamicsPolicy::ResidualOnly)
      .def_rw("enabled", &sim::LoopClosureRuntimePolicy::enabled)
      .def_rw("kinematics", &sim::LoopClosureRuntimePolicy::kinematics)
      .def_rw("dynamics", &sim::LoopClosureRuntimePolicy::dynamics)
      .def("__repr__", [](const sim::LoopClosureRuntimePolicy& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("enabled", self.enabled ? "True" : "False");
        fields.emplace_back(
            "kinematics",
            nb::cast<std::string>(nb::repr(nb::cast(self.kinematics))));
        fields.emplace_back(
            "dynamics",
            nb::cast<std::string>(nb::repr(nb::cast(self.dynamics))));
        return format_repr("LoopClosureRuntimePolicy", fields);
      });

  nb::class_<sim::MultibodyOptions>(m, "MultibodyOptions")
      .def(
          nb::new_([](sim::MultibodyIntegrationFamily integrationFamily,
                      std::size_t variationalMaxIterations,
                      double variationalTolerance) {
            return sim::MultibodyOptions{
                .integrationFamily = integrationFamily,
                .variationalMaxIterations = variationalMaxIterations,
                .variationalTolerance = variationalTolerance};
          }),
          nb::arg("integration_family")
          = sim::MultibodyIntegrationFamily::SemiImplicit,
          nb::arg("variational_max_iterations") = 100,
          nb::arg("variational_tolerance") = 1e-10)
      .def_rw("integration_family", &sim::MultibodyOptions::integrationFamily)
      .def_rw(
          "variational_max_iterations",
          &sim::MultibodyOptions::variationalMaxIterations)
      .def_rw(
          "variational_tolerance", &sim::MultibodyOptions::variationalTolerance)
      .def("__repr__", [](const sim::MultibodyOptions& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back(
            "integration_family",
            nb::cast<std::string>(nb::repr(nb::cast(self.integrationFamily))));
        fields.emplace_back(
            "variational_max_iterations",
            std::to_string(self.variationalMaxIterations));
        fields.emplace_back(
            "variational_tolerance", std::to_string(self.variationalTolerance));
        return format_repr("MultibodyOptions", fields);
      });

  nb::class_<sim::DeactivationOptions>(m, "DeactivationOptions")
      .def(
          nb::new_([](bool enabled,
                      double linearSpeedThreshold,
                      double angularSpeedThreshold,
                      double generalizedSpeedThreshold,
                      double timeUntilSleep,
                      double wakeThresholdScale,
                      double disturbanceForceThreshold) {
            sim::DeactivationOptions options;
            options.enabled = enabled;
            options.linearSpeedThreshold = linearSpeedThreshold;
            options.angularSpeedThreshold = angularSpeedThreshold;
            options.generalizedSpeedThreshold = generalizedSpeedThreshold;
            options.timeUntilSleep = timeUntilSleep;
            options.wakeThresholdScale = wakeThresholdScale;
            options.disturbanceForceThreshold = disturbanceForceThreshold;
            return options;
          }),
          nb::arg("enabled") = false,
          nb::arg("linear_speed_threshold") = 1e-3,
          nb::arg("angular_speed_threshold") = 1e-3,
          nb::arg("generalized_speed_threshold") = 1e-3,
          nb::arg("time_until_sleep") = 0.5,
          nb::arg("wake_threshold_scale") = 2.0,
          nb::arg("disturbance_force_threshold") = 1e-9)
      .def_rw("enabled", &sim::DeactivationOptions::enabled)
      .def_rw(
          "linear_speed_threshold",
          &sim::DeactivationOptions::linearSpeedThreshold)
      .def_rw(
          "angular_speed_threshold",
          &sim::DeactivationOptions::angularSpeedThreshold)
      .def_rw(
          "generalized_speed_threshold",
          &sim::DeactivationOptions::generalizedSpeedThreshold)
      .def_rw("time_until_sleep", &sim::DeactivationOptions::timeUntilSleep)
      .def_rw(
          "wake_threshold_scale", &sim::DeactivationOptions::wakeThresholdScale)
      .def_rw(
          "disturbance_force_threshold",
          &sim::DeactivationOptions::disturbanceForceThreshold)
      .def("__repr__", [](const sim::DeactivationOptions& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("enabled", repr_bool(self.enabled));
        fields.emplace_back(
            "linear_speed_threshold",
            std::to_string(self.linearSpeedThreshold));
        fields.emplace_back(
            "angular_speed_threshold",
            std::to_string(self.angularSpeedThreshold));
        fields.emplace_back(
            "generalized_speed_threshold",
            std::to_string(self.generalizedSpeedThreshold));
        fields.emplace_back(
            "time_until_sleep", std::to_string(self.timeUntilSleep));
        fields.emplace_back(
            "wake_threshold_scale", std::to_string(self.wakeThresholdScale));
        fields.emplace_back(
            "disturbance_force_threshold",
            std::to_string(self.disturbanceForceThreshold));
        return format_repr("DeactivationOptions", fields);
      });

  nb::class_<sim::LoopClosureResidual>(m, "LoopClosureResidual")
      .def_prop_ro(
          "value",
          [](const sim::LoopClosureResidual& self) { return self.value; })
      .def_ro("norm", &sim::LoopClosureResidual::norm)
      .def_ro("enabled", &sim::LoopClosureResidual::enabled)
      .def_ro("active", &sim::LoopClosureResidual::active)
      .def_ro("coordinates", &sim::LoopClosureResidual::coordinates)
      .def_ro("force_available", &sim::LoopClosureResidual::forceAvailable)
      .def("__repr__", [](const sim::LoopClosureResidual& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back(
            "value", nb::cast<std::string>(nb::repr(nb::cast(self.value))));
        fields.emplace_back("norm", std::to_string(self.norm));
        fields.emplace_back("enabled", self.enabled ? "True" : "False");
        fields.emplace_back("active", self.active ? "True" : "False");
        fields.emplace_back(
            "coordinates",
            nb::cast<std::string>(nb::repr(nb::cast(self.coordinates))));
        fields.emplace_back(
            "force_available", self.forceAvailable ? "True" : "False");
        return format_repr("LoopClosureResidual", fields);
      });

  loopClosureClass.def("compute_residual", &sim::LoopClosure::computeResidual)
      .def_prop_ro(
          "name",
          [](const sim::LoopClosure& self) {
            return std::string(self.getName());
          })
      .def_prop_ro("family", &sim::LoopClosure::getFamily)
      .def_prop_ro("frame_a", &sim::LoopClosure::getFrameA)
      .def_prop_ro("frame_b", &sim::LoopClosure::getFrameB)
      .def_prop_ro(
          "offset_a",
          [](const sim::LoopClosure& self) {
            return self.getOffsetA().matrix();
          })
      .def_prop_ro(
          "offset_b",
          [](const sim::LoopClosure& self) {
            return self.getOffsetB().matrix();
          })
      .def_prop_rw(
          "runtime_policy",
          &sim::LoopClosure::getRuntimePolicy,
          &sim::LoopClosure::setRuntimePolicy)
      .def_prop_rw(
          "enabled",
          [](const sim::LoopClosure& self) {
            return self.getRuntimePolicy().enabled;
          },
          [](sim::LoopClosure& self, bool enabled) {
            auto policy = self.getRuntimePolicy();
            policy.enabled = enabled;
            self.setRuntimePolicy(policy);
          })
      .def_prop_rw(
          "kinematics",
          [](const sim::LoopClosure& self) {
            return self.getRuntimePolicy().kinematics;
          },
          [](sim::LoopClosure& self, sim::ClosureKinematicsPolicy kinematics) {
            auto policy = self.getRuntimePolicy();
            policy.kinematics = kinematics;
            self.setRuntimePolicy(policy);
          })
      .def_prop_rw(
          "dynamics",
          [](const sim::LoopClosure& self) {
            return self.getRuntimePolicy().dynamics;
          },
          [](sim::LoopClosure& self, sim::ClosureDynamicsPolicy dynamics) {
            auto policy = self.getRuntimePolicy();
            policy.dynamics = dynamics;
            self.setRuntimePolicy(policy);
          })
      .def_prop_ro("is_valid", &sim::LoopClosure::isValid)
      .def("__repr__", [](const sim::LoopClosure& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("valid", self.isValid() ? "True" : "False");
        if (self.isValid()) {
          fields.emplace_back("name", repr_string(std::string(self.getName())));
          fields.emplace_back(
              "family",
              nb::cast<std::string>(nb::repr(nb::cast(self.getFamily()))));
        }
        return format_repr("LoopClosure", fields);
      });
}

} // namespace dart::python_nb
