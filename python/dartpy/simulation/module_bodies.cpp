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

// Multibody/RigidBody/Contact/StepGradient and (optional) diff rollout bindings
void defSimPartBodies(nb::module_& m)
{
  nb::class_<sim::Multibody>(m, "Multibody")
      .def(
          "add_link",
          [](sim::Multibody& self, const std::string& name) {
            return self.addLink(name);
          },
          nb::arg("name") = "",
          nb::keep_alive<0, 1>())
      .def(
          "add_link",
          [](sim::Multibody& self,
             const std::string& name,
             const sim::Link& parent,
             const sim::JointSpec& joint) {
            return self.addLink(name, parent, joint);
          },
          nb::arg("name"),
          nb::kw_only(),
          nb::arg("parent"),
          nb::arg("joint") = sim::JointSpec{},
          nb::keep_alive<0, 1>())
      .def(
          "get_link",
          [](sim::Multibody& self, const std::string& name) -> nb::object {
            auto link = self.getLink(name);
            if (!link.has_value()) {
              return nb::none();
            }
            return nb::cast(*link, nb::rv_policy::move);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "get_joint",
          [](sim::Multibody& self, const std::string& name) -> nb::object {
            auto joint = self.getJoint(name);
            if (!joint.has_value()) {
              return nb::none();
            }
            return nb::cast(*joint, nb::rv_policy::move);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "set_ground_contact",
          &sim::Multibody::setGroundContact,
          nb::arg("plane_normal"),
          nb::arg("plane_point"),
          nb::arg("stiffness"),
          nb::arg("friction_coefficient") = 0.0,
          nb::arg("friction_regularization") = 1.0e-4,
          nb::arg("damping_coefficient") = 0.0,
          nb::arg("dual_update_cadence") = 0,
          "Configure compliant ground contact for the variational integrator "
          "(an analytic half-space + penalty/friction/damping); add points "
          "with add_ground_contact_point(). dual_update_cadence=0 is the C2 "
          "compliant penalty; >0 enables the C3 augmented-Lagrangian rung, "
          "advancing the duals every N steps for drift-free contact.")
      .def(
          "add_ground_contact_point",
          &sim::Multibody::addGroundContactPoint,
          nb::arg("link"),
          nb::arg("local_point"),
          "Add a body-fixed contact point against the configured ground plane.")
      .def_prop_rw(
          "name",
          [](const sim::Multibody& self) {
            return std::string(self.getName());
          },
          [](sim::Multibody& self, const std::string& name) {
            self.setName(name);
          })
      .def_prop_ro("num_links", &sim::Multibody::getLinkCount)
      .def_prop_ro("num_joints", &sim::Multibody::getJointCount)
      .def_prop_ro("num_dofs", &sim::Multibody::getDOFCount)
      .def_prop_ro("links", &sim::Multibody::getLinks)
      .def_prop_ro("joints", &sim::Multibody::getJoints)
      .def_prop_ro("link_names", &sim::Multibody::getLinkNames)
      .def_prop_ro("joint_names", &sim::Multibody::getJointNames)
      .def_prop_ro("is_sleeping", &sim::Multibody::isSleeping)
      .def_prop_ro(
          "deactivation_group_index",
          &sim::Multibody::getDeactivationGroupIndex)
      .def_prop_ro("mass_matrix", &sim::Multibody::getMassMatrix)
      .def_prop_ro("inverse_mass_matrix", &sim::Multibody::getInverseMassMatrix)
      .def_prop_ro("coriolis_forces", &sim::Multibody::getCoriolisForces)
      .def_prop_ro("gravity_forces", &sim::Multibody::getGravityForces)
      .def_prop_ro(
          "coriolis_and_gravity_forces",
          &sim::Multibody::getCoriolisAndGravityForces)
      .def(
          "compute_inverse_dynamics",
          [](const sim::Multibody& self, const nb::handle& acceleration) {
            return self.computeInverseDynamics(toVectorX(acceleration));
          },
          nb::arg("desired_acceleration"))
      .def(
          "compute_impulse_response",
          [](const sim::Multibody& self, const nb::handle& impulse) {
            return self.computeImpulseResponse(toVectorX(impulse));
          },
          nb::arg("joint_impulse"))
      .def(
          "get_jacobian",
          [](const sim::Multibody& self, const sim::Link& link) {
            return self.getJacobian(link);
          },
          nb::arg("link"))
      .def(
          "get_world_jacobian",
          [](const sim::Multibody& self, const sim::Link& link) {
            return self.getWorldJacobian(link);
          },
          nb::arg("link"))
      .def_prop_ro("is_valid", &sim::Multibody::isValid)
      .def("__repr__", [](const sim::Multibody& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("valid", self.isValid() ? "True" : "False");
        if (self.isValid()) {
          fields.emplace_back("name", repr_string(std::string(self.getName())));
          fields.emplace_back("links", std::to_string(self.getLinkCount()));
          fields.emplace_back("joints", std::to_string(self.getJointCount()));
          fields.emplace_back("dofs", std::to_string(self.getDOFCount()));
        }
        return format_repr("Multibody", fields);
      });

  // sim::RigidBody is registered in defSimPartFrames() so that earlier bindings
  // (e.g. Joint.parent_rigid_body) can reference the type. Borrow the existing
  // type object here to attach its methods -- re-running nb::class_<...>(m,
  // "RigidBody") would re-register the type and emit a RuntimeWarning.
  auto rigidBodyClass = nb::borrow<nb::class_<sim::RigidBody, sim::Frame>>(
      nb::type<sim::RigidBody>());
  rigidBodyClass
      .def(
          "apply_force",
          [](sim::RigidBody& self, const nb::handle& force) {
            self.applyForce(toVector3(force));
          },
          nb::arg("force"))
      .def("clear_force", &sim::RigidBody::clearForce)
      .def(
          "apply_torque",
          [](sim::RigidBody& self, const nb::handle& torque) {
            self.applyTorque(toVector3(torque));
          },
          nb::arg("torque"))
      .def("clear_torque", &sim::RigidBody::clearTorque)
      .def(
          "apply_linear_impulse",
          [](sim::RigidBody& self, const nb::handle& impulse) {
            self.applyLinearImpulse(toVector3(impulse));
          },
          nb::arg("impulse"))
      .def(
          "apply_angular_impulse",
          [](sim::RigidBody& self, const nb::handle& impulse) {
            self.applyAngularImpulse(toVector3(impulse));
          },
          nb::arg("impulse"))
      .def_prop_ro(
          "name", [](const sim::RigidBody& self) { return self.getName(); })
      .def_prop_ro("translation", &sim::RigidBody::getTranslation)
      .def_prop_ro("rotation", &sim::RigidBody::getRotation)
      .def_prop_ro(
          "quaternion",
          [](const sim::RigidBody& self) {
            return toWxyz(self.getQuaternion());
          })
      .def_prop_rw(
          "transform",
          &sim::RigidBody::getTransformMatrix,
          [](sim::RigidBody& self, const nb::handle& transform) {
            self.setTransform(toIsometry(transform));
          })
      .def_prop_rw(
          "linear_velocity",
          &sim::RigidBody::getLinearVelocity,
          [](sim::RigidBody& self, const nb::handle& velocity) {
            self.setLinearVelocity(toVector3(velocity));
          })
      .def_prop_rw(
          "angular_velocity",
          &sim::RigidBody::getAngularVelocity,
          [](sim::RigidBody& self, const nb::handle& velocity) {
            self.setAngularVelocity(toVector3(velocity));
          })
      .def_prop_rw("mass", &sim::RigidBody::getMass, &sim::RigidBody::setMass)
      .def_prop_rw(
          "inertia",
          &sim::RigidBody::getInertia,
          [](sim::RigidBody& self, const nb::handle& inertia) {
            self.setInertia(toMatrix3(inertia));
          })
      .def_prop_rw(
          "force",
          &sim::RigidBody::getForce,
          [](sim::RigidBody& self, const nb::handle& force) {
            self.setForce(toVector3(force));
          })
      .def_prop_rw(
          "torque",
          &sim::RigidBody::getTorque,
          [](sim::RigidBody& self, const nb::handle& torque) {
            self.setTorque(toVector3(torque));
          })
      .def_prop_rw(
          "is_static", &sim::RigidBody::isStatic, &sim::RigidBody::setStatic)
      .def_prop_rw(
          "is_kinematic",
          &sim::RigidBody::isKinematic,
          &sim::RigidBody::setKinematic)
      .def_prop_ro("is_sleeping", &sim::RigidBody::isSleeping)
      .def_prop_ro(
          "deactivation_group_index",
          &sim::RigidBody::getDeactivationGroupIndex)
      .def_prop_rw(
          "restitution",
          &sim::RigidBody::getRestitution,
          &sim::RigidBody::setRestitution)
      .def_prop_rw(
          "friction",
          &sim::RigidBody::getFriction,
          &sim::RigidBody::setFriction)
      .def(
          "set_collision_shape",
          &sim::RigidBody::setCollisionShape,
          nb::arg("shape"))
      .def(
          "add_collision_shape",
          &sim::RigidBody::addCollisionShape,
          nb::arg("shape"))
      .def_prop_rw(
          "deformable_obstacle_policy",
          &getDeformableObstaclePolicy,
          &setDeformableObstaclePolicy,
          nb::keep_alive<0, 1>())
      .def_prop_ro("collision_shape", &sim::RigidBody::getCollisionShape)
      .def_prop_ro("collision_shapes", &sim::RigidBody::getCollisionShapes)
      .def_prop_ro("has_collision_shape", &sim::RigidBody::hasCollisionShape)
      .def_prop_ro("linear_momentum", &sim::RigidBody::getLinearMomentum)
      .def_prop_ro("angular_momentum", &sim::RigidBody::getAngularMomentum)
      .def_prop_ro("kinetic_energy", &sim::RigidBody::getKineticEnergy)
      .def_prop_ro("potential_energy", &sim::RigidBody::getPotentialEnergy)
      .def("__repr__", [](const sim::RigidBody& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("name", repr_string(self.getName()));
        return format_repr("RigidBody", fields);
      });

  nb::class_<sim::RigidBodyOptions>(m, "RigidBodyOptions")
      .def(
          nb::new_([](double mass,
                      const nb::handle& position,
                      const nb::handle& orientation,
                      const nb::handle& linearVelocity,
                      const nb::handle& angularVelocity,
                      const nb::handle& inertia) {
            return makeRigidBodyOptions(
                mass,
                position,
                orientation,
                linearVelocity,
                angularVelocity,
                inertia);
          }),
          nb::arg("mass") = 1.0,
          nb::arg("position") = nb::none(),
          nb::arg("orientation") = nb::none(),
          nb::arg("linear_velocity") = nb::none(),
          nb::arg("angular_velocity") = nb::none(),
          nb::arg("inertia") = nb::none())
      .def_prop_rw(
          "mass",
          [](const sim::RigidBodyOptions& self) { return self.mass; },
          [](sim::RigidBodyOptions& self, double mass) {
            validateMass(mass);
            self.mass = mass;
          })
      .def_prop_rw(
          "position",
          [](const sim::RigidBodyOptions& self) { return self.position; },
          [](sim::RigidBodyOptions& self, const nb::handle& position) {
            const auto value = toVector3(position);
            validateFiniteVector(value, "position");
            self.position = value;
          })
      .def_prop_rw(
          "orientation",
          [](const sim::RigidBodyOptions& self) {
            return toWxyz(self.orientation);
          },
          [](sim::RigidBodyOptions& self, const nb::handle& orientation) {
            const auto value = toQuaternionWxyz(orientation);
            validateOrientation(value);
            self.orientation = value;
          })
      .def_prop_rw(
          "linear_velocity",
          [](const sim::RigidBodyOptions& self) { return self.linearVelocity; },
          [](sim::RigidBodyOptions& self, const nb::handle& linearVelocity) {
            const auto value = toVector3(linearVelocity);
            validateFiniteVector(value, "linear_velocity");
            self.linearVelocity = value;
          })
      .def_prop_rw(
          "angular_velocity",
          [](const sim::RigidBodyOptions& self) {
            return self.angularVelocity;
          },
          [](sim::RigidBodyOptions& self, const nb::handle& angularVelocity) {
            const auto value = toVector3(angularVelocity);
            validateFiniteVector(value, "angular_velocity");
            self.angularVelocity = value;
          })
      .def_prop_rw(
          "inertia",
          [](const sim::RigidBodyOptions& self) { return self.inertia; },
          [](sim::RigidBodyOptions& self, const nb::handle& inertia) {
            const auto value = toMatrix3(inertia);
            validateInertia(value);
            self.inertia = value;
          })
      .def_prop_rw(
          "is_static",
          [](const sim::RigidBodyOptions& self) { return self.isStatic; },
          [](sim::RigidBodyOptions& self, bool isStatic) {
            self.isStatic = isStatic;
          })
      .def("__repr__", [](const sim::RigidBodyOptions& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("mass", repr_double(self.mass));
        fields.emplace_back(
            "position",
            nb::cast<std::string>(nb::repr(nb::cast(self.position))));
        fields.emplace_back(
            "linear_velocity",
            nb::cast<std::string>(nb::repr(nb::cast(self.linearVelocity))));
        fields.emplace_back(
            "angular_velocity",
            nb::cast<std::string>(nb::repr(nb::cast(self.angularVelocity))));
        return format_repr("RigidBodyOptions", fields);
      });

  nb::class_<sim::CollisionBody>(m, "CollisionBody")
      .def_prop_ro(
          "name", [](const sim::CollisionBody& self) { return self.getName(); })
      .def_prop_ro("is_rigid_body", &sim::CollisionBody::isRigidBody)
      .def_prop_ro("is_link", &sim::CollisionBody::isLink)
      .def_prop_ro("is_valid", &sim::CollisionBody::isValid)
      .def(
          "as_rigid_body",
          [](const sim::CollisionBody& self) -> nb::object {
            auto body = self.asRigidBody();
            if (!body.has_value()) {
              return nb::none();
            }
            return nb::cast(*body, nb::rv_policy::move);
          })
      .def("as_link", [](const sim::CollisionBody& self) -> nb::object {
        auto link = self.asLink();
        if (!link.has_value()) {
          return nb::none();
        }
        return nb::cast(*link, nb::rv_policy::move);
      });

  nb::class_<sim::Contact>(m, "Contact")
      .def_prop_ro(
          "body_a", [](const sim::Contact& self) { return self.bodyA; })
      .def_prop_ro(
          "body_b", [](const sim::Contact& self) { return self.bodyB; })
      .def_prop_ro("point", [](const sim::Contact& self) { return self.point; })
      .def_prop_ro(
          "normal", [](const sim::Contact& self) { return self.normal; })
      .def_prop_ro("depth", [](const sim::Contact& self) { return self.depth; })
      .def_prop_ro(
          "shape_index_a",
          [](const sim::Contact& self) { return self.shapeIndexA; })
      .def_prop_ro(
          "shape_index_b",
          [](const sim::Contact& self) { return self.shapeIndexB; })
      .def_prop_ro(
          "local_point_a",
          [](const sim::Contact& self) { return self.localPointA; })
      .def_prop_ro("local_point_b", [](const sim::Contact& self) {
        return self.localPointB;
      });

  nb::class_<sim::CollisionQueryOptions>(m, "CollisionQueryOptions")
      .def(
          "__init__",
          [](sim::CollisionQueryOptions* self,
             bool includeSameMultibodyLinkPairs,
             bool includeRigidBodyPairs,
             bool includeRigidBodyLinkPairs,
             bool includeLinkPairs) {
            sim::CollisionQueryOptions options;
            options.includeSameMultibodyLinkPairs
                = includeSameMultibodyLinkPairs;
            options.includeRigidBodyPairs = includeRigidBodyPairs;
            options.includeRigidBodyLinkPairs = includeRigidBodyLinkPairs;
            options.includeLinkPairs = includeLinkPairs;
            new (self) sim::CollisionQueryOptions(options);
          },
          nb::arg("include_same_multibody_link_pairs") = true,
          nb::kw_only(),
          nb::arg("include_rigid_body_pairs") = true,
          nb::arg("include_rigid_body_link_pairs") = true,
          nb::arg("include_link_pairs") = true)
      .def_rw(
          "include_same_multibody_link_pairs",
          &sim::CollisionQueryOptions::includeSameMultibodyLinkPairs)
      .def_rw(
          "include_rigid_body_pairs",
          &sim::CollisionQueryOptions::includeRigidBodyPairs)
      .def_rw(
          "include_rigid_body_link_pairs",
          &sim::CollisionQueryOptions::includeRigidBodyLinkPairs)
      .def_rw(
          "include_link_pairs", &sim::CollisionQueryOptions::includeLinkPairs)
      .def("__repr__", [](const sim::CollisionQueryOptions& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back(
            "include_same_multibody_link_pairs",
            repr_bool(self.includeSameMultibodyLinkPairs));
        fields.emplace_back(
            "include_rigid_body_pairs", repr_bool(self.includeRigidBodyPairs));
        fields.emplace_back(
            "include_rigid_body_link_pairs",
            repr_bool(self.includeRigidBodyLinkPairs));
        fields.emplace_back(
            "include_link_pairs", repr_bool(self.includeLinkPairs));
        return format_repr("CollisionQueryOptions", fields);
      });

  nb::class_<sim::StepDerivatives>(m, "StepDerivatives")
      .def_prop_ro(
          "state_jacobian",
          [](const sim::StepDerivatives& self) { return self.stateJacobian; })
      .def_prop_ro(
          "control_jacobian",
          [](const sim::StepDerivatives& self) { return self.controlJacobian; })
      .def_prop_ro(
          "parameter_jacobian",
          [](const sim::StepDerivatives& self) {
            return self.parameterJacobian;
          })
      .def("__repr__", [](const sim::StepDerivatives& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back(
            "state_jacobian",
            std::to_string(self.stateJacobian.rows()) + "x"
                + std::to_string(self.stateJacobian.cols()));
        fields.emplace_back(
            "control_jacobian",
            std::to_string(self.controlJacobian.rows()) + "x"
                + std::to_string(self.controlJacobian.cols()));
        fields.emplace_back(
            "parameter_jacobian",
            std::to_string(self.parameterJacobian.rows()) + "x"
                + std::to_string(self.parameterJacobian.cols()));
        return format_repr("StepDerivatives", fields);
      });

  nb::class_<sim::StepGradient>(m, "StepGradient")
      .def_prop_ro(
          "state", [](const sim::StepGradient& self) { return self.state; })
      .def_prop_ro(
          "control", [](const sim::StepGradient& self) { return self.control; })
      .def("__repr__", [](const sim::StepGradient& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("state", std::to_string(self.state.size()));
        fields.emplace_back("control", std::to_string(self.control.size()));
        return format_repr("StepGradient", fields);
      });

#ifdef DART_HAS_DIFF
  // Framework-neutral multi-step rollout (PLAN-110 rollout item). Exposed in
  // C++ on the experimental module here; __init__.py re-exports it onto the
  // pure-Python ``sx.diff`` namespace as ``sx.diff.rollout`` /
  // ``sx.diff.RolloutTrajectory``. This is the torch-free path; the torch
  // ``sx.diff.timestep`` chaining bridge stays as-is.
  nb::class_<sim::diff::RolloutTrajectory>(m, "RolloutTrajectory")
      .def_prop_ro(
          "states",
          [](const sim::diff::RolloutTrajectory& self) {
            // Stack the recorded trajectory into a (steps+1) x state_dim
            // row-major matrix so Python sees a single numpy array.
            const auto rows = static_cast<Eigen::Index>(self.states.size());
            const Eigen::Index cols = rows > 0 ? self.states.front().size() : 0;
            Eigen::MatrixXd out(rows, cols);
            for (Eigen::Index i = 0; i < rows; ++i) {
              out.row(i) = self.states[static_cast<std::size_t>(i)].transpose();
            }
            return out;
          })
      .def_prop_ro("num_steps", &sim::diff::RolloutTrajectory::numSteps)
      .def(
          "gradients",
          [](const sim::diff::RolloutTrajectory& self,
             const nb::handle& finalStateGrad) {
            const sim::diff::RolloutGradient gradient
                = self.rolloutVjp(toVectorX(finalStateGrad));
            // Stack the per-step control gradients into a steps x num_efforts
            // matrix; an empty rollout (no steps) yields a 0 x 0 array.
            const auto rows
                = static_cast<Eigen::Index>(gradient.controlGrads.size());
            const Eigen::Index cols
                = rows > 0 ? gradient.controlGrads.front().size() : 0;
            Eigen::MatrixXd controlGrads(rows, cols);
            for (Eigen::Index t = 0; t < rows; ++t) {
              controlGrads.row(t)
                  = gradient.controlGrads[static_cast<std::size_t>(t)]
                        .transpose();
            }
            return nb::make_tuple(gradient.initialStateGrad, controlGrads);
          },
          nb::arg("final_state_grad"))
      .def("__repr__", [](const sim::diff::RolloutTrajectory& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("steps", std::to_string(self.numSteps()));
        const Eigen::Index stateDim
            = self.states.empty() ? 0 : self.states.front().size();
        fields.emplace_back("state_dim", std::to_string(stateDim));
        return format_repr("RolloutTrajectory", fields);
      });

  m.def(
      "rollout",
      [](sim::World& world,
         const nb::handle& initialStateVector,
         const Eigen::MatrixXd& controlSequence,
         std::size_t steps) {
        return sim::diff::rollout(
            world, toVectorX(initialStateVector), controlSequence, steps);
      },
      nb::arg("world"),
      nb::arg("initial_state_vector"),
      nb::arg("control_sequence"),
      nb::arg("steps"));
#endif // DART_HAS_DIFF
  nb::class_<sim::DeformableSolverDiagnostics>(m, "DeformableSolverDiagnostics")
      .def_ro("body_count", &sim::DeformableSolverDiagnostics::bodyCount)
      .def_ro("node_count", &sim::DeformableSolverDiagnostics::nodeCount)
      .def_ro("edge_count", &sim::DeformableSolverDiagnostics::edgeCount)
      .def_ro(
          "solver_iterations",
          &sim::DeformableSolverDiagnostics::solverIterations)
      .def_ro(
          "objective_evaluations",
          &sim::DeformableSolverDiagnostics::objectiveEvaluations)
      .def_ro(
          "line_search_trials",
          &sim::DeformableSolverDiagnostics::lineSearchTrials)
      .def_ro(
          "projected_newton_steps",
          &sim::DeformableSolverDiagnostics::projectedNewtonSteps)
      .def_ro(
          "projected_newton_fallbacks",
          &sim::DeformableSolverDiagnostics::projectedNewtonFallbacks)
      .def_ro(
          "projected_newton_hessian_nonzeros",
          &sim::DeformableSolverDiagnostics::projectedNewtonHessianNonZeros)
      .def_ro(
          "projected_newton_hessian_storage_bytes",
          &sim::DeformableSolverDiagnostics::projectedNewtonHessianStorageBytes)
      .def_ro(
          "projected_newton_iterative_solves",
          &sim::DeformableSolverDiagnostics::projectedNewtonIterativeSolves)
      .def_ro(
          "projected_newton_matrix_free_solves",
          &sim::DeformableSolverDiagnostics::projectedNewtonMatrixFreeSolves)
      .def_ro(
          "projected_newton_iterative_iterations",
          &sim::DeformableSolverDiagnostics::projectedNewtonIterativeIterations)
      .def_ro(
          "projected_newton_iterative_max_error",
          &sim::DeformableSolverDiagnostics::projectedNewtonIterativeMaxError)
      .def_ro(
          "self_contact_barrier_active_contacts",
          &sim::DeformableSolverDiagnostics::selfContactBarrierActiveContacts)
      .def_ro(
          "surface_contact_candidate_builds",
          &sim::DeformableSolverDiagnostics::surfaceContactCandidateBuilds)
      .def_ro(
          "surface_contact_candidate_pair_capacity",
          &sim::DeformableSolverDiagnostics::
              surfaceContactCandidatePairCapacity)
      .def_ro(
          "surface_contact_candidate_rejected_pairs",
          &sim::DeformableSolverDiagnostics::
              surfaceContactCandidateRejectedPairs)
      .def_ro(
          "surface_contact_point_triangle_candidates",
          &sim::DeformableSolverDiagnostics::
              surfaceContactPointTriangleCandidates)
      .def_ro(
          "surface_contact_edge_edge_candidates",
          &sim::DeformableSolverDiagnostics::surfaceContactEdgeEdgeCandidates)
      .def_ro(
          "surface_contact_ccd_point_triangle_checks",
          &sim::DeformableSolverDiagnostics::
              surfaceContactCcdPointTriangleChecks)
      .def_ro(
          "surface_contact_ccd_edge_edge_checks",
          &sim::DeformableSolverDiagnostics::surfaceContactCcdEdgeEdgeChecks)
      .def_ro(
          "surface_contact_ccd_hits",
          &sim::DeformableSolverDiagnostics::surfaceContactCcdHits)
      .def_ro(
          "surface_contact_ccd_limited_steps",
          &sim::DeformableSolverDiagnostics::surfaceContactCcdLimitedSteps)
      .def_ro(
          "surface_contact_ccd_zero_step_count",
          &sim::DeformableSolverDiagnostics::surfaceContactCcdZeroStepCount)
      .def_ro(
          "inter_body_surface_contact_candidate_builds",
          &sim::DeformableSolverDiagnostics::
              interBodySurfaceContactCandidateBuilds)
      .def_ro(
          "inter_body_surface_contact_candidate_pair_capacity",
          &sim::DeformableSolverDiagnostics::
              interBodySurfaceContactCandidatePairCapacity)
      .def_ro(
          "inter_body_surface_contact_candidate_rejected_pairs",
          &sim::DeformableSolverDiagnostics::
              interBodySurfaceContactCandidateRejectedPairs)
      .def_ro(
          "inter_body_surface_contact_point_triangle_candidates",
          &sim::DeformableSolverDiagnostics::
              interBodySurfaceContactPointTriangleCandidates)
      .def_ro(
          "inter_body_surface_contact_edge_edge_candidates",
          &sim::DeformableSolverDiagnostics::
              interBodySurfaceContactEdgeEdgeCandidates)
      .def_ro(
          "inter_body_surface_contact_ccd_point_triangle_checks",
          &sim::DeformableSolverDiagnostics::
              interBodySurfaceContactCcdPointTriangleChecks)
      .def_ro(
          "inter_body_surface_contact_ccd_edge_edge_checks",
          &sim::DeformableSolverDiagnostics::
              interBodySurfaceContactCcdEdgeEdgeChecks)
      .def_ro(
          "inter_body_surface_contact_ccd_hits",
          &sim::DeformableSolverDiagnostics::interBodySurfaceContactCcdHits)
      .def_ro(
          "inter_body_surface_contact_ccd_limited_steps",
          &sim::DeformableSolverDiagnostics::
              interBodySurfaceContactCcdLimitedSteps)
      .def_ro(
          "inter_body_surface_contact_ccd_zero_step_count",
          &sim::DeformableSolverDiagnostics::
              interBodySurfaceContactCcdZeroStepCount)
      .def_ro(
          "static_rigid_surface_ccd_snapshot_builds",
          &sim::DeformableSolverDiagnostics::
              staticRigidSurfaceCcdSnapshotBuilds)
      .def_ro(
          "static_rigid_surface_ccd_box_count",
          &sim::DeformableSolverDiagnostics::staticRigidSurfaceCcdBoxCount)
      .def_ro(
          "static_rigid_surface_ccd_sphere_count",
          &sim::DeformableSolverDiagnostics::staticRigidSurfaceCcdSphereCount)
      .def_ro(
          "static_rigid_surface_ccd_triangle_count",
          &sim::DeformableSolverDiagnostics::staticRigidSurfaceCcdTriangleCount)
      .def_ro(
          "static_rigid_surface_ccd_edge_count",
          &sim::DeformableSolverDiagnostics::staticRigidSurfaceCcdEdgeCount)
      .def_ro(
          "static_rigid_surface_ccd_candidate_builds",
          &sim::DeformableSolverDiagnostics::
              staticRigidSurfaceCcdCandidateBuilds)
      .def_ro(
          "static_rigid_surface_ccd_candidate_pair_capacity",
          &sim::DeformableSolverDiagnostics::
              staticRigidSurfaceCcdCandidatePairCapacity)
      .def_ro(
          "static_rigid_surface_ccd_candidate_rejected_pairs",
          &sim::DeformableSolverDiagnostics::
              staticRigidSurfaceCcdCandidateRejectedPairs)
      .def_ro(
          "static_rigid_surface_ccd_point_triangle_candidates",
          &sim::DeformableSolverDiagnostics::
              staticRigidSurfaceCcdPointTriangleCandidates)
      .def_ro(
          "static_rigid_surface_ccd_edge_edge_candidates",
          &sim::DeformableSolverDiagnostics::
              staticRigidSurfaceCcdEdgeEdgeCandidates)
      .def_ro(
          "static_rigid_surface_ccd_point_triangle_checks",
          &sim::DeformableSolverDiagnostics::
              staticRigidSurfaceCcdPointTriangleChecks)
      .def_ro(
          "static_rigid_surface_ccd_edge_edge_checks",
          &sim::DeformableSolverDiagnostics::
              staticRigidSurfaceCcdEdgeEdgeChecks)
      .def_ro(
          "static_rigid_surface_ccd_hits",
          &sim::DeformableSolverDiagnostics::staticRigidSurfaceCcdHits)
      .def_ro(
          "static_rigid_surface_ccd_limited_steps",
          &sim::DeformableSolverDiagnostics::staticRigidSurfaceCcdLimitedSteps)
      .def_ro(
          "static_rigid_surface_ccd_zero_step_count",
          &sim::DeformableSolverDiagnostics::staticRigidSurfaceCcdZeroStepCount)
      .def_ro(
          "moving_rigid_surface_ccd_snapshot_builds",
          &sim::DeformableSolverDiagnostics::
              movingRigidSurfaceCcdSnapshotBuilds)
      .def_ro(
          "moving_rigid_surface_ccd_box_count",
          &sim::DeformableSolverDiagnostics::movingRigidSurfaceCcdBoxCount)
      .def_ro(
          "moving_rigid_surface_ccd_sample_count",
          &sim::DeformableSolverDiagnostics::movingRigidSurfaceCcdSampleCount)
      .def_ro(
          "moving_rigid_surface_ccd_inflated_box_count",
          &sim::DeformableSolverDiagnostics::
              movingRigidSurfaceCcdInflatedBoxCount)
      .def_ro(
          "moving_rigid_surface_ccd_triangle_count",
          &sim::DeformableSolverDiagnostics::movingRigidSurfaceCcdTriangleCount)
      .def_ro(
          "moving_rigid_surface_ccd_edge_count",
          &sim::DeformableSolverDiagnostics::movingRigidSurfaceCcdEdgeCount)
      .def_ro(
          "moving_rigid_surface_ccd_candidate_builds",
          &sim::DeformableSolverDiagnostics::
              movingRigidSurfaceCcdCandidateBuilds)
      .def_ro(
          "moving_rigid_surface_ccd_candidate_pair_capacity",
          &sim::DeformableSolverDiagnostics::
              movingRigidSurfaceCcdCandidatePairCapacity)
      .def_ro(
          "moving_rigid_surface_ccd_candidate_rejected_pairs",
          &sim::DeformableSolverDiagnostics::
              movingRigidSurfaceCcdCandidateRejectedPairs)
      .def_ro(
          "moving_rigid_surface_ccd_point_triangle_candidates",
          &sim::DeformableSolverDiagnostics::
              movingRigidSurfaceCcdPointTriangleCandidates)
      .def_ro(
          "moving_rigid_surface_ccd_edge_edge_candidates",
          &sim::DeformableSolverDiagnostics::
              movingRigidSurfaceCcdEdgeEdgeCandidates)
      .def_ro(
          "moving_rigid_surface_ccd_point_triangle_checks",
          &sim::DeformableSolverDiagnostics::
              movingRigidSurfaceCcdPointTriangleChecks)
      .def_ro(
          "moving_rigid_surface_ccd_edge_edge_checks",
          &sim::DeformableSolverDiagnostics::
              movingRigidSurfaceCcdEdgeEdgeChecks)
      .def_ro(
          "moving_rigid_surface_ccd_hits",
          &sim::DeformableSolverDiagnostics::movingRigidSurfaceCcdHits)
      .def_ro(
          "moving_rigid_surface_ccd_limited_steps",
          &sim::DeformableSolverDiagnostics::movingRigidSurfaceCcdLimitedSteps)
      .def_ro(
          "moving_rigid_surface_ccd_zero_step_count",
          &sim::DeformableSolverDiagnostics::movingRigidSurfaceCcdZeroStepCount)
      .def_ro(
          "friction_dissipation",
          &sim::DeformableSolverDiagnostics::frictionDissipation)
      .def_ro(
          "min_active_contact_distance",
          &sim::DeformableSolverDiagnostics::minActiveContactDistance)
      .def_ro(
          "converged_active_contact_count",
          &sim::DeformableSolverDiagnostics::convergedActiveContactCount)
      .def_ro(
          "max_active_contact_count",
          &sim::DeformableSolverDiagnostics::maxActiveContactCount);
}

} // namespace dart::python_nb
