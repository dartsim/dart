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

// core enums, CollisionShape, StateSpace, JointSpec, policy wrappers
void defSimPartCore(nb::module_& m)
{
  nb::enum_<sim::JointType>(m, "JointType")
      .value("FIXED", sim::JointType::Fixed)
      .value("REVOLUTE", sim::JointType::Revolute)
      .value("PRISMATIC", sim::JointType::Prismatic)
      .value("SCREW", sim::JointType::Screw)
      .value("UNIVERSAL", sim::JointType::Universal)
      .value("SPHERICAL", sim::JointType::Spherical)
      .value("PLANAR", sim::JointType::Planar)
      .value("FLOATING", sim::JointType::Floating)
      .value("CUSTOM", sim::JointType::Custom);

  nb::enum_<sim::ActuatorType>(m, "ActuatorType")
      .value("FORCE", sim::ActuatorType::Force)
      .value("PASSIVE", sim::ActuatorType::Passive)
      .value("SERVO", sim::ActuatorType::Servo)
      .value("VELOCITY", sim::ActuatorType::Velocity)
      .value("ACCELERATION", sim::ActuatorType::Acceleration)
      .value("LOCKED", sim::ActuatorType::Locked)
      .value("MIMIC", sim::ActuatorType::Mimic);

  nb::enum_<sim::LoopClosureFamily>(m, "LoopClosureFamily")
      .value("RIGID", sim::LoopClosureFamily::Rigid)
      .value("POINT", sim::LoopClosureFamily::Point)
      .value("DISTANCE", sim::LoopClosureFamily::Distance);

  nb::enum_<sim::LoopClosureResidualCoordinates>(
      m, "LoopClosureResidualCoordinates")
      .value("WORLD", sim::LoopClosureResidualCoordinates::World);

  nb::enum_<sim::WorldSyncStage>(m, "WorldSyncStage")
      .value("KINEMATICS", sim::WorldSyncStage::Kinematics);

  nb::enum_<sim::RigidBodySolver>(m, "RigidBodySolver")
      .value("SEQUENTIAL_IMPULSE", sim::RigidBodySolver::SequentialImpulse)
      .value("IPC", sim::RigidBodySolver::Ipc);
  nb::enum_<sim::ContactSolverMethod>(m, "ContactSolverMethod")
      .value("SEQUENTIAL_IMPULSE", sim::ContactSolverMethod::SequentialImpulse)
      .value("BOXED_LCP", sim::ContactSolverMethod::BoxedLcp);
  nb::enum_<sim::MultibodyIntegrationFamily>(m, "MultibodyIntegrationFamily")
      .value("SEMI_IMPLICIT", sim::MultibodyIntegrationFamily::SemiImplicit)
      .value("VARIATIONAL", sim::MultibodyIntegrationFamily::Variational);

  nb::enum_<sim::ContactGradientMode>(m, "ContactGradientMode")
      .value("ANALYTIC", sim::ContactGradientMode::Analytic)
      .value(
          "COMPLEMENTARITY_AWARE",
          sim::ContactGradientMode::ComplementarityAware)
      .value(
          "PRE_CONTACT_SURROGATE",
          sim::ContactGradientMode::PreContactSurrogate);
  nb::enum_<sim::ComputeAcceleratorPolicy>(m, "ComputeAcceleratorPolicy")
      .value("CPU_ONLY", sim::ComputeAcceleratorPolicy::CpuOnly)
      .value(
          "PREFER_ACCELERATED",
          sim::ComputeAcceleratorPolicy::PreferAccelerated);

  nb::enum_<sim::PhysicalParameter>(m, "PhysicalParameter")
      .value("MASS", sim::PhysicalParameter::MASS)
      .value("CENTER_OF_MASS", sim::PhysicalParameter::CENTER_OF_MASS)
      .value("INERTIA", sim::PhysicalParameter::INERTIA)
      .value("FRICTION", sim::PhysicalParameter::FRICTION);

  nb::enum_<sim::CollisionShapeType>(m, "CollisionShapeType")
      .value("SPHERE", sim::CollisionShapeType::Sphere)
      .value("BOX", sim::CollisionShapeType::Box)
      .value("MESH", sim::CollisionShapeType::Mesh)
      .value("CAPSULE", sim::CollisionShapeType::Capsule)
      .value("CYLINDER", sim::CollisionShapeType::Cylinder)
      .value("PLANE", sim::CollisionShapeType::Plane);

  nb::class_<sim::CollisionShape>(m, "CollisionShape")
      .def_static(
          "sphere",
          [](double radius, const nb::handle& localTransform) {
            sim::CollisionShape shape = sim::CollisionShape::makeSphere(radius);
            if (!localTransform.is_none()) {
              shape.localTransform = toIsometry(localTransform);
            }
            return shape;
          },
          nb::arg("radius"),
          nb::arg("local_transform") = nb::none())
      .def_static(
          "box",
          [](const nb::handle& halfExtents, const nb::handle& localTransform) {
            sim::CollisionShape shape
                = sim::CollisionShape::makeBox(toVector3(halfExtents));
            if (!localTransform.is_none()) {
              shape.localTransform = toIsometry(localTransform);
            }
            return shape;
          },
          nb::arg("half_extents"),
          nb::arg("local_transform") = nb::none())
      .def_static(
          "capsule",
          [](double radius,
             double halfHeight,
             const nb::handle& localTransform) {
            sim::CollisionShape shape
                = sim::CollisionShape::makeCapsule(radius, halfHeight);
            if (!localTransform.is_none()) {
              shape.localTransform = toIsometry(localTransform);
            }
            return shape;
          },
          nb::arg("radius"),
          nb::arg("half_height"),
          nb::arg("local_transform") = nb::none())
      .def_static(
          "cylinder",
          [](double radius,
             double halfHeight,
             const nb::handle& localTransform) {
            sim::CollisionShape shape
                = sim::CollisionShape::makeCylinder(radius, halfHeight);
            if (!localTransform.is_none()) {
              shape.localTransform = toIsometry(localTransform);
            }
            return shape;
          },
          nb::arg("radius"),
          nb::arg("half_height"),
          nb::arg("local_transform") = nb::none())
      .def_static(
          "plane",
          [](const nb::handle& normal,
             double offset,
             const nb::handle& localTransform) {
            sim::CollisionShape shape
                = sim::CollisionShape::makePlane(toVector3(normal), offset);
            if (!localTransform.is_none()) {
              shape.localTransform = toIsometry(localTransform);
            }
            return shape;
          },
          nb::arg("normal"),
          nb::arg("offset"),
          nb::arg("local_transform") = nb::none())
      .def_static(
          "mesh",
          [](const nb::handle& vertices,
             const nb::handle& triangles,
             const nb::handle& localTransform) {
            sim::CollisionShape shape = sim::CollisionShape::makeMesh(
                toVector3List(vertices), toTriangleList(triangles));
            if (!localTransform.is_none()) {
              shape.localTransform = toIsometry(localTransform);
            }
            return shape;
          },
          nb::arg("vertices"),
          nb::arg("triangles"),
          nb::arg("local_transform") = nb::none())
      .def_prop_ro(
          "type", [](const sim::CollisionShape& self) { return self.type; })
      .def_prop_ro(
          "radius", [](const sim::CollisionShape& self) { return self.radius; })
      .def_prop_ro(
          "height",
          [](const sim::CollisionShape& self) {
            return 2.0 * self.halfExtents.z();
          })
      .def_prop_ro(
          "half_height",
          [](const sim::CollisionShape& self) { return self.halfExtents.z(); })
      .def_prop_ro(
          "half_extents",
          [](const sim::CollisionShape& self) { return self.halfExtents; })
      .def_prop_ro(
          "normal", [](const sim::CollisionShape& self) { return self.normal; })
      .def_prop_ro(
          "offset", [](const sim::CollisionShape& self) { return self.offset; })
      .def_prop_ro(
          "vertices",
          [](const sim::CollisionShape& self) { return self.vertices; })
      .def_prop_ro(
          "triangles",
          [](const sim::CollisionShape& self) { return self.triangles; })
      .def_prop_ro("local_transform", [](const sim::CollisionShape& self) {
        return self.localTransform.matrix();
      });

  nb::enum_<sim::ClosureKinematicsPolicy>(m, "ClosureKinematicsPolicy")
      .value("RESIDUAL_ONLY", sim::ClosureKinematicsPolicy::ResidualOnly)
      .value("PROJECT", sim::ClosureKinematicsPolicy::Project);

  nb::enum_<sim::ClosureDynamicsPolicy>(m, "ClosureDynamicsPolicy")
      .value("RESIDUAL_ONLY", sim::ClosureDynamicsPolicy::ResidualOnly)
      .value("SOLVE", sim::ClosureDynamicsPolicy::Solve);

  nb::class_<sim::StateSpace::Variable>(m, "StateVariable")
      .def_prop_ro(
          "name",
          [](const sim::StateSpace::Variable& self) { return self.name; })
      .def_prop_ro(
          "start_index",
          [](const sim::StateSpace::Variable& self) { return self.startIndex; })
      .def_prop_ro(
          "dimension",
          [](const sim::StateSpace::Variable& self) { return self.dimension; })
      .def_prop_ro(
          "lower_bound",
          [](const sim::StateSpace::Variable& self) { return self.lowerBound; })
      .def_prop_ro(
          "upper_bound",
          [](const sim::StateSpace::Variable& self) { return self.upperBound; })
      .def("__repr__", [](const sim::StateSpace::Variable& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("name", repr_string(self.name));
        fields.emplace_back("start_index", std::to_string(self.startIndex));
        fields.emplace_back("dimension", std::to_string(self.dimension));
        fields.emplace_back("lower_bound", repr_double(self.lowerBound));
        fields.emplace_back("upper_bound", repr_double(self.upperBound));
        return format_repr("StateVariable", fields);
      });

  nb::class_<sim::StateSpace>(m, "StateSpace")
      .def(nb::init<>())
      .def(
          "add_variable",
          [](sim::StateSpace& self,
             const std::string& name,
             std::size_t dimension,
             double lower,
             double upper) -> sim::StateSpace& {
            return self.addVariable(name, dimension, lower, upper);
          },
          nb::arg("name"),
          nb::arg("dimension"),
          nb::kw_only(),
          nb::arg("lower") = -std::numeric_limits<double>::infinity(),
          nb::arg("upper") = std::numeric_limits<double>::infinity(),
          nb::rv_policy::reference_internal)
      .def(
          "add_variables",
          [](sim::StateSpace& self,
             const std::vector<std::string>& names,
             double lower,
             double upper) -> sim::StateSpace& {
            return self.addVariables(
                std::span<const std::string>(names.data(), names.size()),
                lower,
                upper);
          },
          nb::arg("names"),
          nb::kw_only(),
          nb::arg("lower") = -std::numeric_limits<double>::infinity(),
          nb::arg("upper") = std::numeric_limits<double>::infinity(),
          nb::rv_policy::reference_internal)
      .def("finalize", &sim::StateSpace::finalize)
      .def("has_variable", &sim::StateSpace::hasVariable, nb::arg("name"))
      .def(
          "get_variable",
          [](const sim::StateSpace& self,
             const std::string& name) -> nb::object {
            auto variable = self.getVariable(name);
            if (!variable.has_value()) {
              return nb::none();
            }
            return nb::cast(*variable, nb::rv_policy::move);
          },
          nb::arg("name"))
      .def(
          "get_variable_index",
          [](const sim::StateSpace& self,
             const std::string& name) -> nb::object {
            auto index = self.getVariableIndex(name);
            if (!index.has_value()) {
              return nb::none();
            }
            return nb::cast(*index);
          },
          nb::arg("name"))
      .def_prop_ro("dimension", &sim::StateSpace::getDimension)
      .def_prop_ro("num_variables", &sim::StateSpace::getNumVariables)
      .def_prop_ro("is_finalized", &sim::StateSpace::isFinalized)
      .def_prop_ro("variables", &sim::StateSpace::getVariables)
      .def_prop_ro("variable_names", &sim::StateSpace::getVariableNames)
      .def_prop_ro(
          "lower_bounds",
          [](const sim::StateSpace& self) {
            return toVectorX(self.getLowerBounds());
          })
      .def_prop_ro(
          "upper_bounds",
          [](const sim::StateSpace& self) {
            return toVectorX(self.getUpperBounds());
          })
      .def("__repr__", [](const sim::StateSpace& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("dimension", std::to_string(self.getDimension()));
        fields.emplace_back(
            "variables", std::to_string(self.getNumVariables()));
        fields.emplace_back("finalized", self.isFinalized() ? "True" : "False");
        return format_repr("StateSpace", fields);
      });

  nb::class_<sim::JointSpec>(m, "JointSpec")
      .def(
          nb::new_([](std::string name,
                      sim::JointType type,
                      const nb::handle& axis,
                      const nb::handle& axis2,
                      const nb::handle& transform_from_parent,
                      const nb::handle& transform_to_parent,
                      const nb::handle& parent_anchor,
                      const nb::handle& child_anchor) {
            sim::JointSpec spec;
            spec.name = std::move(name);
            spec.type = type;
            if (!axis.is_none()) {
              spec.axis = toVector3(axis);
              validateJointSpecAxis(spec.axis);
            }
            if (!axis2.is_none()) {
              spec.axis2 = toVector3(axis2);
              validateJointSpecAxis(spec.axis2, "axis2");
            }
            if (!transform_from_parent.is_none()) {
              spec.transformFromParent = toIsometry(transform_from_parent);
            }
            if (!transform_to_parent.is_none()) {
              spec.transformToParent = toIsometry(transform_to_parent);
            }
            setJointSpecAnchor(
                spec.parentAnchor, parent_anchor, "parent_anchor");
            setJointSpecAnchor(spec.childAnchor, child_anchor, "child_anchor");
            return spec;
          }),
          nb::arg("name") = "",
          nb::arg("type") = sim::JointType::Revolute,
          nb::arg("axis") = nb::none(),
          nb::arg("axis2") = nb::none(),
          nb::arg("transform_from_parent") = nb::none(),
          nb::arg("transform_to_parent") = nb::none(),
          nb::arg("parent_anchor") = nb::none(),
          nb::arg("child_anchor") = nb::none())
      .def_rw("name", &sim::JointSpec::name)
      .def_rw("type", &sim::JointSpec::type)
      .def_prop_rw(
          "axis",
          [](const sim::JointSpec& self) { return self.axis; },
          [](sim::JointSpec& self, const nb::handle& axis) {
            auto value = toVector3(axis);
            validateJointSpecAxis(value);
            self.axis = value;
          })
      .def_prop_rw(
          "axis2",
          [](const sim::JointSpec& self) { return self.axis2; },
          [](sim::JointSpec& self, const nb::handle& axis2) {
            auto value = toVector3(axis2);
            validateJointSpecAxis(value, "axis2");
            self.axis2 = value;
          })
      .def_prop_rw(
          "transform_from_parent",
          [](const sim::JointSpec& self) {
            return self.transformFromParent.matrix();
          },
          [](sim::JointSpec& self, const nb::handle& transform) {
            self.transformFromParent = toIsometry(transform);
          })
      .def_prop_rw(
          "transform_to_parent",
          [](const sim::JointSpec& self) {
            return self.transformToParent.matrix();
          },
          [](sim::JointSpec& self, const nb::handle& transform) {
            self.transformToParent = toIsometry(transform);
          })
      .def_prop_rw(
          "parent_anchor",
          [](const sim::JointSpec& self) -> nb::object {
            if (!self.parentAnchor.has_value()) {
              return nb::none();
            }
            return nb::cast(*self.parentAnchor);
          },
          [](sim::JointSpec& self, const nb::handle& anchor) {
            setJointSpecAnchor(self.parentAnchor, anchor, "parent_anchor");
          })
      .def_prop_rw(
          "child_anchor",
          [](const sim::JointSpec& self) -> nb::object {
            if (!self.childAnchor.has_value()) {
              return nb::none();
            }
            return nb::cast(*self.childAnchor);
          },
          [](sim::JointSpec& self, const nb::handle& anchor) {
            setJointSpecAnchor(self.childAnchor, anchor, "child_anchor");
          })
      .def("__repr__", [](const sim::JointSpec& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("name", repr_string(self.name));
        fields.emplace_back(
            "type", nb::cast<std::string>(nb::repr(nb::cast(self.type))));
        fields.emplace_back(
            "axis", nb::cast<std::string>(nb::repr(nb::cast(self.axis))));
        return format_repr("JointSpec", fields);
      });

  nb::class_<PyJointConstraintProjectionPolicy>(
      m, "JointConstraintProjectionPolicy")
      .def(
          nb::new_([](double startStiffness,
                      double linearStiffness,
                      double angularStiffness) {
            return PyJointConstraintProjectionPolicy(
                sim::JointConstraintProjectionPolicy{
                    .startStiffness = startStiffness,
                    .linearStiffness = linearStiffness,
                    .angularStiffness = angularStiffness});
          }),
          nb::arg("start_stiffness") = 1.0,
          nb::arg("linear_stiffness") = std::numeric_limits<double>::infinity(),
          nb::arg("angular_stiffness")
          = std::numeric_limits<double>::infinity())
      .def_prop_rw(
          "start_stiffness",
          &PyJointConstraintProjectionPolicy::getStartStiffness,
          &PyJointConstraintProjectionPolicy::setStartStiffness)
      .def_prop_rw(
          "linear_stiffness",
          &PyJointConstraintProjectionPolicy::getLinearStiffness,
          &PyJointConstraintProjectionPolicy::setLinearStiffness)
      .def_prop_rw(
          "angular_stiffness",
          &PyJointConstraintProjectionPolicy::getAngularStiffness,
          &PyJointConstraintProjectionPolicy::setAngularStiffness)
      .def("__repr__", [](const PyJointConstraintProjectionPolicy& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back(
            "start_stiffness", repr_double(self.getStartStiffness()));
        fields.emplace_back(
            "linear_stiffness", repr_double(self.getLinearStiffness()));
        fields.emplace_back(
            "angular_stiffness", repr_double(self.getAngularStiffness()));
        return format_repr("JointConstraintProjectionPolicy", fields);
      });

  nb::class_<PyDeformableObstaclePolicy>(m, "DeformableObstaclePolicy")
      .def(
          nb::new_(
              [](bool groundBarrier, bool surfaceObstacle, bool barrierOnly) {
                return PyDeformableObstaclePolicy(
                    sim::DeformableObstaclePolicy{
                        .groundBarrier = groundBarrier,
                        .surfaceObstacle = surfaceObstacle,
                        .barrierOnly = barrierOnly});
              }),
          nb::arg("ground_barrier") = false,
          nb::arg("surface_obstacle") = false,
          nb::arg("barrier_only") = false)
      .def_prop_rw(
          "ground_barrier",
          &PyDeformableObstaclePolicy::getGroundBarrier,
          &PyDeformableObstaclePolicy::setGroundBarrier)
      .def_prop_rw(
          "surface_obstacle",
          &PyDeformableObstaclePolicy::getSurfaceObstacle,
          &PyDeformableObstaclePolicy::setSurfaceObstacle)
      .def_prop_rw(
          "barrier_only",
          &PyDeformableObstaclePolicy::getBarrierOnly,
          &PyDeformableObstaclePolicy::setBarrierOnly)
      .def("__repr__", [](const PyDeformableObstaclePolicy& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back(
            "ground_barrier", repr_bool(self.getGroundBarrier()));
        fields.emplace_back(
            "surface_obstacle", repr_bool(self.getSurfaceObstacle()));
        fields.emplace_back("barrier_only", repr_bool(self.getBarrierOnly()));
        return format_repr("DeformableObstaclePolicy", fields);
      });
}

} // namespace dart::python_nb
