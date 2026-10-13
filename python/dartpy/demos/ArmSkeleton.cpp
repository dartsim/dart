// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

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

#include "demo_models/ssik_ik_gui/ArmSkeleton.hpp"

#include "detail/eigen.hpp"

#include <nanobind/stl/vector.h>

#include <memory>

#include <cmath>

namespace dart::python {
namespace {

namespace ssik_model = dart_demos::ssik_ik_gui;

void requireSsikTransform(const Eigen::Matrix4d& matrix)
{
  const Eigen::Matrix3d rotation = matrix.topLeftCorner<3, 3>();
  if (!matrix.allFinite()
      || !matrix.row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-8)
      || !(rotation.transpose() * rotation)
              .isApprox(Eigen::Matrix3d::Identity(), 1e-8)
      || std::abs(rotation.determinant() - 1.0) > 1e-8)
    throw nb::value_error("transform must be a finite rigid 4x4 matrix");
}

struct DemoArmSkeleton
{
  nb::object worldWrapper;
  std::unique_ptr<ssik_model::ArmSkeleton> native;

  DemoArmSkeleton(nb::object world) : worldWrapper(std::move(world))
  {
    if (worldWrapper.is_none())
      throw nb::value_error("world must not be None");
    if (!nb::isinstance<dart::simulation::World>(worldWrapper))
      throw nb::type_error("world must be a World");
    native = std::make_unique<ssik_model::ArmSkeleton>();
    native->setWorld(nb::cast<dart::simulation::WorldPtr>(worldWrapper));
  }

  ssik_model::ArmSkeleton& get() const
  {
    if (!native)
      throw nb::value_error("arm has been cleared");
    return *native;
  }

  void clear()
  {
    native.reset();
    worldWrapper.reset();
  }
};

int armSkeletonTraverse(PyObject* self, visitproc visit, void* arg) noexcept
{
  Py_VISIT(Py_TYPE(self));
  if (!nb::inst_ready(self))
    return 0;
  try {
    auto* owner = nb::inst_ptr<DemoArmSkeleton>(self);
    Py_VISIT(owner->worldWrapper.ptr());
    dartnb::GcEdges edges;
    if (owner->native)
      edges.add(owner->native->getWorld(), [] {});
    return edges.traverse(visit, arg);
  } catch (...) {
    PyErr_SetString(PyExc_RuntimeError, "arm skeleton GC traversal failed");
    return -1;
  }
}

int armSkeletonClear(PyObject* self) noexcept
{
  if (nb::inst_ready(self))
    nb::inst_ptr<DemoArmSkeleton>(self)->clear();
  return 0;
}

} // namespace
} // namespace dart::python

namespace dartnb {
template <>
const PyType_Slot* gc_slots<dart::python::DemoArmSkeleton>()
{
  static const PyType_Slot slots[] = {
      {Py_tp_traverse,
       reinterpret_cast<void*>(&dart::python::armSkeletonTraverse)},
      {Py_tp_clear, reinterpret_cast<void*>(&dart::python::armSkeletonClear)},
      {0, nullptr}};
  return slots;
}
} // namespace dartnb

namespace dart::python {

void bind_arm_skeleton(nb::module_& demos)
{
  nb::class_<ssik_model::ArmSpec>(demos, "ArmSpec", nb::is_final())
      .def_ro("label", &ssik_model::ArmSpec::label)
      .def_ro("module", &ssik_model::ArmSpec::module);
  demos.def("get_ssik_arm_specs", &ssik_model::getArmSpecs);

  nb::class_<ssik_model::JointSpec>(demos, "JointSpec", nb::is_final())
      .def(
          "__init__",
          [](ssik_model::JointSpec* self,
             const Eigen::Matrix4d& left,
             const Eigen::Matrix4d& right,
             const Eigen::Vector3d& axis,
             bool prismatic) {
            requireSsikTransform(left);
            requireSsikTransform(right);
            if (!axis.allFinite() || !std::isfinite(axis.norm())
                || axis.norm() <= 1e-9)
              throw nb::value_error("axis must be finite and nonzero");
            new (self) ssik_model::JointSpec{
                Eigen::Isometry3d(left),
                Eigen::Isometry3d(right),
                axis,
                prismatic};
          },
          nb::arg("t_left"),
          nb::arg("t_right"),
          nb::arg("axis"),
          nb::kw_only(),
          nb::arg("prismatic") = false)
      .def_prop_ro(
          "t_left",
          [](const ssik_model::JointSpec& spec) -> Eigen::Matrix4d {
            return spec.tLeft.matrix();
          })
      .def_prop_ro(
          "t_right",
          [](const ssik_model::JointSpec& spec) -> Eigen::Matrix4d {
            return spec.tRight.matrix();
          })
      .def_prop_ro(
          "axis",
          [](const ssik_model::JointSpec& spec) -> Eigen::Vector3d {
            return spec.axis;
          })
      .def_ro("prismatic", &ssik_model::JointSpec::prismatic);

  dartnb::dart_class<DemoArmSkeleton>(
      demos, "ArmSkeleton", "Native ssik arm construction and ghost solutions.")
      .def(nb::init<nb::object>(), nb::arg("world").none())
      .def_prop_ro(
          "world",
          [](const DemoArmSkeleton& self) { return self.worldWrapper; })
      .def(
          "build",
          [](DemoArmSkeleton& self,
             const std::vector<ssik_model::JointSpec>& chain) {
            self.get().build(chain);
          },
          nb::arg("chain"))
      .def(
          "show_solutions",
          [](DemoArmSkeleton& self,
             const std::vector<Eigen::VectorXd>& configs,
             int selected) {
            if (selected < -1
                || (!configs.empty() && selected >= 0
                    && static_cast<std::size_t>(selected) >= configs.size())
                || (configs.empty() && selected > 0))
              throw nb::value_error("selected solution is out of range");
            std::vector<ssik_model::SsikSolution> solutions;
            solutions.reserve(configs.size());
            const auto dofs = static_cast<Eigen::Index>(self.get().numDofs());
            for (const auto& q : configs) {
              if (!q.allFinite() || (dofs != 0 && q.size() != dofs))
                throw nb::value_error(
                    "solution must be finite and match the arm DOFs");
              ssik_model::SsikSolution solution;
              solution.q = q;
              solutions.push_back(std::move(solution));
            }
            self.get().showSolutions(solutions, selected);
          },
          nb::arg("configs"),
          nb::arg("selected") = 0);

  demos.def(
      "make_ssik_target_transform",
      [](const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy) {
        if (!xyz.allFinite() || !rpy.allFinite()
            || !xyz.cast<float>().allFinite() || !rpy.cast<float>().allFinite())
          throw nb::value_error("target controls must be finite floats");
        return ssik_model::makeTargetTransform(
            {{static_cast<float>(xyz[0]),
              static_cast<float>(xyz[1]),
              static_cast<float>(xyz[2])}},
            {{static_cast<float>(rpy[0]),
              static_cast<float>(rpy[1]),
              static_cast<float>(rpy[2])}});
      },
      nb::arg("xyz"),
      nb::arg("rpy_degrees"));
  demos.def(
      "ssik_target_controls",
      [](const Eigen::Isometry3d& transform) {
        requireSsikTransform(transform.matrix());
        if (!transform.translation().cast<float>().allFinite())
          throw nb::value_error("target controls must be finite floats");
        std::array<float, 3> xyz;
        std::array<float, 3> rpy;
        ssik_model::targetControlsFromTransform(transform, xyz, rpy);
        return nb::make_tuple(
            Eigen::Vector3d(xyz[0], xyz[1], xyz[2]),
            Eigen::Vector3d(rpy[0], rpy[1], rpy[2]));
      },
      nb::arg("transform"));
}

} // namespace dart::python
