// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 * See LICENSE for the BSD-style license governing this file.
 */

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"
#include "pointers.hpp"

#include <dart/dynamics/SoftBodyNode.hpp>

#include <cmath>

namespace dart {
namespace python {

void SoftBodyNode(nb::module_& m)
{
  using SoftBody = dart::dynamics::SoftBodyNode;
  using Helper = dart::dynamics::SoftBodyNodeHelper;

  dartnb::dart_class<SoftBody::UniqueProperties>(
      m, "SoftBodyNodeUniqueProperties")
      .def(nb::init<>())
      .def_rw("mKv", &SoftBody::UniqueProperties::mKv)
      .def_rw("mKe", &SoftBody::UniqueProperties::mKe)
      .def_rw("mDampCoeff", &SoftBody::UniqueProperties::mDampCoeff);

  dartnb::dart_class<SoftBody::Properties>(m, "SoftBodyNodeProperties")
      .def(
          nb::init<
              const dart::dynamics::BodyNode::Properties&,
              const SoftBody::UniqueProperties&>(),
          nb::arg("bodyProperties"),
          nb::arg("softProperties"));

  dartnb::dart_class<SoftBody, dart::dynamics::BodyNode>(m, "SoftBodyNode")
      .def("getNumPointMasses", &SoftBody::getNumPointMasses)
      .def("getMass", &SoftBody::getMass)
      .def("getVertexSpringStiffness", &SoftBody::getVertexSpringStiffness)
      .def("getEdgeSpringStiffness", &SoftBody::getEdgeSpringStiffness)
      .def("getDampingCoefficient", &SoftBody::getDampingCoefficient);

  dartnb::dart_class<Helper>(m, "SoftBodyNodeHelper")
      .def_static(
          "makeBoxProperties",
          [](const Eigen::Vector3d& size,
             const Eigen::Isometry3d& localTransform,
             const Eigen::Vector3i& frags,
             double mass) {
            if (!size.allFinite() || (size.array() <= 0).any()
                || !localTransform.matrix().allFinite() || !std::isfinite(mass)
                || mass <= 0)
              throw nb::value_error(
                  "size and mass must be finite and positive; transform must "
                  "be finite");
            return Helper::makeBoxProperties(size, localTransform, frags, mass);
          },
          nb::arg("size"),
          nb::arg("localTransform"),
          nb::arg("frags"),
          nb::arg("mass"))
      .def_static(
          "makeEllipsoidProperties",
          [](const Eigen::Vector3d& size, int slices, int stacks, double mass) {
            if (!size.allFinite() || (size.array() <= 0).any()
                || !std::isfinite(mass) || mass <= 0)
              throw nb::value_error(
                  "size and mass must be finite and positive");
            if (slices < 3 || stacks < 2)
              throw nb::value_error("slices must be >= 3 and stacks >= 2");
            return Helper::makeEllipsoidProperties(size, slices, stacks, mass);
          },
          nb::arg("size"),
          nb::arg("slices"),
          nb::arg("stacks"),
          nb::arg("mass"))
      .def_static(
          "makeCylinderProperties",
          [](double radius,
             double height,
             int slices,
             int stacks,
             int rings,
             double mass) {
            if (!std::isfinite(radius) || radius <= 0 || !std::isfinite(height)
                || height <= 0 || !std::isfinite(mass) || mass <= 0)
              throw nb::value_error(
                  "radius, height, and mass must be finite and positive");
            if (slices < 3 || stacks < 2 || rings < 1)
              throw nb::value_error(
                  "slices must be >= 3, stacks >= 2, and rings >= 1");
            return Helper::makeCylinderProperties(
                radius, height, slices, stacks, rings, mass);
          },
          nb::arg("radius"),
          nb::arg("height"),
          nb::arg("slices"),
          nb::arg("stacks"),
          nb::arg("rings"),
          nb::arg("mass"));
}

} // namespace python
} // namespace dart
