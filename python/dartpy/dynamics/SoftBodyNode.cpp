/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 * See LICENSE for the BSD-style license governing this file.
 */

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"
#include "pointers.hpp"

#include <dart/dynamics/SoftBodyNode.hpp>

#include <pybind11/pybind11.h>

#include <cmath>

namespace py = pybind11;

namespace dart {
namespace python {

void SoftBodyNode(py::module& m)
{
  using SoftBody = dart::dynamics::SoftBodyNode;
  using Helper = dart::dynamics::SoftBodyNodeHelper;

  py::class_<SoftBody::UniqueProperties>(m, "SoftBodyNodeUniqueProperties")
      .def(py::init<>())
      .def_readwrite("mKv", &SoftBody::UniqueProperties::mKv)
      .def_readwrite("mKe", &SoftBody::UniqueProperties::mKe)
      .def_readwrite("mDampCoeff", &SoftBody::UniqueProperties::mDampCoeff);

  py::class_<SoftBody::Properties>(m, "SoftBodyNodeProperties")
      .def(
          py::init<
              const dart::dynamics::BodyNode::Properties&,
              const SoftBody::UniqueProperties&>(),
          py::arg("bodyProperties"),
          py::arg("softProperties"));

  py::class_<
      SoftBody,
      dart::dynamics::BodyNode,
      dart::dynamics::SoftBodyNodePtr>(m, "SoftBodyNode")
      .def("getNumPointMasses", &SoftBody::getNumPointMasses)
      .def("getMass", &SoftBody::getMass)
      .def("getVertexSpringStiffness", &SoftBody::getVertexSpringStiffness)
      .def("getEdgeSpringStiffness", &SoftBody::getEdgeSpringStiffness)
      .def("getDampingCoefficient", &SoftBody::getDampingCoefficient);

  py::class_<Helper>(m, "SoftBodyNodeHelper")
      .def_static(
          "makeBoxProperties",
          [](const Eigen::Vector3d& size,
             const Eigen::Isometry3d& localTransform,
             const Eigen::Vector3i& frags,
             double mass) {
            if (!size.allFinite() || (size.array() <= 0).any()
                || !localTransform.matrix().allFinite() || !std::isfinite(mass)
                || mass <= 0)
              throw py::value_error(
                  "size and mass must be finite and positive; transform must "
                  "be finite");
            return Helper::makeBoxProperties(size, localTransform, frags, mass);
          },
          py::arg("size"),
          py::arg("localTransform"),
          py::arg("frags"),
          py::arg("mass"))
      .def_static(
          "makeEllipsoidProperties",
          [](const Eigen::Vector3d& size, int slices, int stacks, double mass) {
            if (!size.allFinite() || (size.array() <= 0).any()
                || !std::isfinite(mass) || mass <= 0)
              throw py::value_error(
                  "size and mass must be finite and positive");
            if (slices < 3 || stacks < 2)
              throw py::value_error("slices must be >= 3 and stacks >= 2");
            return Helper::makeEllipsoidProperties(size, slices, stacks, mass);
          },
          py::arg("size"),
          py::arg("slices"),
          py::arg("stacks"),
          py::arg("mass"))
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
              throw py::value_error(
                  "radius, height, and mass must be finite and positive");
            if (slices < 3 || stacks < 2 || rings < 1)
              throw py::value_error(
                  "slices must be >= 3, stacks >= 2, and rings >= 1");
            return Helper::makeCylinderProperties(
                radius, height, slices, stacks, rings, mass);
          },
          py::arg("radius"),
          py::arg("height"),
          py::arg("slices"),
          py::arg("stacks"),
          py::arg("rings"),
          py::arg("mass"));
}

} // namespace python
} // namespace dart
