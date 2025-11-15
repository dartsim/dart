#include "dynamics/shape_node.hpp"

#include "dart/dynamics/ShapeNode.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defShapeNode(nb::module_& m)
{
  using ShapeNode = dart::dynamics::ShapeNode;

  nb::class_<ShapeNode, dart::dynamics::ShapeFrame>(m, "ShapeNode")
      .def(
          "setRelativeTransform",
          &ShapeNode::setRelativeTransform,
          nb::arg("transform"))
      .def(
          "setRelativeTranslation",
          &ShapeNode::setRelativeTranslation,
          nb::arg("translation"))
      .def("getRelativeTranslation", &ShapeNode::getRelativeTranslation)
      .def(
          "setRelativeRotation",
          &ShapeNode::setRelativeRotation,
          nb::arg("rotation"))
      .def("getRelativeRotation", &ShapeNode::getRelativeRotation)
      .def("setOffset", &ShapeNode::setOffset, nb::arg("offset"))
      .def("getOffset", &ShapeNode::getOffset);
}

} // namespace dart::python_nb
