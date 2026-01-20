#include "simulation_experimental/shape_node.hpp"

#include "dart/dynamics/Shape.hpp"
#include "dart/simulation/experimental/frame/frame.hpp"
#include "dart/simulation/experimental/shape/shape_node.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string_view.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defExpShapeNode(nb::module_& m)
{
  using namespace dart::simulation::experimental;

  nb::class_<ShapeNodeOptions>(m, "ShapeNodeOptions")
      .def(nb::init<>())
      .def_rw("relativeTransform", &ShapeNodeOptions::relativeTransform)
      .def_rw("collidable", &ShapeNodeOptions::collidable)
      .def_rw("frictionCoeff", &ShapeNodeOptions::frictionCoeff)
      .def_rw("restitutionCoeff", &ShapeNodeOptions::restitutionCoeff);

  nb::class_<ShapeNode>(m, "ShapeNode")
      .def("getName", &ShapeNode::getName)
      .def("isValid", &ShapeNode::isValid)
      .def("getParentFrame", &ShapeNode::getParentFrame)
      .def("getRelativeTransform", &ShapeNode::getRelativeTransform)
      .def(
          "setRelativeTransform",
          &ShapeNode::setRelativeTransform,
          nb::arg("transform"))
      .def("getWorldTransform", &ShapeNode::getWorldTransform)
      .def("getShape", &ShapeNode::getShape)
      .def("setShape", &ShapeNode::setShape, nb::arg("shape"))
      .def("setCollidable", &ShapeNode::setCollidable, nb::arg("collidable"))
      .def("isCollidable", &ShapeNode::isCollidable)
      .def(
          "setFrictionCoeff", &ShapeNode::setFrictionCoeff, nb::arg("friction"))
      .def("getFrictionCoeff", &ShapeNode::getFrictionCoeff)
      .def(
          "setRestitutionCoeff",
          &ShapeNode::setRestitutionCoeff,
          nb::arg("restitution"))
      .def("getRestitutionCoeff", &ShapeNode::getRestitutionCoeff);
}

} // namespace dart::python_nb
