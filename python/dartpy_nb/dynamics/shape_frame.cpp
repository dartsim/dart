#include "dynamics/shape_frame.hpp"

#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/ShapeNode.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defShapeFrame(nb::module_& m)
{
  using ShapeFrame = dart::dynamics::ShapeFrame;
  using ShapeNode = dart::dynamics::ShapeNode;
  using VisualAspect = dart::dynamics::VisualAspect;
  using CollisionAspect = dart::dynamics::CollisionAspect;
  using DynamicsAspect = dart::dynamics::DynamicsAspect;

  nb::class_<VisualAspect>(m, "VisualAspect")
      .def(nb::init<>())
      .def("setRGBA", &VisualAspect::setRGBA, nb::arg("color"))
      .def("getRGBA", &VisualAspect::getRGBA, nb::rv_policy::reference_internal)
      .def("setHidden", &VisualAspect::setHidden, nb::arg("value"))
      .def("getHidden", &VisualAspect::getHidden)
      .def("setShadowed", &VisualAspect::setShadowed, nb::arg("value"))
      .def("getShadowed", &VisualAspect::getShadowed);

  nb::class_<CollisionAspect>(m, "CollisionAspect")
      .def(nb::init<>())
      .def("setCollidable", &CollisionAspect::setCollidable, nb::arg("value"))
      .def("getCollidable", &CollisionAspect::getCollidable);

  nb::class_<DynamicsAspect>(m, "DynamicsAspect")
      .def(nb::init<>())
      .def(
          "setFrictionCoeff",
          &DynamicsAspect::setFrictionCoeff,
          nb::arg("value"))
      .def("getFrictionCoeff", &DynamicsAspect::getFrictionCoeff);

  nb::class_<ShapeFrame, dart::dynamics::Frame>(m, "ShapeFrame")
      .def(
          "setShape",
          [](ShapeFrame& self, const dart::dynamics::ShapePtr& shape) {
            self.setShape(shape);
          },
          nb::arg("shape"))
      .def(
          "getShape",
          [](ShapeFrame& self) -> dart::dynamics::ShapePtr {
            return self.getShape();
          })
      .def(
          "getShape",
          [](const ShapeFrame& self) -> dart::dynamics::ConstShapePtr {
            return self.getShape();
          })
      .def("hasVisualAspect", &ShapeFrame::hasVisualAspect)
      .def(
          "getVisualAspect",
          [](ShapeFrame& self, bool createIfNull) -> VisualAspect* {
            return self.getVisualAspect(createIfNull);
          },
          nb::arg("createIfNull") = true,
          nb::rv_policy::reference_internal)
      .def(
          "createVisualAspect",
          &ShapeFrame::createVisualAspect,
          nb::rv_policy::reference_internal)
      .def("hasCollisionAspect", &ShapeFrame::hasCollisionAspect)
      .def(
          "getCollisionAspect",
          [](ShapeFrame& self, bool createIfNull) -> CollisionAspect* {
            return self.getCollisionAspect(createIfNull);
          },
          nb::arg("createIfNull") = true,
          nb::rv_policy::reference_internal)
      .def(
          "createCollisionAspect",
          &ShapeFrame::createCollisionAspect,
          nb::rv_policy::reference_internal)
      .def("hasDynamicsAspect", &ShapeFrame::hasDynamicsAspect)
      .def(
          "getDynamicsAspect",
          [](ShapeFrame& self, bool createIfNull) -> DynamicsAspect* {
            return self.getDynamicsAspect(createIfNull);
          },
          nb::arg("createIfNull") = true,
          nb::rv_policy::reference_internal)
      .def(
          "createDynamicsAspect",
          &ShapeFrame::createDynamicsAspect,
          nb::rv_policy::reference_internal)
      .def("isShapeNode", &ShapeFrame::isShapeNode)
      .def(
          "asShapeNode",
          [](ShapeFrame& self) -> ShapeNode* { return self.asShapeNode(); },
          nb::rv_policy::reference_internal)
      .def(
          "asShapeNode",
          [](const ShapeFrame& self) -> const ShapeNode* {
            return self.asShapeNode();
          },
          nb::rv_policy::reference_internal);
}

} // namespace dart::python_nb
