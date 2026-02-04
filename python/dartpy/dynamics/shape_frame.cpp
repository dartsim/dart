#include "dynamics/shape_frame.hpp"

#include "common/eigen_utils.hpp"
#include "common/repr.hpp"
#include "common/type_casters.hpp"
#include "dart/dynamics/shape.hpp"
#include "dart/dynamics/shape_frame.hpp"
#include "dart/dynamics/shape_node.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <cstddef>

namespace nb = nanobind;

namespace dart::python_nb {

void defShapeFrame(nb::module_& m)
{
  using Frame = dart::dynamics::Frame;
  using ShapeFrame = dart::dynamics::ShapeFrame;
  using ShapeNode = dart::dynamics::ShapeNode;
  using VisualAspect = dart::dynamics::VisualAspect;
  using CollisionAspect = dart::dynamics::CollisionAspect;
  using DynamicsAspect = dart::dynamics::DynamicsAspect;

  nb::class_<VisualAspect>(m, "VisualAspect")
      .def(nb::init<>())
      .def(
          "setColor",
          [](VisualAspect& self, const nb::handle& color) {
            Eigen::VectorXd vec = toVector(color);
            if (vec.size() == 3) {
              self.setColor(Eigen::Vector3d(vec));
            } else if (vec.size() == 4) {
              self.setColor(Eigen::Vector4d(vec));
            } else {
              throw nb::type_error("Color must be length 3 or 4");
            }
          },
          nb::arg("color"))
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
          nb::arg("create_if_null") = true,
          nb::rv_policy::reference_internal)
      .def(
          "createVisualAspect",
          [](ShapeFrame& self) { return self.createVisualAspect(); },
          nb::rv_policy::reference_internal)
      .def("hasCollisionAspect", &ShapeFrame::hasCollisionAspect)
      .def(
          "getCollisionAspect",
          [](ShapeFrame& self, bool createIfNull) -> CollisionAspect* {
            return self.getCollisionAspect(createIfNull);
          },
          nb::arg("create_if_null") = true,
          nb::rv_policy::reference_internal)
      .def(
          "createCollisionAspect",
          [](ShapeFrame& self) { return self.createCollisionAspect(); },
          nb::rv_policy::reference_internal)
      .def("hasDynamicsAspect", &ShapeFrame::hasDynamicsAspect)
      .def(
          "getDynamicsAspect",
          [](ShapeFrame& self, bool createIfNull) -> DynamicsAspect* {
            return self.getDynamicsAspect(createIfNull);
          },
          nb::arg("create_if_null") = true,
          nb::rv_policy::reference_internal)
      .def(
          "createDynamicsAspect",
          [](ShapeFrame& self) { return self.createDynamicsAspect(); },
          nb::rv_policy::reference_internal)
      .def("isShapeNode", &ShapeFrame::isShapeNode)
      .def(
          "__repr__",
          [](const ShapeFrame& self) {
            const auto shape = self.getShape();
            const auto* parent = self.getParentFrame();
            std::vector<std::pair<std::string, std::string>> fields;
            fields.emplace_back("name", repr_string(self.getName()));
            fields.emplace_back(
                "shape", shape ? repr_string(shape->getType()) : "None");
            fields.emplace_back(
                "parent", parent ? repr_string(parent->getName()) : "None");
            return format_repr("ShapeFrame", fields);
          })
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

  registerPolymorphicCaster<Frame, ShapeFrame>();
}

} // namespace dart::python_nb
