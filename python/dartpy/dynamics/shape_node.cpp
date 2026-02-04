#include "dynamics/shape_node.hpp"

#include "common/repr.hpp"
#include "common/type_casters.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/shape_node.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

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
      .def("getOffset", &ShapeNode::getOffset)
      .def("__repr__", [](const ShapeNode& self) {
        const auto shape = self.getShape();
        const auto* parent_frame = self.getParentFrame();
        const auto* body
            = dynamic_cast<const dart::dynamics::BodyNode*>(parent_frame);
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("name", repr_string(self.getName()));
        fields.emplace_back(
            "shape", shape ? repr_string(shape->getType()) : "None");
        fields.emplace_back(
            "body",
            body ? repr_string(body->getName())
                 : (parent_frame ? repr_string(parent_frame->getName())
                                 : "None"));
        return format_repr("ShapeNode", fields);
      });

  registerPolymorphicCaster<dart::dynamics::Frame, ShapeNode>();
  registerPolymorphicCaster<dart::dynamics::ShapeFrame, ShapeNode>();
}

} // namespace dart::python_nb
