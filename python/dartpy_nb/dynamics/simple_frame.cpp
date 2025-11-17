#include "dynamics/simple_frame.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Frame.hpp"
#include "dart/dynamics/SimpleFrame.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defSimpleFrame(nb::module_& m)
{
  using SimpleFrame = dart::dynamics::SimpleFrame;
  using Frame = dart::dynamics::Frame;

  auto resolveFrame = [](nb::handle input) -> Frame* {
    if (!input || input.is_none())
      return Frame::World();
    if (nb::isinstance(input, nb::type<dart::dynamics::BodyNode>()))
      return static_cast<Frame*>(nb::cast<dart::dynamics::BodyNode*>(input));
    if (nb::isinstance(input, nb::type<SimpleFrame>()))
      return static_cast<Frame*>(nb::cast<SimpleFrame*>(input));
    return nb::cast<Frame*>(input);
  };

  nb::class_<SimpleFrame, dart::dynamics::ShapeFrame>(m, "SimpleFrame")
      .def(
          nb::new_([resolveFrame](nb::handle refFrame,
                                  const std::string& name,
                                  const Eigen::Isometry3d& relativeTransform) {
            Frame* parent = resolveFrame(refFrame);
            return std::make_shared<SimpleFrame>(parent, name, relativeTransform);
          }),
          nb::arg("refFrame") = nb::none(),
          nb::arg("name") = "simple_frame",
          nb::arg("relativeTransform") = Eigen::Isometry3d::Identity())
      .def(
          "isShapeFrame",
          [](const SimpleFrame& self) { return self.isShapeFrame(); })
      .def(
          "isShapeNode",
          [](const SimpleFrame& self) { return self.isShapeNode(); })
      .def("isWorld", [](const SimpleFrame& self) { return self.isWorld(); })
      .def(
          "setName",
          [](SimpleFrame& self, const std::string& name) -> const std::string& {
            return self.setName(name);
          },
          nb::rv_policy::reference_internal,
          nb::arg("name"))
      .def(
          "getName",
          [](const SimpleFrame& self) -> const std::string& {
            return self.getName();
          },
          nb::rv_policy::reference_internal)
      .def(
          "spawnChildSimpleFrame",
          [](SimpleFrame& self) { return self.spawnChildSimpleFrame(); })
      .def(
          "spawnChildSimpleFrame",
          [](SimpleFrame& self, const std::string& name) {
            return self.spawnChildSimpleFrame(name);
          },
          nb::arg("name"))
      .def(
          "spawnChildSimpleFrame",
          [](SimpleFrame& self,
             const std::string& name,
             const Eigen::Isometry3d& relativeTransform) {
            return self.spawnChildSimpleFrame(name, relativeTransform);
          },
          nb::arg("name"),
          nb::arg("relativeTransform"))
      .def(
          "setShape",
          [](SimpleFrame& self, const dart::dynamics::ShapePtr& shape) {
            self.setShape(shape);
          },
          nb::arg("shape"))
      .def(
          "getShape",
          [](SimpleFrame& self) -> dart::dynamics::ShapePtr {
            return self.getShape();
          })
      .def(
          "getShape",
          [](const SimpleFrame& self) -> dart::dynamics::ConstShapePtr {
            return self.getShape();
          })
      .def("hasVisualAspect", &SimpleFrame::hasVisualAspect)
      .def(
          "getVisualAspect",
          [](SimpleFrame& self, bool createIfNull) {
            return self.getVisualAspect(createIfNull);
          },
          nb::arg("createIfNull") = true,
          nb::rv_policy::reference_internal)
      .def(
          "createVisualAspect",
          [](SimpleFrame& self) { return self.createVisualAspect(); },
          nb::rv_policy::reference_internal)
      .def("hasCollisionAspect", &SimpleFrame::hasCollisionAspect)
      .def(
          "getCollisionAspect",
          [](SimpleFrame& self, bool createIfNull) {
            return self.getCollisionAspect(createIfNull);
          },
          nb::arg("createIfNull") = true,
          nb::rv_policy::reference_internal)
      .def(
          "createCollisionAspect",
          [](SimpleFrame& self) { return self.createCollisionAspect(); },
          nb::rv_policy::reference_internal)
      .def("hasDynamicsAspect", &SimpleFrame::hasDynamicsAspect)
      .def(
          "getDynamicsAspect",
          [](SimpleFrame& self, bool createIfNull) {
            return self.getDynamicsAspect(createIfNull);
          },
          nb::arg("createIfNull") = true,
          nb::rv_policy::reference_internal)
      .def(
          "createDynamicsAspect",
          [](SimpleFrame& self) { return self.createDynamicsAspect(); },
          nb::rv_policy::reference_internal)
      .def(
          "setRelativeTranslation",
          [](SimpleFrame& self, const Eigen::Vector3d& translation) {
            self.setRelativeTranslation(translation);
          },
          nb::arg("newTranslation"))
      .def(
          "setRelativeRotation",
          [](SimpleFrame& self, const Eigen::Matrix3d& rotation) {
            self.setRelativeRotation(rotation);
          },
          nb::arg("newRotation"))
      .def(
          "setRelativeTransform",
          [](SimpleFrame& self, const Eigen::Isometry3d& transform) {
            self.setRelativeTransform(transform);
          },
          nb::arg("newRelTransform"))
      .def(
          "setTranslation",
          [](SimpleFrame& self, const Eigen::Vector3d& translation) {
            self.setTranslation(translation);
          },
          nb::arg("newTranslation"))
      .def(
          "setTranslation",
          [](SimpleFrame& self,
             const Eigen::Vector3d& translation,
             const Frame* withRespectTo) {
            const Frame* target
                = withRespectTo ? withRespectTo : Frame::World();
            self.setTranslation(translation, target);
          },
          nb::arg("newTranslation"),
          nb::arg("withRespectTo"))
      .def(
          "setTransform",
          [](SimpleFrame& self, const Eigen::Isometry3d& transform) {
            self.setTransform(transform);
          },
          nb::arg("newTransform"))
      .def(
          "setTransform",
          [](SimpleFrame& self,
             const Eigen::Isometry3d& transform,
             const Frame* withRespectTo) {
            const Frame* target
                = withRespectTo ? withRespectTo : Frame::World();
            self.setTransform(transform, target);
          },
          nb::arg("newTransform"),
          nb::arg("withRespectTo"))
      .def(
          "getTransform",
          [](const SimpleFrame& self) { return self.getTransform(); })
      .def(
          "getTransform",
          [](const SimpleFrame& self, const Frame* withRespectTo) {
            const Frame* target
                = withRespectTo ? withRespectTo : Frame::World();
            return self.getTransform(target);
          },
          nb::arg("withRespectTo"))
      .def(
          "getParentFrame",
          [](SimpleFrame& self) -> Frame* { return self.getParentFrame(); },
          nb::rv_policy::reference_internal)
      .def(
          "descendsFrom",
          [](const SimpleFrame& self, const Frame* someFrame) {
            return self.descendsFrom(someFrame);
          },
          nb::arg("someFrame") = nullptr);
}

} // namespace dart::python_nb
