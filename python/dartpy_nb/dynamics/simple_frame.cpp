#include "dynamics/simple_frame.hpp"

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

  nb::class_<SimpleFrame, dart::dynamics::ShapeFrame>(m, "SimpleFrame")
      .def(nb::init<>())
      .def(nb::init<Frame*>(), nb::arg("refFrame"))
      .def(
          nb::init<Frame*, const std::string&>(),
          nb::arg("refFrame"),
          nb::arg("name"))
      .def(
          nb::init<Frame*, const std::string&, const Eigen::Isometry3d&>(),
          nb::arg("refFrame"),
          nb::arg("name"),
          nb::arg("relativeTransform"))
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
