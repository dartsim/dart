#include "dynamics/entity.hpp"

#include "common/type_casters.hpp"
#include "dart/dynamics/entity.hpp"
#include "dart/dynamics/frame.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defEntity(nb::module_& m)
{
  using Entity = dart::dynamics::Entity;
  using Detachable = dart::dynamics::Detachable;
  using Frame = dart::dynamics::Frame;

  nb::class_<Entity>(m, "Entity")
      .def(
          "setName",
          [](Entity& self, const std::string& name) -> const std::string& {
            return self.setName(name);
          },
          nb::rv_policy::reference_internal,
          nb::arg("name"))
      .def(
          "getName",
          [](const Entity& self) -> const std::string& {
            return self.getName();
          },
          nb::rv_policy::reference_internal)
      .def(
          "getParentFrame",
          [](const Entity& self) -> const Frame* {
            return self.getParentFrame();
          },
          nb::rv_policy::reference_internal)
      .def(
          "descendsFrom",
          [](const Entity& self, const Frame* frame) {
            return self.descendsFrom(frame);
          },
          nb::arg("some_frame") = nullptr)
      .def("isFrame", &Entity::isFrame)
      .def("isQuiet", &Entity::isQuiet);

  nb::class_<Detachable, Entity>(m, "Detachable")
      .def(
          "setParentFrame",
          [](Detachable& self, Frame* newParent) {
            self.setParentFrame(newParent);
          },
          nb::arg("new_parent_frame"));
}

} // namespace dart::python_nb
