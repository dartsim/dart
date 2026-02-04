#include "dynamics/meta_skeleton.hpp"

#include "common/type_casters.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/degree_of_freedom.hpp"
#include "dart/dynamics/joint.hpp"
#include "dart/dynamics/meta_skeleton.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defMetaSkeleton(nb::module_& m)
{
  using MetaSkeleton = dart::dynamics::MetaSkeleton;

  nb::class_<MetaSkeleton>(m, "MetaSkeleton")
      .def(
          "cloneMetaSkeleton",
          [](const MetaSkeleton& self, const std::string& name) {
            return self.cloneMetaSkeleton(name);
          },
          nb::arg("name"))
      .def(
          "cloneMetaSkeleton",
          [](const MetaSkeleton& self) { return self.cloneMetaSkeleton(); })
      .def(
          "setName",
          &MetaSkeleton::setName,
          nb::arg("name"),
          nb::rv_policy::reference_internal)
      .def("getName", &MetaSkeleton::getName, nb::rv_policy::reference_internal)
      .def("getNumBodyNodes", &MetaSkeleton::getNumBodyNodes)
      .def(
          "getBodyNode",
          [](MetaSkeleton& self, std::size_t index) {
            return self.getBodyNode(index);
          },
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def(
          "getBodyNode",
          [](MetaSkeleton& self, const std::string& name) {
            return self.getBodyNode(name);
          },
          nb::rv_policy::reference_internal,
          nb::arg("name"))
      .def(
          "getBodyNodes",
          [](MetaSkeleton& self, const std::string& name) {
            return self.getBodyNodes(name);
          },
          nb::arg("name"))
      .def(
          "getBodyNodes",
          [](const MetaSkeleton& self, const std::string& name) {
            return self.getBodyNodes(name);
          },
          nb::arg("name"))
      .def("getNumJoints", &MetaSkeleton::getNumJoints)
      .def(
          "getJoint",
          [](MetaSkeleton& self, std::size_t index) {
            return self.getJoint(index);
          },
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def(
          "getJoint",
          [](MetaSkeleton& self, const std::string& name) {
            return self.getJoint(name);
          },
          nb::rv_policy::reference_internal,
          nb::arg("name"))
      .def("getNumDofs", &MetaSkeleton::getNumDofs)
      .def(
          "getDof",
          [](MetaSkeleton& self, std::size_t index) {
            return self.getDof(index);
          },
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def(
          "getDofs",
          [](MetaSkeleton& self) {
            std::vector<dart::dynamics::DegreeOfFreedom*> dofs;
            dofs.reserve(self.getNumDofs());
            for (std::size_t i = 0; i < self.getNumDofs(); ++i) {
              dofs.emplace_back(self.getDof(i));
            }
            return dofs;
          },
          nb::rv_policy::reference_internal);
}

} // namespace dart::python_nb
