#include "dynamics/meta_skeleton.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/memory.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/MetaSkeleton.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defMetaSkeleton(nb::module_& m)
{
  using MetaSkeleton = dart::dynamics::MetaSkeleton;

  nb::class_<MetaSkeleton, std::shared_ptr<MetaSkeleton>>(m, "MetaSkeleton")
      .def("cloneMetaSkeleton",
          [](const MetaSkeleton& self, const std::string& name) {
            return self.cloneMetaSkeleton(name);
          },
          nb::arg("name"))
      .def("cloneMetaSkeleton",
          [](const MetaSkeleton& self) {
            return self.cloneMetaSkeleton();
          })
      .def("setName", &MetaSkeleton::setName, nb::arg("name"), nb::rv_policy::reference_internal)
      .def("getName", &MetaSkeleton::getName, nb::rv_policy::reference_internal)
      .def("getNumBodyNodes", &MetaSkeleton::getNumBodyNodes)
      .def("getBodyNode",
          [](MetaSkeleton& self, std::size_t index) {
            return self.getBodyNode(index);
          },
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def("getBodyNode",
          [](MetaSkeleton& self, const std::string& name) {
            return self.getBodyNode(name);
          },
          nb::rv_policy::reference_internal,
          nb::arg("name"))
      .def("getBodyNodes",
          [](MetaSkeleton& self, const std::string& name) {
            return self.getBodyNodes(name);
          },
          nb::arg("name"))
      .def("getBodyNodes",
          [](const MetaSkeleton& self, const std::string& name) {
            return self.getBodyNodes(name);
          },
          nb::arg("name"))
      .def("getNumJoints", &MetaSkeleton::getNumJoints)
      .def("getJoint",
          [](MetaSkeleton& self, std::size_t index) {
            return self.getJoint(index);
          },
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def("getJoint",
          [](MetaSkeleton& self, const std::string& name) {
            return self.getJoint(name);
          },
          nb::rv_policy::reference_internal,
          nb::arg("name"))
      .def("getNumDofs", &MetaSkeleton::getNumDofs)
      .def("getDof",
          [](MetaSkeleton& self, std::size_t index) {
            return self.getDof(index);
          },
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def("getDof",
          [](MetaSkeleton& self, const std::string& name) {
            return self.getDof(name);
          },
          nb::rv_policy::reference_internal,
          nb::arg("name"));
}

} // namespace dart::python_nb
