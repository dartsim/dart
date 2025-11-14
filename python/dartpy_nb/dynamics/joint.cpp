#include "dynamics/joint.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "dart/dynamics/Joint.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defJoint(nb::module_& m)
{
  using Joint = dart::dynamics::Joint;

  nb::class_<Joint, std::shared_ptr<Joint>>(m, "Joint")
      .def("getName",
          [](const Joint& self) -> const std::string& {
            return self.getName();
          },
          nb::rv_policy::reference_internal)
      .def("setName", &Joint::setName, nb::arg("name"))
      .def("getType",
          [](const Joint& self) -> const std::string& {
            return self.getType();
          },
          nb::rv_policy::reference_internal);
}

} // namespace dart::python_nb
