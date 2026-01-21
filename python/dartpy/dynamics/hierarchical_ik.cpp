#include "dynamics/hierarchical_ik.hpp"

#include "dart/dynamics/hierarchical_ik.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defHierarchicalIK(nb::module_& m)
{
  using HierarchicalIK = dart::dynamics::HierarchicalIK;
  using WholeBodyIK = dart::dynamics::WholeBodyIK;

  nb::class_<HierarchicalIK>(m, "HierarchicalIK")
      .def(
          "solveAndApply",
          [](HierarchicalIK& self, bool allowIncompleteResult) {
            return self.solveAndApply(allowIncompleteResult);
          },
          nb::arg("allow_incomplete_result") = true);

  nb::class_<WholeBodyIK, HierarchicalIK>(m, "WholeBodyIK")
      .def("refreshIKHierarchy", &WholeBodyIK::refreshIKHierarchy);
}

} // namespace dart::python_nb
