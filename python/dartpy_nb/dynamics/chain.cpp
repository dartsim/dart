#include "dynamics/chain.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Chain.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

std::shared_ptr<dart::dynamics::Chain> create_chain(
    dart::dynamics::BodyNode* start,
    dart::dynamics::BodyNode* target,
    bool include_upstream,
    const std::string& name)
{
  if (include_upstream) {
    if (name.empty()) {
      return dart::dynamics::Chain::create(
          start, target, dart::dynamics::Chain::IncludeUpstreamParentJoint);
    }
    return dart::dynamics::Chain::create(
        start,
        target,
        dart::dynamics::Chain::IncludeUpstreamParentJoint,
        name);
  }

  if (name.empty()) {
    return dart::dynamics::Chain::create(start, target);
  }
  return dart::dynamics::Chain::create(start, target, name);
}

} // namespace

void defChain(nb::module_& m)
{
  using Chain = dart::dynamics::Chain;

  nb::class_<Chain, dart::dynamics::Linkage, std::shared_ptr<Chain>>(m, "Chain")
      .def(nb::init(
               [](dart::dynamics::BodyNode* start,
                   dart::dynamics::BodyNode* target,
                   bool includeUpstreamParentJoint,
                   const std::string& name) {
                 return create_chain(
                     start, target, includeUpstreamParentJoint, name);
               }),
          nb::arg("start"),
          nb::arg("target"),
          nb::arg("includeUpstreamParentJoint") = false,
          nb::arg("name") = std::string())
      .def("getNumBodyNodes", &Chain::getNumBodyNodes)
      .def("cloneChain", [](const Chain& self) { return self.cloneChain(); })
      .def("cloneChain",
          [](const Chain& self, const std::string& name) {
            return self.cloneChain(name);
          },
          nb::arg("name"));
}

} // namespace dart::python_nb
