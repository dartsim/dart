#include "dynamics/chain.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Chain.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

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
        start, target, dart::dynamics::Chain::IncludeUpstreamParentJoint, name);
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

  nb::class_<Chain::Criteria>(m, "ChainCriteria")
      .def(
          nb::init<dart::dynamics::BodyNode*, dart::dynamics::BodyNode*>(),
          nb::arg("start"),
          nb::arg("target"))
      .def(
          nb::init<
              dart::dynamics::BodyNode*,
              dart::dynamics::BodyNode*,
              bool>(),
          nb::arg("start"),
          nb::arg("target"),
          nb::arg("includeUpstreamParentJoint"))
      .def(
          "satisfy",
          [](const Chain::Criteria& self) { return self.satisfy(); })
      .def(
          "convert",
          [](const Chain::Criteria& self) { return self.convert(); })
      .def_static(
          "static_convert",
          [](const dart::dynamics::Linkage::Criteria& criteria) {
            return Chain::Criteria::convert(criteria);
          },
          nb::arg("criteria"))
      .def_rw("mStart", &Chain::Criteria::mStart)
      .def_rw("mTarget", &Chain::Criteria::mTarget)
      .def_rw(
          "mIncludeUpstreamParentJoint",
          &Chain::Criteria::mIncludeUpstreamParentJoint);

  nb::class_<Chain, dart::dynamics::Linkage>(m, "Chain")
      .def(
          nb::new_([](const Chain::Criteria& criteria) {
            return Chain::create(criteria);
          }),
          nb::arg("criteria"))
      .def(
          nb::new_(
              [](const Chain::Criteria& criteria, const std::string& name) {
                return Chain::create(criteria, name);
              }),
          nb::arg("criteria"),
          nb::arg("name"))
      .def(
          nb::new_([](dart::dynamics::BodyNode* start,
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
      .def(
          "cloneChain",
          [](const Chain& self, const std::string& name) {
            return self.cloneChain(name);
          },
          nb::arg("name"))
      .def(
          "cloneMetaSkeleton",
          [](const Chain& self, const std::string& name) {
            return self.cloneMetaSkeleton(name);
          },
          nb::arg("name"))
      .def("isStillChain", &Chain::isStillChain);
}

} // namespace dart::python_nb
