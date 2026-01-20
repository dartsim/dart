#include "dynamics/chain.hpp"

#include "common/repr.hpp"
#include "common/type_casters.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/chain.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

std::shared_ptr<dart::dynamics::BodyNode> lockBodyNode(
    const dart::dynamics::WeakBodyNodePtr& weak)
{
  auto locked = weak.lock();
  auto* node = locked.get();
  if (!node)
    return nullptr;

  auto skeleton = node->getSkeleton();
  if (!skeleton)
    return nullptr;

  return std::shared_ptr<dart::dynamics::BodyNode>(skeleton, node);
}

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
          nb::arg("include_upstream_parent_joint"))
      .def(
          "satisfy", [](const Chain::Criteria& self) { return self.satisfy(); })
      .def(
          "convert", [](const Chain::Criteria& self) { return self.convert(); })
      .def_static(
          "static_convert",
          [](const dart::dynamics::Linkage::Criteria& criteria) {
            return Chain::Criteria::convert(criteria);
          },
          nb::arg("criteria"))
      .def_prop_rw(
          "mStart",
          [](const Chain::Criteria& self) { return lockBodyNode(self.mStart); },
          [](Chain::Criteria& self, nb::handle start) {
            if (!start || start.is_none()) {
              self.mStart = nullptr;
              return;
            }
            self.mStart = nb::cast<dart::dynamics::BodyNode*>(start);
          },
          "Criteria start node.",
          nb::for_setter(nb::arg("start").none()))
      .def_prop_rw(
          "mTarget",
          [](const Chain::Criteria& self) {
            return lockBodyNode(self.mTarget);
          },
          [](Chain::Criteria& self, nb::handle target) {
            if (!target || target.is_none()) {
              self.mTarget = nullptr;
              return;
            }
            self.mTarget = nb::cast<dart::dynamics::BodyNode*>(target);
          },
          "Criteria target node.",
          nb::for_setter(nb::arg("target").none()))
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
          nb::arg("include_upstream_parent_joint") = false,
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
      .def("isStillChain", &Chain::isStillChain)
      .def("__repr__", [](const Chain& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("name", repr_string(self.getName()));
        fields.emplace_back(
            "body_nodes", std::to_string(self.getNumBodyNodes()));
        fields.emplace_back("joints", std::to_string(self.getNumJoints()));
        fields.emplace_back("dofs", std::to_string(self.getNumDofs()));
        fields.emplace_back("assembled", repr_bool(self.isAssembled()));
        fields.emplace_back("valid", repr_bool(self.isStillChain()));
        return format_repr("Chain", fields);
      });
}

} // namespace dart::python_nb
