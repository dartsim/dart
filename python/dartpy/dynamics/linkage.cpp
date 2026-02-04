#include "dynamics/linkage.hpp"

#include "common/repr.hpp"
#include "common/type_casters.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/linkage.hpp"

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
  if (!node) {
    return nullptr;
  }

  auto skeleton = node->getSkeleton();
  if (!skeleton) {
    return nullptr;
  }

  return std::shared_ptr<dart::dynamics::BodyNode>(skeleton, node);
}

} // namespace

void defLinkage(nb::module_& m)
{
  using Linkage = dart::dynamics::Linkage;

  nb::class_<Linkage, dart::dynamics::MetaSkeleton>(m, "Linkage")
      .def(
          nb::new_([](const Linkage::Criteria& criteria) {
            return Linkage::create(criteria);
          }),
          nb::arg("criteria"))
      .def(
          nb::new_(
              [](const Linkage::Criteria& criteria, const std::string& name) {
                return Linkage::create(criteria, name);
              }),
          nb::arg("criteria"),
          nb::arg("name"))
      .def(
          "cloneLinkage",
          [](const Linkage& self) { return self.cloneLinkage(); })
      .def(
          "cloneLinkage",
          [](const Linkage& self, const std::string& name) {
            return self.cloneLinkage(name);
          },
          nb::arg("clone_name"))
      .def(
          "cloneMetaSkeleton",
          [](const Linkage& self, const std::string& name) {
            return self.cloneMetaSkeleton(name);
          },
          nb::arg("clone_name"))
      .def(
          "__repr__",
          [](const Linkage& self) {
            std::vector<std::pair<std::string, std::string>> fields;
            fields.emplace_back("name", repr_string(self.getName()));
            fields.emplace_back(
                "body_nodes", std::to_string(self.getNumBodyNodes()));
            fields.emplace_back("joints", std::to_string(self.getNumJoints()));
            fields.emplace_back("dofs", std::to_string(self.getNumDofs()));
            fields.emplace_back("assembled", repr_bool(self.isAssembled()));
            return format_repr("Linkage", fields);
          })
      .def("isAssembled", &Linkage::isAssembled)
      .def("reassemble", &Linkage::reassemble)
      .def("satisfyCriteria", &Linkage::satisfyCriteria);

  auto linkageCriteria = nb::class_<Linkage::Criteria>(m, "LinkageCriteria");
  linkageCriteria.def(nb::init<>())
      .def(
          "satisfy",
          [](const Linkage::Criteria& self) { return self.satisfy(); })
      .def_rw("mStart", &Linkage::Criteria::mStart, "Criteria start node.")
      .def_rw("mTargets", &Linkage::Criteria::mTargets, "Criteria targets.")
      .def_rw(
          "mTerminals", &Linkage::Criteria::mTerminals, "Criteria terminals.");

  nb::enum_<Linkage::Criteria::ExpansionPolicy>(
      linkageCriteria, "ExpansionPolicy")
      .value("INCLUDE", Linkage::Criteria::ExpansionPolicy::INCLUDE)
      .value("EXCLUDE", Linkage::Criteria::ExpansionPolicy::EXCLUDE)
      .value("DOWNSTREAM", Linkage::Criteria::ExpansionPolicy::DOWNSTREAM)
      .value("UPSTREAM", Linkage::Criteria::ExpansionPolicy::UPSTREAM)
      .export_values();

  nb::class_<Linkage::Criteria::Terminal>(linkageCriteria, "Terminal")
      .def(nb::init<>())
      .def(nb::init<dart::dynamics::BodyNode*>(), nb::arg("terminal"))
      .def(
          nb::init<dart::dynamics::BodyNode*, bool>(),
          nb::arg("terminal"),
          nb::arg("inclusive"))
      .def_prop_rw(
          "mTerminal",
          [](const Linkage::Criteria::Terminal& self) {
            return lockBodyNode(self.mTerminal);
          },
          [](Linkage::Criteria::Terminal& self, nb::handle terminal) {
            if (!terminal || terminal.is_none()) {
              self.mTerminal = nullptr;
              return;
            }
            self.mTerminal = nb::cast<dart::dynamics::BodyNode*>(terminal);
          },
          nb::for_setter(nb::arg("terminal").none()))
      .def_rw("mInclusive", &Linkage::Criteria::Terminal::mInclusive);

  nb::class_<Linkage::Criteria::Target>(linkageCriteria, "Target")
      .def(nb::init<>())
      .def(nb::init<dart::dynamics::BodyNode*>(), nb::arg("target"))
      .def(
          nb::init<
              dart::dynamics::BodyNode*,
              Linkage::Criteria::ExpansionPolicy>(),
          nb::arg("target"),
          nb::arg("policy"))
      .def(
          nb::init<
              dart::dynamics::BodyNode*,
              Linkage::Criteria::ExpansionPolicy,
              bool>(),
          nb::arg("target"),
          nb::arg("policy"),
          nb::arg("chain"))
      .def_prop_rw(
          "mNode",
          [](const Linkage::Criteria::Target& self) {
            return lockBodyNode(self.mNode);
          },
          [](Linkage::Criteria::Target& self, nb::handle node) {
            if (!node || node.is_none()) {
              self.mNode = nullptr;
              return;
            }
            self.mNode = nb::cast<dart::dynamics::BodyNode*>(node);
          },
          nb::for_setter(nb::arg("node").none()))
      .def_rw("mPolicy", &Linkage::Criteria::Target::mPolicy)
      .def_rw("mChain", &Linkage::Criteria::Target::mChain);
}

} // namespace dart::python_nb
