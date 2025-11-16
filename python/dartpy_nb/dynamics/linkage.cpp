#include "dynamics/linkage.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Linkage.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

namespace dart::python_nb {

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
          nb::arg("cloneName"))
      .def(
          "cloneMetaSkeleton",
          [](const Linkage& self, const std::string& name) {
            return self.cloneMetaSkeleton(name);
          },
          nb::arg("cloneName"))
      .def("isAssembled", &Linkage::isAssembled)
      .def("reassemble", &Linkage::reassemble)
      .def("satisfyCriteria", &Linkage::satisfyCriteria);

  auto linkageCriteria = nb::class_<Linkage::Criteria>(m, "LinkageCriteria");
  linkageCriteria
      .def(nb::init<>())
      .def(
          "satisfy",
          [](const Linkage::Criteria& self) { return self.satisfy(); })
      .def_rw(
          "mStart", &Linkage::Criteria::mStart, "Criteria start node.")
      .def_rw(
          "mTargets",
          &Linkage::Criteria::mTargets,
          "Criteria targets.")
      .def_rw(
          "mTerminals",
          &Linkage::Criteria::mTerminals,
          "Criteria terminals.");

  nb::enum_<Linkage::Criteria::ExpansionPolicy>(
      linkageCriteria, "ExpansionPolicy")
      .value(
          "INCLUDE", Linkage::Criteria::ExpansionPolicy::INCLUDE)
      .value(
          "EXCLUDE", Linkage::Criteria::ExpansionPolicy::EXCLUDE)
      .value(
          "DOWNSTREAM", Linkage::Criteria::ExpansionPolicy::DOWNSTREAM)
      .value(
          "UPSTREAM", Linkage::Criteria::ExpansionPolicy::UPSTREAM)
      .export_values();

  nb::class_<Linkage::Criteria::Terminal>(linkageCriteria, "Terminal")
      .def(nb::init<>())
      .def(nb::init<dart::dynamics::BodyNode*>(), nb::arg("terminal"))
      .def(
          nb::init<dart::dynamics::BodyNode*, bool>(),
          nb::arg("terminal"),
          nb::arg("inclusive"))
      .def_rw(
          "mTerminal", &Linkage::Criteria::Terminal::mTerminal)
      .def_rw(
          "mInclusive", &Linkage::Criteria::Terminal::mInclusive);

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
      .def_rw("mNode", &Linkage::Criteria::Target::mNode)
      .def_rw("mPolicy", &Linkage::Criteria::Target::mPolicy)
      .def_rw("mChain", &Linkage::Criteria::Target::mChain);
}

} // namespace dart::python_nb
