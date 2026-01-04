#include "simulation/constraint_solver.hpp"

#include "collision/collision_group.hpp"
#include "collision/collision_option.hpp"
#include "dart/constraint/ConstraintBase.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/constraint/ContactManifoldCache.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defConstraintSolver(nb::module_& m)
{
  using ConstraintSolver = dart::constraint::ConstraintSolver;

  nb::class_<ConstraintSolver>(m, "ConstraintSolver")
      .def(
          "getCollisionDetector",
          [](ConstraintSolver& self) { return self.getCollisionDetector(); })
      .def(
          "setCollisionDetector",
          [](ConstraintSolver& self,
             const std::shared_ptr<dart::collision::CollisionDetector>&
                 detector) { self.setCollisionDetector(detector); },
          nb::arg("detector"))
      .def(
          "getCollisionGroup",
          [](ConstraintSolver& self) { return self.getCollisionGroup(); })
      .def(
          "getCollisionOption",
          [](ConstraintSolver& self) -> dart::collision::CollisionOption& {
            return self.getCollisionOption();
          },
          nb::rv_policy::reference_internal)
      .def(
          "setContactManifoldCacheOptions",
          &ConstraintSolver::setContactManifoldCacheOptions,
          nb::arg("options"))
      .def(
          "getContactManifoldCacheOptions",
          [](const ConstraintSolver& self) {
            return self.getContactManifoldCacheOptions();
          })
      .def(
          "setContactManifoldCacheEnabled",
          &ConstraintSolver::setContactManifoldCacheEnabled,
          nb::arg("enabled"))
      .def(
          "isContactManifoldCacheEnabled",
          &ConstraintSolver::isContactManifoldCacheEnabled)
      .def(
          "getNumPersistentContacts",
          &ConstraintSolver::getNumPersistentContacts)
      .def("getNumContactManifolds", &ConstraintSolver::getNumContactManifolds)
      .def(
          "getNumContactConstraints",
          &ConstraintSolver::getNumContactConstraints)
      .def(
          "getNumSoftContactConstraints",
          &ConstraintSolver::getNumSoftContactConstraints)
      .def(
          "getContactsUsedForConstraints",
          [](const ConstraintSolver& self) {
            std::vector<dart::collision::Contact> contacts;
            self.getContactsUsedForConstraints(contacts);
            return contacts;
          })
      .def(
          "addConstraint",
          [](ConstraintSolver& self,
             const std::shared_ptr<dart::constraint::ConstraintBase>&
                 constraint) { self.addConstraint(constraint); },
          nb::arg("constraint"))
      .def(
          "removeConstraint",
          [](ConstraintSolver& self,
             const std::shared_ptr<dart::constraint::ConstraintBase>&
                 constraint) { self.removeConstraint(constraint); },
          nb::arg("constraint"));
}

} // namespace dart::python_nb
