#include "simulation/constraint_solver.hpp"

#include "collision/collision_group.hpp"
#include "collision/collision_option.hpp"
#include "dart/constraint/constraint_base.hpp"
#include "dart/constraint/constraint_solver.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

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
