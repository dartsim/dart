#include "optimizer/gradient_descent.hpp"

#include "dart/common/diagnostics.hpp"
#include "dart/math/optimization/gradient_descent_solver.hpp"
#include "dart/math/optimization/problem.hpp"
#include "dart/math/optimization/solver.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string_view.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defGradientDescentSolver(nb::module_& m)
{
  using Solver = dart::math::GradientDescentSolver;

  nb::class_<Solver::UniqueProperties>(
      m, "GradientDescentSolverUniqueProperties")
      .def(nb::init<>())
      .def(nb::init<double>(), nb::arg("step_multiplier"))
      .def(
          nb::init<double, std::size_t>(),
          nb::arg("step_multiplier"),
          nb::arg("max_attempts"));

  nb::class_<Solver::Properties, dart::math::Solver::Properties>(
      m, "GradientDescentSolverProperties")
      .def(nb::init<>())
      .def(
          nb::init<
              const dart::math::Solver::Properties&,
              const Solver::UniqueProperties&>(),
          nb::arg("solver_properties"),
          nb::arg("gradient_properties") = Solver::UniqueProperties());

  nb::class_<Solver, dart::math::Solver>(m, "GradientDescentSolver")
      .def(nb::init<>())
      .def(nb::init<const Solver::Properties&>(), nb::arg("properties"))
      .def(nb::init<std::shared_ptr<dart::math::Problem>>(), nb::arg("problem"))
      .def("solve", &Solver::solve)
      .def("getLastConfiguration", &Solver::getLastConfiguration)
      .def("getType", &Solver::getType)
      .def("clone", &Solver::clone)
      .def(
          "setProperties",
          nb::overload_cast<const Solver::Properties&>(&Solver::setProperties),
          nb::arg("properties"))
      .def(
          "setProperties",
          nb::overload_cast<const Solver::UniqueProperties&>(
              &Solver::setProperties),
          nb::arg("properties"))
      .def("getProperties", &Solver::getGradientDescentProperties)
      .def("setStepSize", &Solver::setStepSize, nb::arg("step"))
      .def("getStepSize", &Solver::getStepSize)
      .def("getTolerance", &Solver::getTolerance);
}

} // namespace dart::python_nb
