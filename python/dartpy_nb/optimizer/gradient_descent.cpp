#include "optimizer/gradient_descent.hpp"

#include "dart/common/Diagnostics.hpp"
#include "dart/optimizer/GradientDescentSolver.hpp"
#include "dart/optimizer/Problem.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

DART_SUPPRESS_DEPRECATED_BEGIN

void defGradientDescentSolver(nb::module_& m)
{
  using Solver = dart::optimizer::GradientDescentSolver;

  nb::class_<Solver::UniqueProperties>(
      m, "GradientDescentSolverUniqueProperties")
      .def(nb::init<>())
      .def(nb::init<double>(), nb::arg("stepMultiplier"))
      .def(
          nb::init<double, std::size_t>(),
          nb::arg("stepMultiplier"),
          nb::arg("maxAttempts"));

  nb::class_<Solver::Properties, dart::optimizer::Solver::Properties>(
      m, "GradientDescentSolverProperties")
      .def(nb::init<>())
      .def(
          nb::init<
              const dart::optimizer::Solver::Properties&,
              const Solver::UniqueProperties&>(),
          nb::arg("solverProperties"),
          nb::arg("gradientProperties") = Solver::UniqueProperties());

  nb::class_<Solver, dart::optimizer::Solver>(m, "GradientDescentSolver")
      .def(nb::init<>())
      .def(nb::init<const Solver::Properties&>(), nb::arg("properties"))
      .def(
          nb::init<std::shared_ptr<dart::optimizer::Problem>>(),
          nb::arg("problem"))
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

DART_SUPPRESS_DEPRECATED_END

} // namespace dart::python_nb
