#include "optimizer/gradient_descent.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "dart/optimizer/GradientDescentSolver.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defGradientDescentSolver(nb::module_& m)
{
  using Solver = dart::optimizer::GradientDescentSolver;

  nb::class_<Solver::UniqueProperties>(m, "GradientDescentSolverUniqueProperties")
      .def(nb::init<>())
      .def(nb::init<double>(), nb::arg("stepMultiplier"))
      .def(nb::init<double, std::size_t>(), nb::arg("stepMultiplier"), nb::arg("maxAttempts"));

  nb::class_<Solver::Properties, dart::optimizer::Solver::Properties, Solver::UniqueProperties>(m, "GradientDescentSolverProperties")
      .def(nb::init<>())
      .def(nb::init<const dart::optimizer::Solver::Properties&>(), nb::arg("solverProperties"));

  nb::class_<Solver, dart::optimizer::Solver, std::shared_ptr<Solver>>(m, "GradientDescentSolver")
      .def(nb::init<>())
      .def(nb::init<const Solver::Properties&>(), nb::arg("properties"))
      .def(nb::init<std::shared_ptr<dart::optimizer::Problem>>(), nb::arg("problem"))
      .def("solve", &Solver::solve)
      .def("getLastConfiguration", &Solver::getLastConfiguration)
      .def("getType", &Solver::getType)
      .def("clone", &Solver::clone)
      .def("setProperties", &Solver::setProperties, nb::arg("properties"))
      .def("getProperties", &Solver::getProperties)
      .def("setStepSize", &Solver::setStepSize, nb::arg("step"))
      .def("getStepSize", &Solver::getStepSize)
      .def("getTolerance", &Solver::getTolerance);
}

} // namespace dart::python_nb
