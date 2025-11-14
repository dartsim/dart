#include "optimizer/problem.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "dart/optimizer/Problem.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defOptimizerProblem(nb::module_& m)
{
  using Problem = dart::optimizer::Problem;

  nb::class_<Problem, std::shared_ptr<Problem>>(m, "Problem")
      .def(nb::init<>())
      .def(nb::init<std::size_t>(), nb::arg("dim"))
      .def("setDimension", &Problem::setDimension, nb::arg("dim"))
      .def("getDimension", &Problem::getDimension)
      .def("setInitialGuess", &Problem::setInitialGuess, nb::arg("guess"))
      .def("getInitialGuess", [](Problem& self) { return self.getInitialGuess(); })
      .def("addSeed", &Problem::addSeed, nb::arg("seed"))
      .def("clearAllSeeds", &Problem::clearAllSeeds)
      .def("setLowerBounds", &Problem::setLowerBounds, nb::arg("lb"))
      .def("setUpperBounds", &Problem::setUpperBounds, nb::arg("ub"))
      .def("setObjective", &Problem::setObjective, nb::arg("function"))
      .def("getObjective", &Problem::getObjective)
      .def("addEqConstraint", &Problem::addEqConstraint, nb::arg("constraint"))
      .def("addIneqConstraint", &Problem::addIneqConstraint, nb::arg("constraint"))
      .def("getNumEqConstraints", &Problem::getNumEqConstraints)
      .def("getNumIneqConstraints", &Problem::getNumIneqConstraints)
      .def("getEqConstraint", &Problem::getEqConstraint, nb::arg("idx"))
      .def("getIneqConstraint", &Problem::getIneqConstraint, nb::arg("idx"))
      .def("removeEqConstraint", &Problem::removeEqConstraint, nb::arg("constraint"))
      .def("removeIneqConstraint", &Problem::removeIneqConstraint, nb::arg("constraint"))
      .def("removeAllEqConstraints", &Problem::removeAllEqConstraints)
      .def("removeAllIneqConstraints", &Problem::removeAllIneqConstraints)
      .def("setOptimumValue", &Problem::setOptimumValue, nb::arg("value"))
      .def("getOptimumValue", &Problem::getOptimumValue)
      .def("setOptimalSolution", &Problem::setOptimalSolution, nb::arg("solution"))
      .def("getOptimalSolution", [](Problem& self) { return self.getOptimalSolution(); });
}

} // namespace dart::python_nb
