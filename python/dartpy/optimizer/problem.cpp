#include "optimizer/problem.hpp"

#include "common/eigen_utils.hpp"
#include "dart/common/diagnostics.hpp"
#include "dart/math/optimization/problem.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

DART_SUPPRESS_DEPRECATED_BEGIN

void defOptimizerProblem(nb::module_& m)
{
  using Problem = dart::math::Problem;

  nb::class_<Problem>(m, "Problem")
      .def(nb::init<>())
      .def(nb::init<std::size_t>(), nb::arg("dim"))
      .def("setDimension", &Problem::setDimension, nb::arg("dim"))
      .def("getDimension", &Problem::getDimension)
      .def(
          "setInitialGuess",
          [](Problem& self, const nb::handle& guess) {
            self.setInitialGuess(toVector(guess));
          },
          nb::arg("guess"))
      .def(
          "getInitialGuess",
          [](Problem& self) { return self.getInitialGuess(); })
      .def("addSeed", &Problem::addSeed, nb::arg("seed"))
      .def("clearAllSeeds", &Problem::clearAllSeeds)
      .def(
          "setLowerBounds",
          [](Problem& self, const nb::handle& lb) {
            self.setLowerBounds(toVector(lb));
          },
          nb::arg("lb"))
      .def(
          "setUpperBounds",
          [](Problem& self, const nb::handle& ub) {
            self.setUpperBounds(toVector(ub));
          },
          nb::arg("ub"))
      .def("setObjective", &Problem::setObjective, nb::arg("function"))
      .def("getObjective", &Problem::getObjective)
      .def("addEqConstraint", &Problem::addEqConstraint, nb::arg("constraint"))
      .def(
          "addIneqConstraint",
          &Problem::addIneqConstraint,
          nb::arg("constraint"))
      .def("getNumEqConstraints", &Problem::getNumEqConstraints)
      .def("getNumIneqConstraints", &Problem::getNumIneqConstraints)
      .def("getEqConstraint", &Problem::getEqConstraint, nb::arg("idx"))
      .def("getIneqConstraint", &Problem::getIneqConstraint, nb::arg("idx"))
      .def(
          "removeEqConstraint",
          &Problem::removeEqConstraint,
          nb::arg("constraint"))
      .def(
          "removeIneqConstraint",
          &Problem::removeIneqConstraint,
          nb::arg("constraint"))
      .def("removeAllEqConstraints", &Problem::removeAllEqConstraints)
      .def("removeAllIneqConstraints", &Problem::removeAllIneqConstraints)
      .def("setOptimumValue", &Problem::setOptimumValue, nb::arg("value"))
      .def("getOptimumValue", &Problem::getOptimumValue)
      .def(
          "setOptimalSolution",
          &Problem::setOptimalSolution,
          nb::arg("solution"))
      .def("getOptimalSolution", [](Problem& self) {
        return self.getOptimalSolution();
      });
}

DART_SUPPRESS_DEPRECATED_END

} // namespace dart::python_nb
