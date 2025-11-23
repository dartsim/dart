#include "optimizer/solver.hpp"

#include "dart/common/Diagnostics.hpp"
#include "dart/math/optimization/Problem.hpp"
#include "dart/math/optimization/Solver.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/trampoline.h>

namespace nb = nanobind;

namespace dart::python_nb {

class PySolver : public dart::math::Solver
{
public:
  NB_TRAMPOLINE(dart::math::Solver, 1);

  bool solve() override
  {
    NB_OVERRIDE_PURE(solve);
  }

  std::string getType() const override
  {
    NB_OVERRIDE_PURE(getType);
  }

  std::shared_ptr<dart::math::Solver> clone() const override
  {
    NB_OVERRIDE_PURE(clone);
  }
};

void defOptimizerSolver(nb::module_& m)
{
  using Solver = dart::math::Solver;

  nb::class_<Solver::Properties>(m, "SolverProperties")
      .def(nb::init<>())
      .def(nb::init<std::shared_ptr<dart::math::Problem>>(), nb::arg("problem"))
      .def(
          nb::init<std::shared_ptr<dart::math::Problem>, double>(),
          nb::arg("problem"),
          nb::arg("tolerance"))
      .def(
          nb::init<std::shared_ptr<dart::math::Problem>, double, std::size_t>(),
          nb::arg("problem"),
          nb::arg("tolerance"),
          nb::arg("numMaxIterations"))
      .def(
          nb::init<
              std::shared_ptr<dart::math::Problem>,
              double,
              std::size_t,
              std::size_t>(),
          nb::arg("problem"),
          nb::arg("tolerance"),
          nb::arg("numMaxIterations"),
          nb::arg("iterationsPerPrint"))
      .def(
          nb::init<
              std::shared_ptr<dart::math::Problem>,
              double,
              std::size_t,
              std::size_t,
              std::ostream*>(),
          nb::arg("problem"),
          nb::arg("tolerance"),
          nb::arg("numMaxIterations"),
          nb::arg("iterationsPerPrint"),
          nb::arg("ostream"))
      .def(
          nb::init<
              std::shared_ptr<dart::math::Problem>,
              double,
              std::size_t,
              std::size_t,
              std::ostream*,
              bool>(),
          nb::arg("problem"),
          nb::arg("tolerance"),
          nb::arg("numMaxIterations"),
          nb::arg("iterationsPerPrint"),
          nb::arg("ostream"),
          nb::arg("printFinalResult"))
      .def(
          nb::init<
              std::shared_ptr<dart::math::Problem>,
              double,
              std::size_t,
              std::size_t,
              std::ostream*,
              bool,
              const std::string&>(),
          nb::arg("problem"),
          nb::arg("tolerance"),
          nb::arg("numMaxIterations"),
          nb::arg("iterationsPerPrint"),
          nb::arg("ostream"),
          nb::arg("printFinalResult"),
          nb::arg("resultFile"))
      .def_rw("mProblem", &Solver::Properties::mProblem)
      .def_rw("mTolerance", &Solver::Properties::mTolerance)
      .def_rw("mNumMaxIterations", &Solver::Properties::mNumMaxIterations)
      .def_rw("mIterationsPerPrint", &Solver::Properties::mIterationsPerPrint)
      .def_rw("mOutStream", &Solver::Properties::mOutStream)
      .def_rw("mPrintFinalResult", &Solver::Properties::mPrintFinalResult)
      .def_rw("mResultFile", &Solver::Properties::mResultFile);

  nb::class_<Solver, PySolver>(m, "Solver")
      .def(nb::init<>())
      .def(nb::init<Solver::Properties>(), nb::arg("properties"))
      .def(nb::init<std::shared_ptr<dart::math::Problem>>(), nb::arg("problem"))
      .def("solve", &Solver::solve)
      .def("getType", &Solver::getType)
      .def("clone", &Solver::clone)
      .def("setProperties", &Solver::setProperties, nb::arg("properties"))
      .def(
          "getProperties",
          [](Solver& self) -> const Solver::Properties& {
            return self.getSolverProperties();
          },
          nb::rv_policy::reference_internal)
      .def("setProblem", &Solver::setProblem, nb::arg("problem"))
      .def("getProblem", &Solver::getProblem)
      .def("setTolerance", &Solver::setTolerance, nb::arg("tol"))
      .def("getTolerance", &Solver::getTolerance)
      .def("setNumMaxIterations", &Solver::setNumMaxIterations, nb::arg("num"))
      .def("getNumMaxIterations", &Solver::getNumMaxIterations)
      .def(
          "setIterationsPerPrint",
          &Solver::setIterationsPerPrint,
          nb::arg("ratio"))
      .def("getIterationsPerPrint", &Solver::getIterationsPerPrint)
      .def("setOutStream", &Solver::setOutStream, nb::arg("os"))
      .def(
          "setPrintFinalResult", &Solver::setPrintFinalResult, nb::arg("print"))
      .def("getPrintFinalResult", &Solver::getPrintFinalResult)
      .def(
          "setResultFileName",
          &Solver::setResultFileName,
          nb::arg("resultFile"))
      .def(
          "getResultFileName",
          &Solver::getResultFileName,
          nb::rv_policy::reference_internal);
}

} // namespace dart::python_nb
