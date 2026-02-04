#include "optimizer/solver.hpp"

#include "dart/common/diagnostics.hpp"
#include "dart/math/optimization/problem.hpp"
#include "dart/math/optimization/solver.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>
#include <nanobind/trampoline.h>

#include <string>

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

  std::string_view getType() const override
  {
    // Cache the Python string so the view stays valid after the override.
    if (!mTypeCacheInitialized) {
      nb::detail::ticket nb_ticket(nb_trampoline, "getType", true);
      mTypeCache
          = nb::cast<std::string>(nb_trampoline.base().attr(nb_ticket.key)());
      mTypeCacheInitialized = true;
    }
    return mTypeCache;
  }

  std::shared_ptr<dart::math::Solver> clone() const override
  {
    NB_OVERRIDE_PURE(clone);
  }

private:
  mutable std::string mTypeCache;
  mutable bool mTypeCacheInitialized = false;
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
          nb::arg("num_max_iterations"))
      .def(
          nb::init<
              std::shared_ptr<dart::math::Problem>,
              double,
              std::size_t,
              std::size_t>(),
          nb::arg("problem"),
          nb::arg("tolerance"),
          nb::arg("num_max_iterations"),
          nb::arg("iterations_per_print"))
      .def(
          nb::init<
              std::shared_ptr<dart::math::Problem>,
              double,
              std::size_t,
              std::size_t,
              std::ostream*>(),
          nb::arg("problem"),
          nb::arg("tolerance"),
          nb::arg("num_max_iterations"),
          nb::arg("iterations_per_print"),
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
          nb::arg("num_max_iterations"),
          nb::arg("iterations_per_print"),
          nb::arg("ostream"),
          nb::arg("print_final_result"))
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
          nb::arg("num_max_iterations"),
          nb::arg("iterations_per_print"),
          nb::arg("ostream"),
          nb::arg("print_final_result"),
          nb::arg("result_file"))
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
          nb::arg("result_file"))
      .def(
          "getResultFileName",
          &Solver::getResultFileName,
          nb::rv_policy::reference_internal);

  m.def(
      "get_solver_type_wrapper",
      [](const Solver* solver) -> std::string_view {
        if (!solver) {
          return "nullptr";
        }
        return solver->getType();
      },
      nb::arg("solver"));
}

} // namespace dart::python_nb
