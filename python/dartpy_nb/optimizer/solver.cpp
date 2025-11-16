#include "optimizer/solver.hpp"

#include "dart/optimizer/Solver.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/ostream.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

class PySolver : public dart::optimizer::Solver
{
public:
  using dart::optimizer::Solver::Solver;

  bool solve() override
  {
    NB_OVERRIDE_PURE(bool, dart::optimizer::Solver, solve);
  }

  std::string getType() const override
  {
    NB_OVERRIDE_PURE(std::string, dart::optimizer::Solver, getType);
  }

  std::shared_ptr<dart::optimizer::Solver> clone() const override
  {
    NB_OVERRIDE_PURE(
        std::shared_ptr<dart::optimizer::Solver>,
        dart::optimizer::Solver,
        clone);
  }
};

void defOptimizerSolver(nb::module_& m)
{
  using Solver = dart::optimizer::Solver;

  nb::class_<Solver::Properties>(m, "SolverProperties")
      .def(nb::init<>())
      .def(
          nb::init<std::shared_ptr<dart::optimizer::Problem>>(),
          nb::arg("problem"))
      .def(
          nb::init<std::shared_ptr<dart::optimizer::Problem>, double>(),
          nb::arg("problem"),
          nb::arg("tolerance"))
      .def(
          nb::init<
              std::shared_ptr<dart::optimizer::Problem>,
              double,
              std::size_t>(),
          nb::arg("problem"),
          nb::arg("tolerance"),
          nb::arg("numMaxIterations"))
      .def(
          nb::init<
              std::shared_ptr<dart::optimizer::Problem>,
              double,
              std::size_t,
              std::size_t>(),
          nb::arg("problem"),
          nb::arg("tolerance"),
          nb::arg("numMaxIterations"),
          nb::arg("iterationsPerPrint"))
      .def_rw("mProblem", &Solver::Properties::mProblem)
      .def_rw("mTolerance", &Solver::Properties::mTolerance)
      .def_rw("mNumMaxIterations", &Solver::Properties::mNumMaxIterations)
      .def_rw("mIterationsPerPrint", &Solver::Properties::mIterationsPerPrint);

  nb::class_<Solver, PySolver>(m, "Solver")
      .def(nb::init<>())
      .def(nb::init<Solver::Properties>(), nb::arg("properties"))
      .def(
          nb::init<std::shared_ptr<dart::optimizer::Problem>>(),
          nb::arg("problem"))
      .def("solve", &Solver::solve)
      .def("getType", &Solver::getType)
      .def("clone", &Solver::clone)
      .def("setProperties", &Solver::setProperties, nb::arg("properties"))
      .def("getProperties", &Solver::getProperties)
      .def("setProblem", &Solver::setProblem, nb::arg("problem"))
      .def("getProblem", &Solver::getProblem)
      .def("setTolerance", &Solver::setTolerance, nb::arg("tol"))
      .def("getTolerance", &Solver::getTolerance)
      .def("setNumMaxIterations", &Solver::setNumMaxIterations, nb::arg("num"))
      .def("getNumMaxIterations", &Solver::getNumMaxIterations);
}

} // namespace dart::python_nb
