#include "optimizer/module.hpp"

#include "optimizer/function.hpp"
#include "optimizer/gradient_descent.hpp"
#include "optimizer/problem.hpp"
#include "optimizer/solver.hpp"

namespace dart::python_nb {

void defOptimizerModule(nanobind::module_& m)
{
  auto sm = m.def_submodule("optimizer");
  defOptimizerFunction(sm);
  defOptimizerProblem(sm);
  defOptimizerSolver(sm);
  defGradientDescentSolver(sm);
}

} // namespace dart::python_nb
