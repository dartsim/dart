#include "optimizer/module.hpp"

#include "optimizer/function.hpp"
#include "optimizer/gradient_descent.hpp"
#include "optimizer/problem.hpp"
#include "optimizer/solver.hpp"

namespace dart::python_nb {

void defOptimizerModule(nanobind::module_& m)
{
  defOptimizerFunction(m);
  defOptimizerProblem(m);
  defOptimizerSolver(m);
  defGradientDescentSolver(m);
}

} // namespace dart::python_nb
