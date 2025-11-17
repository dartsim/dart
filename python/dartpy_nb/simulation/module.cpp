#include "simulation/module.hpp"

#include "simulation/constraint_solver.hpp"
#include "simulation/world.hpp"

namespace dart::python_nb {

void defSimulationModule(nanobind::module_& m)
{
  defWorld(m);
  defConstraintSolver(m);
}

} // namespace dart::python_nb
