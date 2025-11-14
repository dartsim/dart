#include "simulation/module.hpp"

#include "simulation/constraint_solver.hpp"
#include "simulation/world.hpp"

namespace dart::python_nb {

void defSimulationModule(nanobind::module_& m)
{
  auto simulation = m.def_submodule("simulation");
  defWorld(simulation);
  defConstraintSolver(simulation);
}

} // namespace dart::python_nb
