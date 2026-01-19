#include "simulation_experimental/state_space.hpp"

#include <dart/simulation/experimental/space/state_space.hpp>

#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace dart::simulation::experimental;

namespace dart::python_nb {

void defStateSpace(nb::module_& m)
{
  nb::class_<StateSpace::Variable>(m, "StateSpaceVariable")
      .def_ro("name", &StateSpace::Variable::name)
      .def_ro("start_index", &StateSpace::Variable::startIndex)
      .def_ro("dimension", &StateSpace::Variable::dimension)
      .def_ro("lower_bound", &StateSpace::Variable::lowerBound)
      .def_ro("upper_bound", &StateSpace::Variable::upperBound);

  nb::class_<StateSpace>(m, "StateSpace")
      .def(nb::init<>())
      .def(
          "add_variable",
          &StateSpace::addVariable,
          nb::arg("name"),
          nb::arg("dimension"),
          nb::arg("lower") = -std::numeric_limits<double>::infinity(),
          nb::arg("upper") = std::numeric_limits<double>::infinity(),
          nb::rv_policy::reference)
      .def("finalize", &StateSpace::finalize)
      .def("is_finalized", &StateSpace::isFinalized)
      .def("get_dimension", &StateSpace::getDimension)
      .def("get_num_variables", &StateSpace::getNumVariables)
      .def("get_variables", &StateSpace::getVariables)
      .def("get_variable", &StateSpace::getVariable, nb::arg("name"))
      .def("get_variable_index", &StateSpace::getVariableIndex, nb::arg("name"))
      .def("get_variable_names", &StateSpace::getVariableNames)
      .def("get_lower_bounds", &StateSpace::getLowerBounds)
      .def("get_upper_bounds", &StateSpace::getUpperBounds)
      .def("has_variable", &StateSpace::hasVariable, nb::arg("name"));
}

} // namespace dart::python_nb
