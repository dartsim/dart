#pragma once

#include <nanobind/nanobind.h>

namespace dart::python_nb {

/// Define StateSpace Python bindings
void defStateSpace(nanobind::module_& m);

} // namespace dart::python_nb
