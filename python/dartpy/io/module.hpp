#pragma once

#include <nanobind/nanobind.h>

namespace dart::python_nb {

void defIoRead(nanobind::module_& m);

void defIoModule(nanobind::module_& m);

} // namespace dart::python_nb
