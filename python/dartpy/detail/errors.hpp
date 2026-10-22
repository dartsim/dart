#pragma once

#include <nanobind/nanobind.h>

namespace dartnb {
// dartpy.errors.OperationError, the DartError branch under RuntimeError. Use it
// as the base of nanobind exceptions that wrap DART's std::runtime_error types.
nanobind::handle operationError();
} // namespace dartnb
