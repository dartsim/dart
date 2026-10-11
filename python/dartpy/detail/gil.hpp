#pragma once

#include <nanobind/nanobind.h>

#include <stdexcept>

namespace dartnb {
// Attaches a Python thread state for a callback. Raises instead of touching
// Python, or substituting a C++ result, when Python 3.15+ refuses attachment
// during finalization.
struct GilScopedAcquire : nanobind::gil_scoped_acquire
{
  GilScopedAcquire()
  {
    if (!is_valid())
      throw std::runtime_error("dartpy: Python interpreter is finalizing");
  }
};
} // namespace dartnb
