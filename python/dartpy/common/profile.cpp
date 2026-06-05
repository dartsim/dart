#include "common/profile.hpp"

#include <dart/common/profile.hpp>

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defProfile(nb::module_& m)
{
  m.def(
      "isProfileEnabled",
      &dart::common::profile::isProfilingEnabled,
      "Return whether DART was built with profiling support.");
  m.def(
      "isTextProfileEnabled",
      &dart::common::profile::isTextProfilingEnabled,
      "Return whether the built-in text profiling backend is available.");
  m.def(
      "markProfileFrame",
      &dart::common::profile::markProfileFrame,
      "Mark one frame in the built-in text profiler.");
  m.def(
      "resetProfile",
      &dart::common::profile::resetProfile,
      "Clear the built-in text profiler state.");
  m.def(
      "getProfileSummaryText",
      &dart::common::profile::getProfileSummaryText,
      "Return the built-in text profiler summary.");
}

} // namespace dart::python_nb
