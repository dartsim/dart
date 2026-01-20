#include "common/observer.hpp"

#include "dart/common/observer.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defObserver(nb::module_& m)
{
  nb::class_<dart::common::Observer>(m, "Observer");
}

} // namespace dart::python_nb
