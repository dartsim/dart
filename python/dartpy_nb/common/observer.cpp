#include "common/observer.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "dart/common/Observer.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defObserver(nb::module_& m)
{
  nb::class_<dart::common::Observer, std::shared_ptr<dart::common::Observer>>(m, "Observer");
}

} // namespace dart::python_nb
