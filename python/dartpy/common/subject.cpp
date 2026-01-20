#include "common/subject.hpp"

#include "dart/common/subject.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defSubject(nb::module_& m)
{
  nb::class_<dart::common::Subject>(m, "Subject");
}

} // namespace dart::python_nb
