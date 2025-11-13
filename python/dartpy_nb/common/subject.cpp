#include "common/subject.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "dart/common/Subject.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defSubject(nb::module_& m)
{
  nb::class_<dart::common::Subject, std::shared_ptr<dart::common::Subject>>(m, "Subject");
}

} // namespace dart::python_nb
