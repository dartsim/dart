#include "common/resource_retriever.hpp"

#include "dart/common/diagnostics.hpp"
#include "dart/common/local_resource_retriever.hpp"
#include "dart/common/resource_retriever.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defResourceRetriever(nb::module_& m)
{
  using ResourceRetriever = dart::common::ResourceRetriever;

  DART_SUPPRESS_DEPRECATED_BEGIN
  nb::class_<ResourceRetriever>(m, "ResourceRetriever")
      .def("exists", &ResourceRetriever::exists, nb::arg("uri"))
      .def("retrieve", &ResourceRetriever::retrieve, nb::arg("uri"))
      .def("readAll", &ResourceRetriever::readAll, nb::arg("uri"))
      .def("getFilePath", &ResourceRetriever::getFilePath, nb::arg("uri"));
  DART_SUPPRESS_DEPRECATED_END

  using LocalResourceRetriever = dart::common::LocalResourceRetriever;
  nb::class_<LocalResourceRetriever, ResourceRetriever>(
      m, "LocalResourceRetriever")
      .def(nb::init<>());
}

} // namespace dart::python_nb
