#include "common/resource_retriever.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/ResourceRetriever.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defResourceRetriever(nb::module_& m)
{
  using ResourceRetriever = dart::common::ResourceRetriever;

  nb::class_<ResourceRetriever, std::shared_ptr<ResourceRetriever>>(m, "ResourceRetriever")
      .def("exists", &ResourceRetriever::exists, nb::arg("uri"))
      .def("retrieve", &ResourceRetriever::retrieve, nb::arg("uri"))
      .def("readAll", &ResourceRetriever::readAll, nb::arg("uri"))
      .def("getFilePath", &ResourceRetriever::getFilePath, nb::arg("uri"));

  using LocalResourceRetriever = dart::common::LocalResourceRetriever;
  nb::class_<LocalResourceRetriever, ResourceRetriever, std::shared_ptr<LocalResourceRetriever>>(m, "LocalResourceRetriever")
      .def(nb::init<>());
}

} // namespace dart::python_nb
