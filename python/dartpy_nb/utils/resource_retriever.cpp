#include "utils/resource_retriever.hpp"

#include "dart/utils/CompositeResourceRetriever.hpp"
#include "dart/utils/DartResourceRetriever.hpp"
#include "dart/utils/PackageResourceRetriever.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defUtilsResourceRetriever(nb::module_& m)
{
  using CompositeRetriever = utils::CompositeResourceRetriever;
  nb::class_<CompositeRetriever, common::ResourceRetriever>(
      m, "CompositeResourceRetriever")
      .def(nb::init<>())
      .def(
          "addDefaultRetriever",
          &CompositeRetriever::addDefaultRetriever,
          nb::arg("resourceRetriever"))
      .def(
          "addSchemaRetriever",
          &CompositeRetriever::addSchemaRetriever,
          nb::arg("schema"),
          nb::arg("resourceRetriever"));

  using DartRetriever = utils::DartResourceRetriever;
  nb::class_<DartRetriever, common::ResourceRetriever>(
      m, "DartResourceRetriever")
      .def(nb::init<>());

  using PackageRetriever = utils::PackageResourceRetriever;
  nb::class_<PackageRetriever, common::ResourceRetriever>(
      m, "PackageResourceRetriever")
      .def(
          nb::init<const common::ResourceRetrieverPtr&>(),
          nb::arg("localRetriever"))
      .def(
          "addPackageDirectory",
          &PackageRetriever::addPackageDirectory,
          nb::arg("packageName"),
          nb::arg("packageDirectory"));
}

} // namespace dart::python_nb
