#include "utils/resource_retriever.hpp"

#include "dart/utils/composite_resource_retriever.hpp"
#include "dart/utils/dart_resource_retriever.hpp"
#include "dart/utils/package_resource_retriever.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defUtilsResourceRetriever(nb::module_& m)
{
  using CompositeRetriever = ::dart::utils::CompositeResourceRetriever;
  nb::class_<CompositeRetriever, ::dart::common::ResourceRetriever>(
      m, "CompositeResourceRetriever")
      .def(nb::init<>())
      .def(
          "addDefaultRetriever",
          &CompositeRetriever::addDefaultRetriever,
          nb::arg("resource_retriever"))
      .def(
          "addSchemaRetriever",
          &CompositeRetriever::addSchemaRetriever,
          nb::arg("schema"),
          nb::arg("resource_retriever"));

  using DartRetriever = ::dart::utils::DartResourceRetriever;
  nb::class_<DartRetriever, ::dart::common::ResourceRetriever>(
      m, "DartResourceRetriever")
      .def(nb::init<>());

  using PackageRetriever = ::dart::utils::PackageResourceRetriever;
  nb::class_<PackageRetriever, ::dart::common::ResourceRetriever>(
      m, "PackageResourceRetriever")
      .def(
          nb::init<const ::dart::common::ResourceRetrieverPtr&>(),
          nb::arg("local_retriever"))
      .def(
          "addPackageDirectory",
          &PackageRetriever::addPackageDirectory,
          nb::arg("package_name"),
          nb::arg("package_directory"));
}

} // namespace dart::python_nb
