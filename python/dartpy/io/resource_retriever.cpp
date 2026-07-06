#include "io/resource_retriever.hpp"

#include "dart/io/composite_resource_retriever.hpp"
#include "dart/io/dart_resource_retriever.hpp"
#include "dart/io/package_resource_retriever.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defIoResourceRetriever(nb::module_& m)
{
  using CompositeRetriever = ::dart::io::CompositeResourceRetriever;
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

  using DartRetriever = ::dart::io::DartResourceRetriever;
  nb::class_<DartRetriever, ::dart::common::ResourceRetriever>(
      m, "DartResourceRetriever")
      .def(nb::init<>());

  using PackageRetriever = ::dart::io::PackageResourceRetriever;
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
