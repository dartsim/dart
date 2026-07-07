#include "io/read.hpp"

#include "dart/common/uri.hpp"
#include "dart/io/read.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defIoRead(nb::module_& m)
{
  using ::dart::io::ModelFormat;
  using ::dart::io::ReadOptions;
  using ::dart::io::RootJointType;

  nb::enum_<ModelFormat>(m, "ModelFormat")
      .value("AUTO", ModelFormat::Auto)
      .value("SDF", ModelFormat::Sdf)
      .value("URDF", ModelFormat::Urdf)
      .value("MJCF", ModelFormat::Mjcf)
      .value("USD", ModelFormat::Usd);

  nb::enum_<RootJointType>(m, "RootJointType")
      .value("FLOATING", RootJointType::Floating)
      .value("FIXED", RootJointType::Fixed);

  nb::class_<ReadOptions>(m, "ReadOptions")
      .def(nb::init<>())
      .def_rw("format", &ReadOptions::format)
      .def_rw("resourceRetriever", &ReadOptions::resourceRetriever)
      .def_rw("sdfDefaultRootJointType", &ReadOptions::sdfDefaultRootJointType)
      .def(
          "addPackageDirectory",
          [](ReadOptions& self,
             const std::string& packageName,
             const std::string& packageDirectory) {
            self.addPackageDirectory(packageName, packageDirectory);
          },
          nb::arg("package_name"),
          nb::arg("package_directory"));

  m.def(
      "readSkeleton",
      [](const common::Uri& uri, const ReadOptions& options) {
        return ::dart::io::readSkeleton(uri, options);
      },
      nb::arg("uri"),
      nb::arg("options") = ReadOptions());
  m.def(
      "readSkeleton",
      [](const std::string& uri, const ReadOptions& options) {
        return ::dart::io::readSkeleton(common::Uri(uri), options);
      },
      nb::arg("uri"),
      nb::arg("options") = ReadOptions());
}

} // namespace dart::python_nb
