#include "utils/urdf_parser.hpp"

#include "dart/dynamics/inertia.hpp"
#include "dart/utils/urdf/urdf_parser.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defUrdfParser(nb::module_& m)
{
  using UrdfParser = ::dart::utils::UrdfParser;

  auto root_joint_type
      = nb::enum_<UrdfParser::RootJointType>(m, "UrdfParserRootJointType")
            .value("Floating", UrdfParser::RootJointType::Floating)
            .value("Fixed", UrdfParser::RootJointType::Fixed);
  root_joint_type.attr("FLOATING") = root_joint_type.attr("Floating");
  root_joint_type.attr("FIXED") = root_joint_type.attr("Fixed");

  auto options
      = nb::class_<UrdfParser::Options>(m, "UrdfParserOptions")
            .def(
                nb::init<
                    common::ResourceRetrieverPtr,
                    UrdfParser::RootJointType,
                    const dynamics::Inertia&>(),
                nb::arg("resource_retriever") = nullptr,
                nb::arg("default_root_joint_type")
                = UrdfParser::RootJointType::Floating,
                nb::arg("default_inertia") = dynamics::Inertia())
            .def_rw(
                "mResourceRetriever", &UrdfParser::Options::mResourceRetriever)
            .def_rw(
                "mDefaultRootJointType",
                &UrdfParser::Options::mDefaultRootJointType)
            .def_rw("mDefaultInertia", &UrdfParser::Options::mDefaultInertia);

  auto cls = nb::class_<UrdfParser>(m, "UrdfParser")
                 .def(nb::init<>())
                 .def(
                     "setOptions",
                     &UrdfParser::setOptions,
                     nb::arg("options") = UrdfParser::Options())
                 .def("getOptions", &UrdfParser::getOptions)
                 .def(
                     "addPackageDirectory",
                     &UrdfParser::addPackageDirectory,
                     nb::arg("package_name"),
                     nb::arg("package_directory"))
                 .def(
                     "parseSkeleton",
                     [](UrdfParser& self, const common::Uri& uri) {
                       return self.parseSkeleton(uri);
                     },
                     nb::arg("uri"))
                 .def(
                     "parseSkeleton",
                     [](UrdfParser& self, const std::string& uri) {
                       return self.parseSkeleton(common::Uri(uri));
                     },
                     nb::arg("uri"))
                 .def(
                     "parseSkeletonString",
                     [](UrdfParser& self,
                        const std::string& data,
                        const common::Uri& base) {
                       return self.parseSkeletonString(data, base);
                     },
                     nb::arg("urdf_string"),
                     nb::arg("base_uri"))
                 .def(
                     "parseWorld",
                     [](UrdfParser& self, const common::Uri& uri) {
                       return self.parseWorld(uri);
                     },
                     nb::arg("uri"))
                 .def(
                     "parseWorld",
                     [](UrdfParser& self, const std::string& uri) {
                       return self.parseWorld(common::Uri(uri));
                     },
                     nb::arg("uri"))
                 .def(
                     "parseWorldString",
                     [](UrdfParser& self,
                        const std::string& data,
                        const common::Uri& base) {
                       return self.parseWorldString(data, base);
                     },
                     nb::arg("urdf_string"),
                     nb::arg("base_uri"));

  cls.attr("RootJointType") = root_joint_type;
  cls.attr("Options") = options;

  // Backward compatibility aliases
  m.attr("DartLoader") = cls;
  m.attr("DartLoaderOptions") = options;
  m.attr("DartLoaderRootJointType") = root_joint_type;
}

} // namespace dart::python_nb
