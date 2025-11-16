#include "utils/dart_loader.hpp"

#include "dart/dynamics/Inertia.hpp"
#include "dart/utils/urdf/DartLoader.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defDartLoader(nb::module_& m)
{
  using DartLoader = ::dart::utils::DartLoader;

  auto root_joint_type
      = nb::enum_<DartLoader::RootJointType>(m, "DartLoaderRootJointType")
            .value("FLOATING", DartLoader::RootJointType::Floating)
            .value("FIXED", DartLoader::RootJointType::Fixed);

  auto options
      = nb::class_<DartLoader::Options>(m, "DartLoaderOptions")
            .def(
                nb::init<
                    common::ResourceRetrieverPtr,
                    DartLoader::RootJointType,
                    const dynamics::Inertia&>(),
                nb::arg("resourceRetriever") = nullptr,
                nb::arg("defaultRootJointType")
                = DartLoader::RootJointType::Floating,
                nb::arg("defaultInertia") = dynamics::Inertia())
            .def_rw(
                "mResourceRetriever", &DartLoader::Options::mResourceRetriever)
            .def_rw(
                "mDefaultRootJointType",
                &DartLoader::Options::mDefaultRootJointType)
            .def_rw("mDefaultInertia", &DartLoader::Options::mDefaultInertia);

  auto cls = nb::class_<DartLoader>(m, "DartLoader")
                 .def(nb::init<>())
                 .def(
                     "setOptions",
                     &DartLoader::setOptions,
                     nb::arg("options") = DartLoader::Options())
                 .def("getOptions", &DartLoader::getOptions)
                 .def(
                     "addPackageDirectory",
                     &DartLoader::addPackageDirectory,
                     nb::arg("packageName"),
                     nb::arg("packageDirectory"))
                 .def(
                     "parseSkeleton",
                     [](DartLoader& self, const common::Uri& uri) {
                       return self.parseSkeleton(uri);
                     },
                     nb::arg("uri"))
                 .def(
                     "parseSkeleton",
                     [](DartLoader& self, const std::string& uri) {
                       return self.parseSkeleton(common::Uri(uri));
                     },
                     nb::arg("uri"))
                 .def(
                     "parseSkeletonString",
                     [](DartLoader& self,
                        const std::string& data,
                        const common::Uri& base) {
                       return self.parseSkeletonString(data, base);
                     },
                     nb::arg("urdfString"),
                     nb::arg("baseUri"))
                 .def(
                     "parseWorld",
                     [](DartLoader& self, const common::Uri& uri) {
                       return self.parseWorld(uri);
                     },
                     nb::arg("uri"))
                 .def(
                     "parseWorld",
                     [](DartLoader& self, const std::string& uri) {
                       return self.parseWorld(common::Uri(uri));
                     },
                     nb::arg("uri"))
                 .def(
                     "parseWorldString",
                     [](DartLoader& self,
                        const std::string& data,
                        const common::Uri& base) {
                       return self.parseWorldString(data, base);
                     },
                     nb::arg("urdfString"),
                     nb::arg("baseUri"));

  cls.attr("RootJointType") = root_joint_type;
  cls.attr("Options") = options;
}

} // namespace dart::python_nb
