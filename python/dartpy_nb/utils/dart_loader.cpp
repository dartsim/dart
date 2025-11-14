#include "utils/dart_loader.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "dart/dynamics/Inertia.hpp"
#include "dart/utils/urdf/DartLoader.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defDartLoader(nb::module_& m)
{
  using DartLoader = utils::DartLoader;

  auto root_joint_type = nb::enum_<DartLoader::RootJointType>(m, "DartLoaderRootJointType")
                             .value("FLOATING", DartLoader::RootJointType::FLOATING)
                             .value("FIXED", DartLoader::RootJointType::FIXED);

  auto options = nb::class_<DartLoader::Options>(m, "DartLoaderOptions")
                     .def(
                         nb::init<
                             common::ResourceRetrieverPtr,
                             DartLoader::RootJointType,
                             const dynamics::Inertia&>(),
                         nb::arg("resourceRetriever") = nullptr,
                         nb::arg("defaultRootJointType") = DartLoader::RootJointType::FLOATING,
                         nb::arg("defaultInertia") = dynamics::Inertia())
                     .def_readwrite("mResourceRetriever", &DartLoader::Options::mResourceRetriever)
                     .def_readwrite("mDefaultRootJointType", &DartLoader::Options::mDefaultRootJointType)
                     .def_readwrite("mDefaultInertia", &DartLoader::Options::mDefaultInertia);

  auto cls = nb::class_<DartLoader>(m, "DartLoader")
                 .def(nb::init<>())
                 .def("setOptions", &DartLoader::setOptions, nb::arg("options") = DartLoader::Options())
                 .def("getOptions", &DartLoader::getOptions)
                 .def(
                     "addPackageDirectory",
                     &DartLoader::addPackageDirectory,
                     nb::arg("packageName"),
                     nb::arg("packageDirectory"))
                 .def(
                     "parseSkeleton",
                     [](DartLoader& self, const common::Uri& uri) { return self.parseSkeleton(uri); },
                     nb::arg("uri"))
                 .def(
                     "parseSkeletonString",
                     [](DartLoader& self, const std::string& data, const common::Uri& base) {
                       return self.parseSkeletonString(data, base);
                     },
                     nb::arg("urdfString"),
                     nb::arg("baseUri"))
                 .def(
                     "parseWorld",
                     [](DartLoader& self, const common::Uri& uri) { return self.parseWorld(uri); },
                     nb::arg("uri"))
                 .def(
                     "parseWorldString",
                     [](DartLoader& self, const std::string& data, const common::Uri& base) {
                       return self.parseWorldString(data, base);
                     },
                     nb::arg("urdfString"),
                     nb::arg("baseUri"));

  cls.attr("RootJointType") = root_joint_type;
  cls.attr("Options") = options;
}

} // namespace dart::python_nb
