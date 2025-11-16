#include "utils/sdf_parser.hpp"

#include "dart/utils/sdf/SdfParser.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defSdfParser(nb::module_& m)
{
  using SdfParser = ::dart::utils::SdfParser;

  auto sm = m.def_submodule("SdfParser");

  nb::enum_<SdfParser::RootJointType>(sm, "RootJointType")
      .value("FLOATING", SdfParser::RootJointType::FLOATING)
      .value("FIXED", SdfParser::RootJointType::FIXED);

  nb::class_<SdfParser::Options>(sm, "Options")
      .def(
          nb::init<common::ResourceRetrieverPtr, SdfParser::RootJointType>(),
          nb::arg("resourceRetriever") = nullptr,
          nb::arg("defaultRootJointType") = SdfParser::RootJointType::FLOATING)
      .def_rw("mResourceRetriever", &SdfParser::Options::mResourceRetriever)
      .def_rw(
          "mDefaultRootJointType", &SdfParser::Options::mDefaultRootJointType);

  auto read_world = [](auto uri, const SdfParser::Options& options) {
    return SdfParser::readWorld(uri, options);
  };
  auto read_skeleton = [](auto uri, const SdfParser::Options& options) {
    return SdfParser::readSkeleton(uri, options);
  };

  sm.def(
      "readWorld",
      [=](const common::Uri& uri, const SdfParser::Options& options) {
        return read_world(uri, options);
      },
      nb::arg("uri"),
      nb::arg("options") = SdfParser::Options());
  sm.def(
      "readWorld",
      [=](const std::string& uri, const SdfParser::Options& options) {
        return read_world(common::Uri(uri), options);
      },
      nb::arg("uri"),
      nb::arg("options") = SdfParser::Options());
  sm.def(
      "readSkeleton",
      [=](const common::Uri& uri, const SdfParser::Options& options) {
        return read_skeleton(uri, options);
      },
      nb::arg("uri"),
      nb::arg("options") = SdfParser::Options());
  sm.def(
      "readSkeleton",
      [=](const std::string& uri, const SdfParser::Options& options) {
        return read_skeleton(common::Uri(uri), options);
      },
      nb::arg("uri"),
      nb::arg("options") = SdfParser::Options());
}

} // namespace dart::python_nb
