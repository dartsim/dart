#include "utils/sdf_parser.hpp"

#include "dart/utils/sdf/sdf_parser.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defSdfParser(nb::module_& m)
{
  namespace SdfParserNs = ::dart::utils::SdfParser;

  auto sm = m.def_submodule("SdfParser");

  auto root_joint_type
      = nb::enum_<SdfParserNs::RootJointType>(sm, "RootJointType")
            .value("Floating", SdfParserNs::RootJointType::Floating)
            .value("Fixed", SdfParserNs::RootJointType::Fixed);
  root_joint_type.attr("FLOATING") = root_joint_type.attr("Floating");
  root_joint_type.attr("FIXED") = root_joint_type.attr("Fixed");

  nb::class_<SdfParserNs::Options>(sm, "Options")
      .def(
          nb::init<common::ResourceRetrieverPtr, SdfParserNs::RootJointType>(),
          nb::arg("resource_retriever") = nullptr,
          nb::arg("default_root_joint_type")
          = SdfParserNs::RootJointType::Floating)
      .def_rw("mResourceRetriever", &SdfParserNs::Options::mResourceRetriever)
      .def_rw(
          "mDefaultRootJointType",
          &SdfParserNs::Options::mDefaultRootJointType);

  auto read_world = [](auto uri, const SdfParserNs::Options& options) {
    return SdfParserNs::readWorld(uri, options);
  };
  auto read_skeleton = [](auto uri, const SdfParserNs::Options& options) {
    return SdfParserNs::readSkeleton(uri, options);
  };

  sm.def(
      "readWorld",
      [=](const common::Uri& uri, const SdfParserNs::Options& options) {
        return read_world(uri, options);
      },
      nb::arg("uri"),
      nb::arg("options") = SdfParserNs::Options());
  sm.def(
      "readWorld",
      [=](const std::string& uri, const SdfParserNs::Options& options) {
        return read_world(common::Uri(uri), options);
      },
      nb::arg("uri"),
      nb::arg("options") = SdfParserNs::Options());
  sm.def(
      "readSkeleton",
      [=](const common::Uri& uri, const SdfParserNs::Options& options) {
        return read_skeleton(uri, options);
      },
      nb::arg("uri"),
      nb::arg("options") = SdfParserNs::Options());
  sm.def(
      "readSkeleton",
      [=](const std::string& uri, const SdfParserNs::Options& options) {
        return read_skeleton(common::Uri(uri), options);
      },
      nb::arg("uri"),
      nb::arg("options") = SdfParserNs::Options());
}

} // namespace dart::python_nb
