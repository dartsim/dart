#include "utils/mjcf_parser.hpp"

#include "dart/utils/mjcf/mjcf_parser.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defMjcfParser(nb::module_& m)
{
  auto sm = m.def_submodule("MjcfParser");

  nb::class_<::dart::utils::MjcfParser::Options>(sm, "Options")
      .def(
          nb::init<
              const common::ResourceRetrieverPtr&,
              const std::string&,
              const std::string&>(),
          nb::arg("resource_retriever") = nullptr,
          nb::arg("geom_skeleton_name_prefix") = "__geom_skel__",
          nb::arg("site_skeleton_name_prefix") = "__site_skel__")
      .def_rw("mRetriever", &::dart::utils::MjcfParser::Options::mRetriever)
      .def_rw(
          "mGeomSkeletonNamePrefix",
          &::dart::utils::MjcfParser::Options::mGeomSkeletonNamePrefix)
      .def_rw(
          "mSiteSkeletonNamePrefix",
          &::dart::utils::MjcfParser::Options::mSiteSkeletonNamePrefix);

  sm.def(
      "readWorld",
      [](const common::Uri& uri,
         const ::dart::utils::MjcfParser::Options& options) {
        return ::dart::utils::MjcfParser::readWorld(uri, options);
      },
      nb::arg("uri"),
      nb::arg("options") = ::dart::utils::MjcfParser::Options());
  sm.def(
      "readWorld",
      [](const std::string& uri,
         const ::dart::utils::MjcfParser::Options& options) {
        return ::dart::utils::MjcfParser::readWorld(common::Uri(uri), options);
      },
      nb::arg("uri"),
      nb::arg("options") = ::dart::utils::MjcfParser::Options());
}

} // namespace dart::python_nb
