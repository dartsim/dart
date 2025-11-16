#include "utils/mjcf_parser.hpp"

#include "dart/utils/mjcf/MjcfParser.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defMjcfParser(nb::module_& m)
{
  using MjcfParser = utils::MjcfParser;

  auto sm = m.def_submodule("MjcfParser");

  nb::class_<MjcfParser::Options>(sm, "Options")
      .def(
          nb::init<
              const common::ResourceRetrieverPtr&,
              const std::string&,
              const std::string&>(),
          nb::arg("resourceRetretrieverOrNullptrriever") = nullptr,
          nb::arg("geomSkeletonNamePrefix") = "__geom_skel__",
          nb::arg("siteSkeletonNamePrefix") = "__site_skel__")
      .def_rw("mRetriever", &MjcfParser::Options::mRetriever)
      .def_rw(
          "mGeomSkeletonNamePrefix",
          &MjcfParser::Options::mGeomSkeletonNamePrefix)
      .def_rw(
          "mSiteSkeletonNamePrefix",
          &MjcfParser::Options::mSiteSkeletonNamePrefix);

  sm.def(
      "readWorld",
      [](const common::Uri& uri, const MjcfParser::Options& options) {
        return MjcfParser::readWorld(uri, options);
      },
      nb::arg("uri"),
      nb::arg("options") = MjcfParser::Options());
  sm.def(
      "readWorld",
      [](const std::string& uri, const MjcfParser::Options& options) {
        return MjcfParser::readWorld(common::Uri(uri), options);
      },
      nb::arg("uri"),
      nb::arg("options") = MjcfParser::Options());
}

} // namespace dart::python_nb
