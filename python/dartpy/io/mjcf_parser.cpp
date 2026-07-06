#include "io/mjcf_parser.hpp"

#include "dart/io/mjcf/mjcf_parser.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defMjcfParser(nb::module_& m)
{
  auto sm = m.def_submodule("MjcfParser");

  nb::class_<::dart::io::MjcfParser::Options>(sm, "Options")
      .def(
          nb::init<
              const common::ResourceRetrieverPtr&,
              const std::string&,
              const std::string&>(),
          nb::arg("resource_retriever") = nullptr,
          nb::arg("geom_skeleton_name_prefix") = "__geom_skel__",
          nb::arg("site_skeleton_name_prefix") = "__site_skel__")
      .def_rw("mRetriever", &::dart::io::MjcfParser::Options::mRetriever)
      .def_rw(
          "mGeomSkeletonNamePrefix",
          &::dart::io::MjcfParser::Options::mGeomSkeletonNamePrefix)
      .def_rw(
          "mSiteSkeletonNamePrefix",
          &::dart::io::MjcfParser::Options::mSiteSkeletonNamePrefix);
}

} // namespace dart::python_nb
