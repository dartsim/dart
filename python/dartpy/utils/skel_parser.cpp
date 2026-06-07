#include "utils/skel_parser.hpp"

#include "dart/utils/skel_parser.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defSkelParser(nb::module_& m)
{
  namespace SkelParser = ::dart::utils::SkelParser;

  auto sm = m.def_submodule("SkelParser");

  sm.def(
      "readSkeleton",
      [](const common::Uri& uri,
         const common::ResourceRetrieverPtr& retriever) {
        return SkelParser::readSkeleton(uri, retriever);
      },
      nb::arg("uri"),
      nb::arg("retriever") = nullptr);
  sm.def(
      "readSkeleton",
      [](const std::string& uri,
         const common::ResourceRetrieverPtr& retriever) {
        return SkelParser::readSkeleton(common::Uri(uri), retriever);
      },
      nb::arg("uri"),
      nb::arg("retriever") = nullptr);
}

} // namespace dart::python_nb
