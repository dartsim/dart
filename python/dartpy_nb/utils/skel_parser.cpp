#include "utils/skel_parser.hpp"

#include "dart/utils/skel/SkelParser.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defSkelParser(nb::module_& m)
{
  auto sm = m.def_submodule("SkelParser");

  sm.def(
      "readWorld",
      [](const common::Uri& uri,
         const common::ResourceRetrieverPtr& retriever) {
        return utils::SkelParser::readWorld(uri, retriever);
      },
      nb::arg("uri"),
      nb::arg("retriever") = nullptr);
  sm.def(
      "readWorld",
      [](const std::string& uri,
         const common::ResourceRetrieverPtr& retriever) {
        return utils::SkelParser::readWorld(common::Uri(uri), retriever);
      },
      nb::arg("uri"),
      nb::arg("retriever") = nullptr);
  sm.def(
      "readWorldXML",
      [](const std::string& xml,
         const std::string& base,
         const common::ResourceRetrieverPtr& retriever) {
        return utils::SkelParser::readWorldXML(xml, base, retriever);
      },
      nb::arg("xmlString"),
      nb::arg("baseUri") = "",
      nb::arg("retriever") = nullptr);
  sm.def(
      "readSkeleton",
      [](const common::Uri& uri,
         const common::ResourceRetrieverPtr& retriever) {
        return utils::SkelParser::readSkeleton(uri, retriever);
      },
      nb::arg("uri"),
      nb::arg("retriever") = nullptr);
  sm.def(
      "readSkeleton",
      [](const std::string& uri,
         const common::ResourceRetrieverPtr& retriever) {
        return utils::SkelParser::readSkeleton(common::Uri(uri), retriever);
      },
      nb::arg("uri"),
      nb::arg("retriever") = nullptr);
}

} // namespace dart::python_nb
