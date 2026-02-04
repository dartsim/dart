#include "common/uri.hpp"

#include "dart/common/uri.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <string_view>

namespace nb = nanobind;

namespace dart::python_nb {

void defUri(nb::module_& m)
{
  using Uri = dart::common::Uri;

  auto cls = nb::class_<Uri>(m, "Uri");
  cls.def(nb::init<>())
      .def(nb::init<const std::string&>(), nb::arg("input"))
      .def(nb::init<const char*>(), nb::arg("input"))
      .def("clear", &Uri::clear)
      .def(
          "fromString",
          [](Uri& self, const std::string& input) {
            return self.fromString(input);
          },
          nb::arg("input"))
      .def(
          "fromPath",
          [](Uri& self, const std::string& path) {
            return self.fromPath(path);
          },
          nb::arg("path"))
      .def(
          "fromStringOrPath",
          [](Uri& self, const std::string& input) {
            return self.fromStringOrPath(input);
          },
          nb::arg("input"))
      .def(
          "fromRelativeUri",
          [](Uri& self,
             const std::string& base,
             const std::string& relative,
             bool strict) {
            return self.fromRelativeUri(
                std::string_view{base}, std::string_view{relative}, strict);
          },
          nb::arg("base"),
          nb::arg("relative"),
          nb::arg("strict") = false)
      .def(
          "fromRelativeUri",
          [](Uri& self, const char* base, const char* relative, bool strict) {
            return self.fromRelativeUri(base, relative, strict);
          },
          nb::arg("base"),
          nb::arg("relative"),
          nb::arg("strict") = false)
      .def(
          "fromRelativeUri",
          [](Uri& self,
             const Uri& base,
             const std::string& relative,
             bool strict) {
            return self.fromRelativeUri(
                base, std::string_view{relative}, strict);
          },
          nb::arg("base"),
          nb::arg("relative"),
          nb::arg("strict") = false)
      .def(
          "fromRelativeUri",
          [](Uri& self, const Uri& base, const char* relative, bool strict) {
            return self.fromRelativeUri(base, relative, strict);
          },
          nb::arg("base"),
          nb::arg("relative"),
          nb::arg("strict") = false)
      .def(
          "fromRelativeUri",
          [](Uri& self, const Uri& base, const Uri& relative, bool strict) {
            return self.fromRelativeUri(base, relative, strict);
          },
          nb::arg("base"),
          nb::arg("relative"),
          nb::arg("strict") = false)
      .def("toString", [](const Uri& self) { return self.toString(); })
      .def("getPath", [](const Uri& self) { return self.getPath(); })
      .def(
          "getFilesystemPath",
          [](const Uri& self) { return self.getFilesystemPath(); })
      .def_static(
          "createFromString",
          [](const std::string& input) { return Uri::createFromString(input); },
          nb::arg("input"))
      .def_static(
          "createFromPath",
          [](const std::string& path) { return Uri::createFromPath(path); },
          nb::arg("path"))
      .def_static(
          "createFromStringOrPath",
          [](const std::string& input) {
            return Uri::createFromStringOrPath(input);
          },
          nb::arg("input"))
      .def_static(
          "createFromRelativeUri",
          [](const std::string& base,
             const std::string& relative,
             bool strict) {
            return Uri::createFromRelativeUri(
                std::string_view{base}, std::string_view{relative}, strict);
          },
          nb::arg("base"),
          nb::arg("relative"),
          nb::arg("strict") = false)
      .def_static(
          "createFromRelativeUri",
          [](const Uri& base, const std::string& relative, bool strict) {
            return Uri::createFromRelativeUri(
                base, std::string_view{relative}, strict);
          },
          nb::arg("base"),
          nb::arg("relative"),
          nb::arg("strict") = false)
      .def_static(
          "createFromRelativeUri",
          [](const Uri& base, const Uri& relative, bool strict) {
            return Uri::createFromRelativeUri(base, relative, strict);
          },
          nb::arg("base"),
          nb::arg("relative"),
          nb::arg("strict") = false)
      .def_static(
          "getUri",
          [](const std::string& input) { return Uri::getUri(input); },
          nb::arg("input"))
      .def_static(
          "getRelativeUri",
          [](const std::string& base,
             const std::string& relative,
             bool strict) {
            return Uri::getRelativeUri(
                std::string_view{base}, std::string_view{relative}, strict);
          },
          nb::arg("base"),
          nb::arg("relative"),
          nb::arg("strict") = false)
      .def_static(
          "getRelativeUri",
          [](const Uri& base, const std::string& relative, bool strict) {
            return Uri::getRelativeUri(
                base, std::string_view{relative}, strict);
          },
          nb::arg("base"),
          nb::arg("relative"),
          nb::arg("strict") = false)
      .def_static(
          "getRelativeUri",
          [](const Uri& base, const Uri& relative, bool strict) {
            return Uri::getRelativeUri(base, relative, strict);
          },
          nb::arg("base"),
          nb::arg("relative"),
          nb::arg("strict") = false)
      .def_rw("mScheme", &Uri::mScheme)
      .def_rw("mAuthority", &Uri::mAuthority)
      .def_rw("mPath", &Uri::mPath)
      .def_rw("mQuery", &Uri::mQuery)
      .def_rw("mFragment", &Uri::mFragment);

  nb::implicitly_convertible<std::string, Uri>();
}

} // namespace dart::python_nb
