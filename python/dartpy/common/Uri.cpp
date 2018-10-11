#include <dart/dart.hpp>
#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void Uri(pybind11::module& m)
{
  ::pybind11::class_<dart::common::Uri >(m, "Uri")
      .def(::pybind11::init<>())
      .def(::pybind11::init<const std::string &>(), ::pybind11::arg("_input"))
      .def(::pybind11::init<const char *>(), ::pybind11::arg("_input"))
      .def("clear", +[](dart::common::Uri *self) -> void { return self->clear(); })
      .def("fromString", +[](dart::common::Uri *self, const std::string & _input) -> bool { return self->fromString(_input); }, ::pybind11::arg("_input"))
      .def("fromPath", +[](dart::common::Uri *self, const std::string & _path) -> bool { return self->fromPath(_path); }, ::pybind11::arg("_path"))
      .def("fromStringOrPath", +[](dart::common::Uri *self, const std::string & _input) -> bool { return self->fromStringOrPath(_input); }, ::pybind11::arg("_input"))
      .def("fromRelativeUri", +[](dart::common::Uri *self, const std::string & _base, const std::string & _relative) -> bool { return self->fromRelativeUri(_base, _relative); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"))
      .def("fromRelativeUri", +[](dart::common::Uri *self, const std::string & _base, const std::string & _relative, bool _strict) -> bool { return self->fromRelativeUri(_base, _relative, _strict); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"), ::pybind11::arg("_strict"))
      .def("fromRelativeUri", +[](dart::common::Uri *self, const char * _base, const char * _relative) -> bool { return self->fromRelativeUri(_base, _relative); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"))
      .def("fromRelativeUri", +[](dart::common::Uri *self, const char * _base, const char * _relative, bool _strict) -> bool { return self->fromRelativeUri(_base, _relative, _strict); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"), ::pybind11::arg("_strict"))
      .def("fromRelativeUri", +[](dart::common::Uri *self, const dart::common::Uri & _base, const std::string & _relative) -> bool { return self->fromRelativeUri(_base, _relative); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"))
      .def("fromRelativeUri", +[](dart::common::Uri *self, const dart::common::Uri & _base, const std::string & _relative, bool _strict) -> bool { return self->fromRelativeUri(_base, _relative, _strict); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"), ::pybind11::arg("_strict"))
      .def("fromRelativeUri", +[](dart::common::Uri *self, const dart::common::Uri & _base, const char * _relative) -> bool { return self->fromRelativeUri(_base, _relative); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"))
      .def("fromRelativeUri", +[](dart::common::Uri *self, const dart::common::Uri & _base, const char * _relative, bool _strict) -> bool { return self->fromRelativeUri(_base, _relative, _strict); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"), ::pybind11::arg("_strict"))
      .def("fromRelativeUri", +[](dart::common::Uri *self, const dart::common::Uri & _base, const dart::common::Uri & _relative) -> bool { return self->fromRelativeUri(_base, _relative); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"))
      .def("fromRelativeUri", +[](dart::common::Uri *self, const dart::common::Uri & _base, const dart::common::Uri & _relative, bool _strict) -> bool { return self->fromRelativeUri(_base, _relative, _strict); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"), ::pybind11::arg("_strict"))
      .def("toString", +[](const dart::common::Uri *self) -> std::string { return self->toString(); })
      .def("getPath", +[](const dart::common::Uri *self) -> std::string { return self->getPath(); })
      .def("getFilesystemPath", +[](const dart::common::Uri *self) -> std::string { return self->getFilesystemPath(); })
      .def_static("createFromString", +[](const std::string & _input) -> dart::common::Uri { return dart::common::Uri::createFromString(_input); }, ::pybind11::arg("_input"))
      .def_static("createFromPath", +[](const std::string & _path) -> dart::common::Uri { return dart::common::Uri::createFromPath(_path); }, ::pybind11::arg("_path"))
      .def_static("createFromStringOrPath", +[](const std::string & _input) -> dart::common::Uri { return dart::common::Uri::createFromStringOrPath(_input); }, ::pybind11::arg("_input"))
      .def_static("createFromRelativeUri", +[](const std::string & _base, const std::string & _relative) -> dart::common::Uri { return dart::common::Uri::createFromRelativeUri(_base, _relative); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"))
      .def_static("createFromRelativeUri", +[](const std::string & _base, const std::string & _relative, bool _strict) -> dart::common::Uri { return dart::common::Uri::createFromRelativeUri(_base, _relative, _strict); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"), ::pybind11::arg("_strict"))
      .def_static("createFromRelativeUri", +[](const dart::common::Uri & _base, const std::string & _relative) -> dart::common::Uri { return dart::common::Uri::createFromRelativeUri(_base, _relative); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"))
      .def_static("createFromRelativeUri", +[](const dart::common::Uri & _base, const std::string & _relative, bool _strict) -> dart::common::Uri { return dart::common::Uri::createFromRelativeUri(_base, _relative, _strict); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"), ::pybind11::arg("_strict"))
      .def_static("createFromRelativeUri", +[](const dart::common::Uri & _base, const dart::common::Uri & _relative) -> dart::common::Uri { return dart::common::Uri::createFromRelativeUri(_base, _relative); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"))
      .def_static("createFromRelativeUri", +[](const dart::common::Uri & _base, const dart::common::Uri & _relative, bool _strict) -> dart::common::Uri { return dart::common::Uri::createFromRelativeUri(_base, _relative, _strict); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"), ::pybind11::arg("_strict"))
      .def_static("getUri", +[](const std::string & _input) -> std::string { return dart::common::Uri::getUri(_input); }, ::pybind11::arg("_input"))
      .def_static("getRelativeUri", +[](const std::string & _base, const std::string & _relative) -> std::string { return dart::common::Uri::getRelativeUri(_base, _relative); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"))
      .def_static("getRelativeUri", +[](const std::string & _base, const std::string & _relative, bool _strict) -> std::string { return dart::common::Uri::getRelativeUri(_base, _relative, _strict); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"), ::pybind11::arg("_strict"))
      .def_static("getRelativeUri", +[](const dart::common::Uri & _base, const std::string & _relative) -> std::string { return dart::common::Uri::getRelativeUri(_base, _relative); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"))
      .def_static("getRelativeUri", +[](const dart::common::Uri & _base, const std::string & _relative, bool _strict) -> std::string { return dart::common::Uri::getRelativeUri(_base, _relative, _strict); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"), ::pybind11::arg("_strict"))
      .def_static("getRelativeUri", +[](const dart::common::Uri & _base, const dart::common::Uri & _relative) -> std::string { return dart::common::Uri::getRelativeUri(_base, _relative); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"))
      .def_static("getRelativeUri", +[](const dart::common::Uri & _base, const dart::common::Uri & _relative, bool _strict) -> std::string { return dart::common::Uri::getRelativeUri(_base, _relative, _strict); }, ::pybind11::arg("_base"), ::pybind11::arg("_relative"), ::pybind11::arg("_strict"))
      .def_readwrite("mScheme", &dart::common::Uri::mScheme)
      .def_readwrite("mAuthority", &dart::common::Uri::mAuthority)
      .def_readwrite("mPath", &dart::common::Uri::mPath)
      .def_readwrite("mQuery", &dart::common::Uri::mQuery)
      .def_readwrite("mFragment", &dart::common::Uri::mFragment)
      ;

  ::pybind11::implicitly_convertible<std::string, dart::common::Uri>();
}

} // namespace python
} // namespace dart
