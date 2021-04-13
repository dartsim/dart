/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/dart.hpp>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void Uri(py::module& m)
{
  ::py::class_<dart::common::Uri>(m, "Uri")
      .def(::py::init<>())
      .def(::py::init<const std::string&>(), ::py::arg("input"))
      .def(::py::init<const char*>(), ::py::arg("input"))
      .def(
          "clear",
          +[](dart::common::Uri* self) -> void { return self->clear(); })
      .def(
          "fromString",
          +[](dart::common::Uri* self, const std::string& _input) -> bool {
            return self->fromString(_input);
          },
          ::py::arg("input"))
      .def(
          "fromPath",
          +[](dart::common::Uri* self, const std::string& _path) -> bool {
            return self->fromPath(_path);
          },
          ::py::arg("path"))
      .def(
          "fromStringOrPath",
          +[](dart::common::Uri* self, const std::string& _input) -> bool {
            return self->fromStringOrPath(_input);
          },
          ::py::arg("input"))
      .def(
          "fromRelativeUri",
          +[](dart::common::Uri* self,
              const std::string& _base,
              const std::string& _relative) -> bool {
            return self->fromRelativeUri(_base, _relative);
          },
          ::py::arg("base"),
          ::py::arg("relative"))
      .def(
          "fromRelativeUri",
          +[](dart::common::Uri* self,
              const std::string& _base,
              const std::string& _relative,
              bool _strict) -> bool {
            return self->fromRelativeUri(_base, _relative, _strict);
          },
          ::py::arg("base"),
          ::py::arg("relative"),
          ::py::arg("strict"))
      .def(
          "fromRelativeUri",
          +[](dart::common::Uri* self, const char* _base, const char* _relative)
              -> bool { return self->fromRelativeUri(_base, _relative); },
          ::py::arg("base"),
          ::py::arg("relative"))
      .def(
          "fromRelativeUri",
          +[](dart::common::Uri* self,
              const char* _base,
              const char* _relative,
              bool _strict) -> bool {
            return self->fromRelativeUri(_base, _relative, _strict);
          },
          ::py::arg("base"),
          ::py::arg("relative"),
          ::py::arg("strict"))
      .def(
          "fromRelativeUri",
          +[](dart::common::Uri* self,
              const dart::common::Uri& _base,
              const std::string& _relative) -> bool {
            return self->fromRelativeUri(_base, _relative);
          },
          ::py::arg("base"),
          ::py::arg("relative"))
      .def(
          "fromRelativeUri",
          +[](dart::common::Uri* self,
              const dart::common::Uri& _base,
              const std::string& _relative,
              bool _strict) -> bool {
            return self->fromRelativeUri(_base, _relative, _strict);
          },
          ::py::arg("base"),
          ::py::arg("relative"),
          ::py::arg("strict"))
      .def(
          "fromRelativeUri",
          +[](dart::common::Uri* self,
              const dart::common::Uri& _base,
              const char* _relative) -> bool {
            return self->fromRelativeUri(_base, _relative);
          },
          ::py::arg("base"),
          ::py::arg("relative"))
      .def(
          "fromRelativeUri",
          +[](dart::common::Uri* self,
              const dart::common::Uri& _base,
              const char* _relative,
              bool _strict) -> bool {
            return self->fromRelativeUri(_base, _relative, _strict);
          },
          ::py::arg("base"),
          ::py::arg("relative"),
          ::py::arg("strict"))
      .def(
          "fromRelativeUri",
          +[](dart::common::Uri* self,
              const dart::common::Uri& _base,
              const dart::common::Uri& _relative) -> bool {
            return self->fromRelativeUri(_base, _relative);
          },
          ::py::arg("base"),
          ::py::arg("relative"))
      .def(
          "fromRelativeUri",
          +[](dart::common::Uri* self,
              const dart::common::Uri& _base,
              const dart::common::Uri& _relative,
              bool _strict) -> bool {
            return self->fromRelativeUri(_base, _relative, _strict);
          },
          ::py::arg("base"),
          ::py::arg("relative"),
          ::py::arg("strict"))
      .def(
          "toString",
          +[](const dart::common::Uri* self) -> std::string {
            return self->toString();
          })
      .def(
          "getPath",
          +[](const dart::common::Uri* self) -> std::string {
            return self->getPath();
          })
      .def(
          "getFilesystemPath",
          +[](const dart::common::Uri* self) -> std::string {
            return self->getFilesystemPath();
          })
      .def_static(
          "createFromString",
          +[](const std::string& _input) -> dart::common::Uri {
            return dart::common::Uri::createFromString(_input);
          },
          ::py::arg("input"))
      .def_static(
          "createFromPath",
          +[](const std::string& _path) -> dart::common::Uri {
            return dart::common::Uri::createFromPath(_path);
          },
          ::py::arg("path"))
      .def_static(
          "createFromStringOrPath",
          +[](const std::string& _input) -> dart::common::Uri {
            return dart::common::Uri::createFromStringOrPath(_input);
          },
          ::py::arg("input"))
      .def_static(
          "createFromRelativeUri",
          +[](const std::string& _base,
              const std::string& _relative) -> dart::common::Uri {
            return dart::common::Uri::createFromRelativeUri(_base, _relative);
          },
          ::py::arg("base"),
          ::py::arg("relative"))
      .def_static(
          "createFromRelativeUri",
          +[](const std::string& _base,
              const std::string& _relative,
              bool _strict) -> dart::common::Uri {
            return dart::common::Uri::createFromRelativeUri(
                _base, _relative, _strict);
          },
          ::py::arg("base"),
          ::py::arg("relative"),
          ::py::arg("strict"))
      .def_static(
          "createFromRelativeUri",
          +[](const dart::common::Uri& _base,
              const std::string& _relative) -> dart::common::Uri {
            return dart::common::Uri::createFromRelativeUri(_base, _relative);
          },
          ::py::arg("base"),
          ::py::arg("relative"))
      .def_static(
          "createFromRelativeUri",
          +[](const dart::common::Uri& _base,
              const std::string& _relative,
              bool _strict) -> dart::common::Uri {
            return dart::common::Uri::createFromRelativeUri(
                _base, _relative, _strict);
          },
          ::py::arg("base"),
          ::py::arg("relative"),
          ::py::arg("strict"))
      .def_static(
          "createFromRelativeUri",
          +[](const dart::common::Uri& _base,
              const dart::common::Uri& _relative) -> dart::common::Uri {
            return dart::common::Uri::createFromRelativeUri(_base, _relative);
          },
          ::py::arg("base"),
          ::py::arg("relative"))
      .def_static(
          "createFromRelativeUri",
          +[](const dart::common::Uri& _base,
              const dart::common::Uri& _relative,
              bool _strict) -> dart::common::Uri {
            return dart::common::Uri::createFromRelativeUri(
                _base, _relative, _strict);
          },
          ::py::arg("base"),
          ::py::arg("relative"),
          ::py::arg("strict"))
      .def_static(
          "getUri",
          +[](const std::string& _input) -> std::string {
            return dart::common::Uri::getUri(_input);
          },
          ::py::arg("input"))
      .def_static(
          "getRelativeUri",
          +[](const std::string& _base,
              const std::string& _relative) -> std::string {
            return dart::common::Uri::getRelativeUri(_base, _relative);
          },
          ::py::arg("base"),
          ::py::arg("relative"))
      .def_static(
          "getRelativeUri",
          +[](const std::string& _base,
              const std::string& _relative,
              bool _strict) -> std::string {
            return dart::common::Uri::getRelativeUri(_base, _relative, _strict);
          },
          ::py::arg("base"),
          ::py::arg("relative"),
          ::py::arg("strict"))
      .def_static(
          "getRelativeUri",
          +[](const dart::common::Uri& _base,
              const std::string& _relative) -> std::string {
            return dart::common::Uri::getRelativeUri(_base, _relative);
          },
          ::py::arg("base"),
          ::py::arg("relative"))
      .def_static(
          "getRelativeUri",
          +[](const dart::common::Uri& _base,
              const std::string& _relative,
              bool _strict) -> std::string {
            return dart::common::Uri::getRelativeUri(_base, _relative, _strict);
          },
          ::py::arg("base"),
          ::py::arg("relative"),
          ::py::arg("strict"))
      .def_static(
          "getRelativeUri",
          +[](const dart::common::Uri& _base,
              const dart::common::Uri& _relative) -> std::string {
            return dart::common::Uri::getRelativeUri(_base, _relative);
          },
          ::py::arg("base"),
          ::py::arg("relative"))
      .def_static(
          "getRelativeUri",
          +[](const dart::common::Uri& _base,
              const dart::common::Uri& _relative,
              bool _strict) -> std::string {
            return dart::common::Uri::getRelativeUri(_base, _relative, _strict);
          },
          ::py::arg("base"),
          ::py::arg("relative"),
          ::py::arg("strict"))
      .def_readwrite("mScheme", &dart::common::Uri::mScheme)
      .def_readwrite("mAuthority", &dart::common::Uri::mAuthority)
      .def_readwrite("mPath", &dart::common::Uri::mPath)
      .def_readwrite("mQuery", &dart::common::Uri::mQuery)
      .def_readwrite("mFragment", &dart::common::Uri::mFragment);

  ::py::implicitly_convertible<std::string, dart::common::Uri>();
}

} // namespace python
} // namespace dart
