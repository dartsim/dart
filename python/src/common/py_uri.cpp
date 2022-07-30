/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/common/resource/uri.hpp"

#include "include_pybind11.h"

namespace py = pybind11;

namespace dart::python {

void py_uri(py::module& m)
{
  py::class_<common::Uri>(m, "Uri")
      .def(py::init<>())
      .def(py::init<const std::string&>(), py::arg("input"))
      .def(py::init<const char*>(), py::arg("input"))
      .def("clear", &common::Uri::clear)
      .def("from_string", &common::Uri::from_string, py::arg("input"))
      .def("from_path", &common::Uri::from_path, py::arg("path"))
      .def(
          "from_string_or_path",
          &common::Uri::from_string_or_path,
          py::arg("input"))
      .def(
          "from_relative_uri",
          py::overload_cast<const std::string&, const std::string&, bool>(
              &common::Uri::from_relative_uri),
          py::arg("base"),
          py::arg("relative"),
          py::arg("strict") = false)
      .def(
          "from_relative_uri",
          py::overload_cast<const char*, const char*, bool>(
              &common::Uri::from_relative_uri),
          py::arg("base"),
          py::arg("relative"),
          py::arg("strict") = false)
      .def(
          "from_relative_uri",
          py::overload_cast<const common::Uri&, const std::string&, bool>(
              &common::Uri::from_relative_uri),
          py::arg("base"),
          py::arg("relative"),
          py::arg("strict") = false)
      .def(
          "from_relative_uri",
          py::overload_cast<const common::Uri&, const char*, bool>(
              &common::Uri::from_relative_uri),
          py::arg("base"),
          py::arg("relative"),
          py::arg("strict") = false)
      .def(
          "from_relative_uri",
          py::overload_cast<const common::Uri&, const common::Uri&, bool>(
              &common::Uri::from_relative_uri),
          py::arg("base"),
          py::arg("relative"),
          py::arg("strict") = false)
      .def("to_string", &common::Uri::to_string)
      .def("get_path", &common::Uri::get_path)
      .def("get_filesystem_path", &common::Uri::get_filesystem_path)
      .def_static(
          "CreateFromString", &common::Uri::CreateFromString, py::arg("input"))
      .def_static(
          "CreateFromPath", &common::Uri::CreateFromPath, py::arg("path"))
      .def_static(
          "CreateFromString_or_path",
          &common::Uri::CreateFromString_or_path,
          py::arg("input"))
      .def_static(
          "CreateFromRelativeUri",
          py::overload_cast<const std::string&, const std::string&, bool>(
              &common::Uri::CreateFromRelativeUri),
          py::arg("base"),
          py::arg("relative"),
          py::arg("strict") = false)
      .def_static(
          "CreateFromRelativeUri",
          py::overload_cast<const common::Uri&, const std::string&, bool>(
              &common::Uri::CreateFromRelativeUri),
          py::arg("base"),
          py::arg("relative"),
          py::arg("strict") = false)
      .def_static(
          "CreateFromRelativeUri",
          py::overload_cast<const common::Uri&, const common::Uri&, bool>(
              &common::Uri::CreateFromRelativeUri),
          py::arg("base"),
          py::arg("relative"),
          py::arg("strict") = false)
      .def_static("GetUri", &common::Uri::GetUri, py::arg("input"))
      .def_static(
          "GetRelativeUri",
          py::overload_cast<const std::string&, const std::string&, bool>(
              &common::Uri::GetRelativeUri),
          py::arg("base"),
          py::arg("relative"),
          py::arg("strict") = false)
      .def_static(
          "GetRelativeUri",
          py::overload_cast<const common::Uri&, const std::string&, bool>(
              &common::Uri::GetRelativeUri),
          py::arg("base"),
          py::arg("relative"),
          py::arg("strict") = false)
      .def_static(
          "GetRelativeUri",
          py::overload_cast<const common::Uri&, const common::Uri&, bool>(
              &common::Uri::GetRelativeUri),
          py::arg("base"),
          py::arg("relative"),
          py::arg("strict") = false)
      .def_readwrite("scheme", &common::Uri::scheme)
      .def_readwrite("authority", &common::Uri::authority)
      .def_readwrite("path", &common::Uri::path)
      .def_readwrite("query", &common::Uri::query)
      .def_readwrite("fragment", &common::Uri::fragment);

  py::implicitly_convertible<std::string, common::Uri>();
}

} // namespace dart::python
