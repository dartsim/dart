/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#if DART_HAVE_spdlog
  #include <spdlog/spdlog.h>
#endif

namespace py = pybind11;

namespace dart::python {

void Logging(py::module& m)
{
  // Logging functions
  m.def("trace", [](const std::string& log) { common::trace(log); });
  m.def("debug", [](const std::string& log) { common::debug(log); });
  m.def("info", [](const std::string& log) { common::info(log); });
  m.def("warn", [](const std::string& log) { common::warn(log); });
  m.def("error", [](const std::string& log) { common::error(log); });
  m.def("fatal", [](const std::string& log) { common::fatal(log); });

  // Runtime log level control (only available with spdlog)
#if DART_HAVE_spdlog
  m.def(
      "set_log_level",
      [](const std::string& level) {
        if (level == "trace") {
          spdlog::set_level(spdlog::level::trace);
        } else if (level == "debug") {
          spdlog::set_level(spdlog::level::debug);
        } else if (level == "info") {
          spdlog::set_level(spdlog::level::info);
        } else if (level == "warn" || level == "warning") {
          spdlog::set_level(spdlog::level::warn);
        } else if (level == "error" || level == "err") {
          spdlog::set_level(spdlog::level::err);
        } else if (level == "critical" || level == "fatal") {
          spdlog::set_level(spdlog::level::critical);
        } else if (level == "off") {
          spdlog::set_level(spdlog::level::off);
        } else {
          throw std::invalid_argument(
              "Invalid log level: " + level
              + ". Valid levels are: trace, debug, info, warn, error, critical, off");
        }
      },
      py::arg("level"),
      "Set the runtime log level. Valid levels: trace, debug, info, warn, "
      "error, critical, off");

  m.def(
      "get_log_level",
      []() {
        switch (spdlog::level::level_enum(spdlog::get_level())) {
          case spdlog::level::trace:
            return "trace";
          case spdlog::level::debug:
            return "debug";
          case spdlog::level::info:
            return "info";
          case spdlog::level::warn:
            return "warn";
          case spdlog::level::err:
            return "error";
          case spdlog::level::critical:
            return "critical";
          case spdlog::level::off:
            return "off";
          default:
            return "unknown";
        }
      },
      "Get the current runtime log level");

  // Expose log level constants
  m.attr("LOG_LEVEL_TRACE") = "trace";
  m.attr("LOG_LEVEL_DEBUG") = "debug";
  m.attr("LOG_LEVEL_INFO") = "info";
  m.attr("LOG_LEVEL_WARN") = "warn";
  m.attr("LOG_LEVEL_ERROR") = "error";
  m.attr("LOG_LEVEL_CRITICAL") = "critical";
  m.attr("LOG_LEVEL_OFF") = "off";
#else
  m.def(
      "set_log_level",
      [](const std::string& level) {
        throw std::runtime_error(
            "Runtime log level control is only available when DART is built "
            "with spdlog support");
      },
      py::arg("level"),
      "Set the runtime log level (not available without spdlog)");

  m.def(
      "get_log_level",
      []() {
        throw std::runtime_error(
            "Runtime log level control is only available when DART is built "
            "with spdlog support");
        return "";
      },
      "Get the current runtime log level (not available without spdlog)");
#endif
}

} // namespace dart::python
