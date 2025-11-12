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

#include "logging_bind.hpp"

#include "dart8/common/logging.hpp"

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace dartpy8 {

void defLogging(nb::module_& m)
{
  DART8_DEBUG("Registering logging API");

  // Logging severity levels enum
  nb::enum_<dart8::common::LogLevel>(m, "LogLevel", "Logging severity levels")
      .value("Trace", dart8::common::LogLevel::Trace)
      .value("Debug", dart8::common::LogLevel::Debug)
      .value("Info", dart8::common::LogLevel::Info)
      .value("Warn", dart8::common::LogLevel::Warn)
      .value("Error", dart8::common::LogLevel::Error)
      .value("Critical", dart8::common::LogLevel::Critical)
      .value("Off", dart8::common::LogLevel::Off);

  // Logging control functions
  m.def(
      "set_log_level",
      &dart8::common::setLogLevel,
      nb::arg("level"),
      "Set the global logging level");

  m.def(
      "get_log_level",
      &dart8::common::getLogLevel,
      "Get the current logging level");

  m.def(
      "initialize_logging",
      &dart8::common::initializeLogging,
      "Initialize DART8 logging system");
}

} // namespace dartpy8
