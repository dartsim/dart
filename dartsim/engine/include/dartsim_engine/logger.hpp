/*
 * Copyright (c) 2011, The DART development contributors
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

#pragma once

#include <string>
#include <vector>

#include <cstddef>

namespace dartsim {

/// Severity of a log entry.
enum class LogLevel
{
  Info,
  Warning,
  Error
};

/// One logged message.
struct LogEntry
{
  LogLevel level = LogLevel::Info;
  std::string message;
};

/// Engine-side message log shown by the Console panel.
///
/// Keeps a bounded ring of recent entries so the headless engine owns the log
/// (rather than the GUI), matching the design doc's Logger/EventBus split.
class Logger
{
public:
  explicit Logger(std::size_t capacity = 1000) : m_capacity(capacity) {}

  void log(LogLevel level, std::string message)
  {
    m_entries.push_back(LogEntry{level, std::move(message)});
    if (m_entries.size() > m_capacity) {
      m_entries.erase(m_entries.begin());
    }
  }

  void info(std::string message)
  {
    log(LogLevel::Info, std::move(message));
  }
  void warning(std::string message)
  {
    log(LogLevel::Warning, std::move(message));
  }
  void error(std::string message)
  {
    log(LogLevel::Error, std::move(message));
  }

  [[nodiscard]] const std::vector<LogEntry>& entries() const
  {
    return m_entries;
  }
  [[nodiscard]] std::size_t size() const
  {
    return m_entries.size();
  }
  void clear()
  {
    m_entries.clear();
  }

private:
  std::vector<LogEntry> m_entries;
  std::size_t m_capacity;
};

} // namespace dartsim
