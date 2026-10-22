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

#include "dart/common/Console.hpp"

#include <atomic>
#include <iostream>

namespace dart {
namespace common {

namespace {
std::ostream& defaultOut()
{
  return std::cout;
}
std::ostream& defaultErr()
{
  return std::cerr;
}
std::atomic<ConsoleStream> sConsoleOut{&defaultOut};
std::atomic<ConsoleStream> sConsoleErr{&defaultErr};
} // namespace

void setConsoleStreams(ConsoleStream out, ConsoleStream err)
{
  sConsoleOut = out ? out : &defaultOut;
  sConsoleErr = err ? err : &defaultErr;
}

std::ostream& consoleOut()
{
  return sConsoleOut.load()();
}

std::ostream& consoleErr()
{
  return sConsoleErr.load()();
}

std::ostream& colorMsg(const std::string& _msg, int _color)
{
  std::ostream& out = consoleOut();
  out << "\033[1;" << _color << "m" << _msg << "\033[0m ";
  return out;
}

std::ostream& colorErr(
    const std::string& _msg,
    const std::string& _file,
    unsigned int _line,
    int _color)
{
  int index = _file.find_last_of("/") + 1;

  std::ostream& err = consoleErr();
  err << "\033[1;" << _color << "m" << _msg << " ["
      << _file.substr(index, _file.size() - index) << ":" << _line
      << "]\033[0m ";

  return err;
}

} // namespace common
} // namespace dart
