/*
 * Copyright (c) 2011-2022, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/common/type_name.hpp"

#include "dart/common/compiler.hpp"

#if DART_COMPILER_GNU || DART_COMPILER_CLANG
  #include <cxxabi.h>
#endif

#include <iostream>
#include <sstream>

namespace dart::common {

//========================================================================================
std::string error_message(int status)
{
  std::stringstream ss;

  switch (status) {
    case -1:
      ss << "A memory allocation failure occurred.";
      break;
    case -2:
      ss << "The mangled name is not a valid name under the C++ ABI mangling "
            "rules.";
      break;
    case -3:
      ss << "One of the arguments is invalid";
      break;
    default:
      ss << "Unknow error";
  }

  return ss.str();
}

//========================================================================================
std::string demangle_symbol(const std::string& mangled_name)
{
#if DART_COMPILER_GNU || DART_COMPILER_CLANG

  int status;
  char* demangled_str
      = abi::__cxa_demangle(mangled_name.c_str(), nullptr, nullptr, &status);

  const auto demangled_name = std::string(demangled_str);

  // The caller is resoponsible for deallocating demangled_str.
  if (demangled_str) {
    std::free(demangled_str);
  }

  // Return the original name if failed to demangle it
  if (status != 0) {
    std::cerr << "Failsed to demangle name [" << mangled_name
              << "]: " << error_message(status) << std::endl;
    return mangled_name;
  }

  return demangled_name;
#else
  return mangled_name;
#endif
}

} // namespace dart::common
