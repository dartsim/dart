/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/common/detail/SharedLibraryManager.hpp"

#include "dart/common/SharedLibrary.hpp"

namespace dart {
namespace common {
namespace detail {

//==============================================================================
std::shared_ptr<SharedLibrary> SharedLibraryManager::load(
    const std::string& path)
{
  const auto iter = mLibraries.find(path);

  const auto found = iter != mLibraries.end();
  if (found)
  {
    auto lib = iter->second.lock();

    // This check could fail if all instances to the library go out of scope,
    // since iter->second is a std::weak_ptr. In that case, we remove the
    // destructed library from the list and create new one.
    if (lib)
      return lib;
    else
      mLibraries.erase(iter);
  }

  const auto newLib = std::make_shared<SharedLibrary>(
      SharedLibrary::ProtectedConstruction, path);

  if (!newLib->isValid())
    return nullptr;

  mLibraries[path] = newLib;

  return newLib;
}

} // namespace detail
} // namespace common
} // namespace dart
