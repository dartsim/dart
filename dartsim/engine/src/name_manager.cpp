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

#include <dartsim_engine/name_manager.hpp>
#include <dartsim_engine/scene_model.hpp>

#include <string>

namespace dartsim {

std::string NameManager::defaultBaseName(ObjectType type)
{
  switch (type) {
    case ObjectType::RigidBody:
      return "Body";
    case ObjectType::MultiBody:
      return "MultiBody";
    case ObjectType::Link:
      return "Link";
    case ObjectType::Joint:
      return "Joint";
    case ObjectType::FreeFrame:
      return "FreeFrame";
    case ObjectType::FixedFrame:
      return "FixedFrame";
  }
  return "Object";
}

std::string NameManager::makeUnique(
    const SceneModel& model,
    ObjectId parent,
    std::string_view base,
    ObjectId except)
{
  const std::string baseName = base.empty() ? "Object" : std::string(base);
  std::string candidate = baseName;
  if (model.isNameAvailable(parent, candidate, except)) {
    return candidate;
  }
  for (int suffix = 1;; ++suffix) {
    std::string next = baseName + " " + std::to_string(suffix);
    if (model.isNameAvailable(parent, next, except)) {
      return next;
    }
  }
}

} // namespace dartsim
