/*
 * Copyright (c) 2011-2026, The DART development contributors
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

#include <dart/simulation/experimental/comps/component_category.hpp>

#include <entt/entt.hpp>

#include <vector>

namespace dart::simulation::experimental::comps {

/// Tag marking entity as a MultiBody
///
/// Automatically serialized via DART_EXPERIMENTAL_TAG_COMPONENT macro.
/// **Internal Implementation Detail** - Not exposed in public API
struct MultiBodyTag
{
  DART_EXPERIMENTAL_TAG_COMPONENT(MultiBodyTag);
};

/// Component storing MultiBody structure (links, joints)
///
/// Automatically serialized with entity remapping via
/// DART_EXPERIMENTAL_STATE_COMPONENT macro.
/// **Internal Implementation Detail** - Not exposed in public API
struct MultiBodyStructure
{
  DART_EXPERIMENTAL_STATE_COMPONENT(MultiBodyStructure);

  std::vector<entt::entity> links;
  std::vector<entt::entity> joints;

  /// Declare which fields need entity remapping during serialization
  static constexpr auto entityFields()
  {
    return std::tuple{&MultiBodyStructure::links, &MultiBodyStructure::joints};
  }
};

} // namespace dart::simulation::experimental::comps
