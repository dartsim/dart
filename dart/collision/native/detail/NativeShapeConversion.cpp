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

#include "dart/collision/native/detail/NativeShapeConversion.hpp"

#include "dart/collision/native/shapes/shape.hpp"
#include "dart/common/Console.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/SphereShape.hpp"

#include <set>
#include <string>

namespace dart {
namespace collision {
namespace detail {

//==============================================================================
std::unique_ptr<native::Shape> NativeShapeConversion::create(
    const dynamics::Shape& shape)
{
  const auto& shapeType = shape.getType();

  if (shapeType == dynamics::SphereShape::getStaticType()) {
    const auto& sphere = static_cast<const dynamics::SphereShape&>(shape);
    return std::make_unique<native::SphereShape>(sphere.getRadius());
  }

  if (shapeType == dynamics::BoxShape::getStaticType()) {
    const auto& box = static_cast<const dynamics::BoxShape&>(shape);
    return std::make_unique<native::BoxShape>(0.5 * box.getSize());
  }

  static std::set<std::string> warnedShapeTypes;
  if (warnedShapeTypes.insert(shapeType).second) {
    dtwarn << "[NativeShapeConversion] Shape type [" << shapeType
           << "] is not supported by NativeCollisionDetector yet. This "
           << "shape will be skipped by the native adapter.\n";
  }

  return nullptr;
}

} // namespace detail
} // namespace collision
} // namespace dart
