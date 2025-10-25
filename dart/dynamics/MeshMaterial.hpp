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

#ifndef DART_DYNAMICS_MESHMATERIAL_HPP_
#define DART_DYNAMICS_MESHMATERIAL_HPP_

#include <Eigen/Core>

#include <string>
#include <vector>

namespace dart {
namespace dynamics {

/// Simple material representation for mesh rendering
/// Stores material properties independently of Assimp types
struct MeshMaterial
{
  /// Material colors
  Eigen::Vector4f ambient{0.2f, 0.2f, 0.2f, 1.0f};
  Eigen::Vector4f diffuse{0.8f, 0.8f, 0.8f, 1.0f};
  Eigen::Vector4f specular{0.0f, 0.0f, 0.0f, 1.0f};
  Eigen::Vector4f emissive{0.0f, 0.0f, 0.0f, 1.0f};

  /// Shininess coefficient
  float shininess{0.0f};

  /// Texture image paths (absolute paths)
  /// Index 0: diffuse texture
  /// Index 1: specular texture
  /// Index 2: normal texture
  /// etc.
  std::vector<std::string> textureImagePaths;

  /// Default constructor
  MeshMaterial() = default;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_MESHMATERIAL_HPP_
