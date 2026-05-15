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

#ifndef EXAMPLES_FILAMENT_GUI_TEXTURES_HPP_
#define EXAMPLES_FILAMENT_GUI_TEXTURES_HPP_

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include <cstdint>

namespace filament {
class Engine;
class MaterialInstance;
class Texture;
} // namespace filament

namespace dart::examples::filament_gui {

struct TextureBinding
{
  filament::Texture* texture = nullptr;
};

enum class TextureColorSpace
{
  Linear,
  Srgb,
};

struct PbrTextureBindings
{
  const TextureBinding* baseColor = nullptr;
  const TextureBinding* metallic = nullptr;
  const TextureBinding* roughness = nullptr;
  const TextureBinding* metallicRoughness = nullptr;
  const TextureBinding* normal = nullptr;
  const TextureBinding* occlusion = nullptr;
  const TextureBinding* emissive = nullptr;
};

struct TextureCache
{
  std::unordered_map<std::string, TextureBinding> bindings;
  std::vector<filament::Texture*> ownedTextures;
};

const TextureBinding* getOrLoadTextureBinding(
    filament::Engine& engine,
    TextureCache& cache,
    const std::string& source,
    TextureColorSpace colorSpace);

bool hasTextureBindings(const PbrTextureBindings& textures);

void setPbrTextureParameters(
    filament::MaterialInstance& material,
    const TextureBinding& fallback,
    const PbrTextureBindings& textures);

filament::Texture* createCheckerTexture(filament::Engine& engine);

filament::Texture* createSolidTexture(
    filament::Engine& engine,
    const std::array<std::uint8_t, 4>& color,
    TextureColorSpace colorSpace);

} // namespace dart::examples::filament_gui

#endif // EXAMPLES_FILAMENT_GUI_TEXTURES_HPP_
