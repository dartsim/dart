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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR
 *   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 *   OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 *   BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *   USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 *   DAMAGE.
 */

#ifndef DART_GUI_DETAIL_RENDERABLE_FACTORY_HPP_
#define DART_GUI_DETAIL_RENDERABLE_FACTORY_HPP_

#include <dart/gui/debug.hpp>
#include <dart/gui/detail/renderable_resources.hpp>
#include <dart/gui/renderable.hpp>

#include <optional>
#include <vector>

namespace filament {
class Engine;
class Material;
} // namespace filament

namespace dart::gui::detail {

std::optional<Renderable> createDebugLineRenderable(
    ::filament::Engine& engine,
    ::filament::Material& material,
    const std::vector<dart::gui::DebugLineDescriptor>& lines);

std::optional<Renderable> createDebugLineRenderable(
    ::filament::Engine& engine,
    ::filament::Material& material,
    const std::vector<dart::gui::DebugLineDescriptor>& lines,
    const std::vector<dart::gui::DebugTriangleDescriptor>& triangles);

/// True for shapes rendered with a lit material that exposes
/// metallic/roughness/reflectance parameters and does not supply its own
/// per-part asset materials (asset meshes and unlit line/point/voxel
/// renderables are excluded).
DART_GUI_API bool shapeUsesLitMaterialOverride(dart::gui::ShapeKind kind);

std::optional<Renderable> createRenderableFromDescriptor(
    ::filament::Engine& engine,
    const MaterialSet& materials,
    TextureCache& textureCache,
    const dart::gui::RenderableDescriptor& descriptor);

} // namespace dart::gui::detail

#endif // DART_GUI_DETAIL_RENDERABLE_FACTORY_HPP_
