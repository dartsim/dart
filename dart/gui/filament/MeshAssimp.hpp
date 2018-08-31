/*
 * Copyright (c) 2011-2018, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

/*
 * Copyright (C) 2015 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DART_GUI_FILAMENT_MESHASSIMP_HPP_
#define DART_GUI_FILAMENT_MESHASSIMP_HPP_

#include <map>
#include <vector>

#include <math/mat4.h>
#include <math/quat.h>
#include <math/vec3.h>

#include <utils/EntityManager.h>
#include "Path.hpp"

#include <filamat/MaterialBuilder.h>
#include <filament/Color.h>
#include <filament/Box.h>

namespace filament {
    class Engine;
    class VertexBuffer;
    class IndexBuffer;
    class Material;
    class MaterialInstance;
    class Renderable;
}

namespace dart {
namespace gui {
namespace flmt {

class MeshAssimp {
public:
    using mat4f = ::math::mat4f;
    using half4 = ::math::half4;
    using short4 = ::math::short4;
    using half2 = ::math::half2;
    explicit MeshAssimp(filament::Engine& engine);
    ~MeshAssimp();

    void addFromFile(const utils::Path& path,
            std::map<std::string, filament::MaterialInstance*>& materials,
            bool overrideMaterial = false);

    const std::vector<::utils::Entity> getRenderables() const noexcept {
        return mRenderables;
    }

private:
    struct Part {
        size_t offset;
        size_t count;
        std::string material;
        filament::sRGBColor baseColor;
        float opacity;
        float metallic;
        float roughness;
        float reflectance;
    };

    struct Mesh {
        size_t offset;
        size_t count;
        std::vector<Part> parts;
        filament::Box aabb;
        mat4f transform;
    };

    bool setFromFile(const utils::Path& file,
            std::vector<uint32_t>& outIndices,
            std::vector<half4>&    outPositions,
            std::vector<short4>&   outTangents,
            std::vector<half2>&    outTexCoords,
            std::vector<Mesh>&     outMeshes,
            std::vector<int>&      outParents);

    filament::Engine& mEngine;
    filament::VertexBuffer* mVertexBuffer = nullptr;
    filament::IndexBuffer* mIndexBuffer = nullptr;

    filament::Material* mDefaultColorMaterial = nullptr;
    filament::Material* mDefaultTransparentColorMaterial = nullptr;

    std::vector<::utils::Entity> mRenderables;
};

} // namespace flmt
} // namespace gui
} // namespace dart

#endif // DART_GUI_FILAMENT_MESHASSIMP_HPP_
