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

#ifndef DART_GUI_EXPERIMENTAL_RENDERABLE_HPP_
#define DART_GUI_EXPERIMENTAL_RENDERABLE_HPP_

#include <dart/gui/renderable.hpp>

namespace dart::gui::experimental {

using ::dart::gui::ActiveRenderableState;
using ::dart::gui::describeShape;
using ::dart::gui::extractRenderables;
using ::dart::gui::GeometryDescriptor;
using ::dart::gui::makeRenderableId;
using ::dart::gui::MaterialDescriptor;
using ::dart::gui::MeshAlphaMode;
using ::dart::gui::MeshMaterialDescriptor;
using ::dart::gui::MeshPartDescriptor;
using ::dart::gui::planRenderableSetUpdate;
using ::dart::gui::RenderableDescriptor;
using ::dart::gui::RenderableId;
using ::dart::gui::RenderableSetUpdatePlan;
using ::dart::gui::ShapeKind;

} // namespace dart::gui::experimental

#endif // DART_GUI_EXPERIMENTAL_RENDERABLE_HPP_
