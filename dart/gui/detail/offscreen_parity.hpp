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

#ifndef DART_GUI_DETAIL_OFFSCREEN_PARITY_HPP_
#define DART_GUI_DETAIL_OFFSCREEN_PARITY_HPP_

#include <iosfwd>

#include <cstdint>

namespace dart::gui::detail {

struct FilamentRenderContext;

/// Diagnostic self-check for the Filament offscreen render-to-texture path.
///
/// This is the Phase-1 parity gate of the offscreen-viewport spike (Decision 3
/// of docs/design/dartsim_gui_toolkit_decisions.md): it renders the primary
/// view both to the swapchain (the on-screen path) and to an offscreen
/// `RenderTarget` backed by a color `Texture`, reads both back, and reports
/// whether the offscreen render matches the swapchain render within a small
/// tolerance. It also requires the content to be non-trivial (the image must
/// vary) so a uniform clear cannot trivially "pass".
///
/// Returns true when the offscreen and swapchain renders agree and the content
/// is non-trivial. Requires a live graphics context (headless OpenGL is fine)
/// and the NOOP backend is skipped (it produces no pixels). The check is
/// diagnostic only and is gated by the `DART_GUI_OFFSCREEN_PARITY` environment
/// variable at the call site.
bool runOffscreenParitySelfCheck(
    FilamentRenderContext& context,
    std::uint32_t width,
    std::uint32_t height,
    std::ostream& log);

} // namespace dart::gui::detail

#endif // DART_GUI_DETAIL_OFFSCREEN_PARITY_HPP_
