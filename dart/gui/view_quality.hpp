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

#ifndef DART_GUI_VIEW_QUALITY_HPP_
#define DART_GUI_VIEW_QUALITY_HPP_

#include <dart/gui/export.hpp>
#include <dart/gui/renderable.hpp>
#include <dart/gui/viewer.hpp>

#include <string>
#include <vector>

namespace dart::gui {

/// Machine-readable, geometry-first verdict for one camera against one scene.
///
/// Every field is a pure function of the descriptor set, the camera, and the
/// viewport (no GPU): `assessView` projects focus bounds, casts CPU picking
/// rays for occlusion, and compares screen boxes for ambiguity, so a headless
/// agent can reject an inadequate capture (cropped, too close/far, occluded,
/// or ambiguous) before any pixels are rendered.
struct ViewQualityReport
{
  /// Fraction of projected focus-bounds corners that land inside the viewport.
  double cornerCoverage = 0.0;
  /// Screen-area fraction of the focus subject's viewport-clipped bounding box.
  double subjectFraction = 0.0;
  /// Whether the focus centroid projects in front of and inside the viewport.
  bool centerVisible = false;
  /// Fraction of focus-sample rays blocked by another renderable.
  double occlusionFraction = 0.0;
  /// Worst pairwise screen-box IoU among depth-separated bodies.
  double ambiguityIoU = 0.0;
  /// Named issues (cropped/off-frame/too-far/too-close/occluded/ambiguous/
  /// no-bounded-focus); empty when the view is acceptable.
  std::vector<std::string> issues;
  /// Deterministic, monotone quality score in [0, 1].
  double score = 0.0;
};

/// Assesses one orbit-camera view of `descriptors` without rendering.
///
/// `focusIds` selects the subject descriptors by their `RenderableId`; an empty
/// list assesses all bounded descriptors. The geometry mirrors the reviewed
/// Python reference exactly: corner coverage over the focus bounds' corners,
/// subject screen fraction with viewport clipping, center visibility, occlusion
/// via `pickNearestRenderable` rays from the eye to the focus corners and
/// centroid (excluding focus hits, counting a hit only when it is meaningfully
/// closer than the sample), and ambiguity as the worst pairwise IoU of the
/// viewport-clipped screen boxes of fully-in-front bodies, gated by a
/// depth-gap test so overlapping-but-coplanar bodies do not register.
DART_GUI_API ViewQualityReport assessView(
    const std::vector<RenderableDescriptor>& descriptors,
    const OrbitCamera& camera,
    int width,
    int height,
    const std::vector<RenderableId>& focusIds = {},
    const ProjectionOptions& options = {});

} // namespace dart::gui

#endif // DART_GUI_VIEW_QUALITY_HPP_
