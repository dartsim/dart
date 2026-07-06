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

#ifndef DART_GUI_OSG_OFFSCREENVIEWER_HPP_
#define DART_GUI_OSG_OFFSCREENVIEWER_HPP_

#include <osg/BoundingSphere>
#include <osg/Vec3>

#include <string>

namespace dart {
namespace gui {
namespace osg {

class Viewer;

/// Parameters for an off-screen render. The defaults reproduce the framing both
/// DART 6 GUI examples hand-assembled before this helper existed: a 30-degree
/// vertical field of view with 0.1 / 1000.0 clip planes.
struct OffscreenSetup
{
  int width = 640;
  int height = 480;
  double fovYDeg = 30.0;
  double nearClip = 0.1;
  double farClip = 1000.0;
};

/// Camera pose (eye / center / up) for an off-screen shot.
struct OffscreenCamera
{
  ::osg::Vec3 eye;
  ::osg::Vec3 center;
  ::osg::Vec3 up;
};

/// Attaches an off-screen GLX pbuffer to \c viewer's camera and realizes it for
/// headless rendering: the shared traits/pbuffer, camera projection/viewport,
/// single-threaded deterministic drive, realize, and event-queue setup that the
/// GUI examples used to duplicate. This is additive and does not alter any
/// existing Viewer behavior. It does not pin the view matrix; the caller sets
/// \c setViewMatrixAsLookAt (directly or via captureOffscreen) so per-scene
/// framing stays caller-controlled.
///
/// \return \c true on success. Returns \c false, logging an Xvfb hint via
/// \c dterr, when no GL context can be created (no usable DISPLAY / no X
/// server); the caller should then exit non-zero.
bool setUpOffscreenViewer(Viewer& viewer, const OffscreenSetup& setup = {});

/// One-shot convenience for the common "single still" case: set up off-screen,
/// pin the look-at, draw \c warmupFrames frames (so ImGui and the world node
/// build their first frame), then write \c pngPath.
///
/// \return \c true on success, \c false if the off-screen context could not be
/// created (see setUpOffscreenViewer).
bool captureOffscreen(
    Viewer& viewer,
    const std::string& pngPath,
    const ::osg::Vec3& eye,
    const ::osg::Vec3& center,
    const ::osg::Vec3& up,
    const OffscreenSetup& setup = {},
    int warmupFrames = 10);

/// Computes an agent-friendly default camera from a scene bounding sphere: a
/// canonical 3/4 view (azimuth 45, elevation 30) framed to fill the vertical
/// field of view, with distance = radius / sin(fovYDeg / 2) and a z-up axis
/// (matching both examples and DART's gravity convention). Callers that need a
/// different view pass an explicit eye / center / up instead.
OffscreenCamera defaultAgentCamera(
    const ::osg::BoundingSphere& bound,
    double fovYDeg = 30.0,
    double azimuthDeg = 45.0,
    double elevationDeg = 30.0);

} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_GUI_OSG_OFFSCREENVIEWER_HPP_
