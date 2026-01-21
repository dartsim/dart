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

#pragma once

#include <dart/gui/vsg/export.hpp>
#include <dart/gui/vsg/materials.hpp>

#include <Eigen/Core>
#include <vsg/all.h>

#include <vector>

namespace dart::gui::vsg {

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::Node> createPoint(
    const Eigen::Vector3d& position,
    double size = 0.02,
    const Eigen::Vector4d& color = colors::Red);

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::Node> createLine(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const Eigen::Vector4d& color = colors::White);

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::Node> createArrow(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& direction,
    double length = 0.1,
    const Eigen::Vector4d& color = colors::Blue);

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::Node> createPoints(
    const std::vector<Eigen::Vector3d>& positions,
    double size = 0.02,
    const Eigen::Vector4d& color = colors::Red);

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::Node> createLines(
    const std::vector<Eigen::Vector3d>& starts,
    const std::vector<Eigen::Vector3d>& ends,
    const Eigen::Vector4d& color = colors::White);

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::Node> createArrows(
    const std::vector<Eigen::Vector3d>& starts,
    const std::vector<Eigen::Vector3d>& directions,
    double length = 0.1,
    const Eigen::Vector4d& color = colors::Blue);

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::Node> createAxes(
    double length = 1.0, double thickness = 0.02);

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::Node> createGrid(
    double size = 10.0,
    double spacing = 1.0,
    const Eigen::Vector4d& color = colors::Gray);

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::Node> createWireframeBox(
    const Eigen::Vector3d& min,
    const Eigen::Vector3d& max,
    const Eigen::Vector4d& color = colors::Yellow);

} // namespace dart::gui::vsg
