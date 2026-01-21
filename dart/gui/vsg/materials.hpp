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

#include <Eigen/Core>
#include <vsg/all.h>

namespace dart::gui::vsg {

namespace colors {

inline const Eigen::Vector4d Red{1.0, 0.2, 0.2, 1.0};
inline const Eigen::Vector4d Green{0.2, 1.0, 0.2, 1.0};
inline const Eigen::Vector4d Blue{0.2, 0.2, 1.0, 1.0};
inline const Eigen::Vector4d Yellow{1.0, 1.0, 0.2, 1.0};
inline const Eigen::Vector4d Cyan{0.2, 1.0, 1.0, 1.0};
inline const Eigen::Vector4d Magenta{1.0, 0.2, 1.0, 1.0};
inline const Eigen::Vector4d Orange{1.0, 0.5, 0.2, 1.0};
inline const Eigen::Vector4d White{1.0, 1.0, 1.0, 1.0};
inline const Eigen::Vector4d Black{0.0, 0.0, 0.0, 1.0};
inline const Eigen::Vector4d Gray{0.5, 0.5, 0.5, 1.0};
inline const Eigen::Vector4d LightGray{0.7, 0.7, 0.7, 1.0};
inline const Eigen::Vector4d DarkGray{0.3, 0.3, 0.3, 1.0};

} // namespace colors

struct DART_GUI_VSG_API MaterialOptions
{
  Eigen::Vector4d color{0.7, 0.7, 0.7, 1.0};
  bool wireframe{false};
  bool twoSided{false};
};

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::StateGroup> createStateGroup(
    const MaterialOptions& options = {});

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::vec4Value> createColorValue(
    const Eigen::Vector4d& color);

} // namespace dart::gui::vsg
