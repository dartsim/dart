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
#include <Eigen/Geometry>
#include <vsg/all.h>

#include <array>
#include <vector>

namespace dart::collision::experimental {
class Shape;
} // namespace dart::collision::experimental

namespace dart::gui::vsg {

struct DART_GUI_VSG_API GeometryOptions
{
  Eigen::Vector4d color{0.7, 0.7, 0.7, 1.0};
  bool wireframe{false};
  bool twoSided{false};
};

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::Node> createSphere(
    double radius, const GeometryOptions& options = {});

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::Node> createBox(
    const Eigen::Vector3d& size, const GeometryOptions& options = {});

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::Node> createCapsule(
    double radius, double height, const GeometryOptions& options = {});

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::Node> createCylinder(
    double radius, double height, const GeometryOptions& options = {});

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::Node> createCone(
    double radius, double height, const GeometryOptions& options = {});

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::Node> createPlane(
    double width, double height, const GeometryOptions& options = {});

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::Node> createMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::array<unsigned int, 3>>& triangles,
    const GeometryOptions& options = {});

DART_GUI_VSG_API ::vsg::ref_ptr<::vsg::Node> createFromShape(
    const collision::experimental::Shape& shape,
    const GeometryOptions& options = {});

} // namespace dart::gui::vsg
