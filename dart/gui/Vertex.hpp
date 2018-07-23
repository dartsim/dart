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

#ifndef DART_GUI_VERTEX_HPP_
#define DART_GUI_VERTEX_HPP_

#include <type_traits>

#include "dart/gui/LoadOpengl.hpp"

namespace dart {
namespace gui {

struct PointVertex
{
  GLfloat position[3];

  static GLuint setVertexAttribute(GLuint attributeOffset = 0u)
  {
    // Necessary to use offsetof(), which only works for standard layout
    static_assert(
        std::is_standard_layout<PointVertex>::value,
        "This class is not a standard layout.");

    glVertexAttribPointer(
        attributeOffset,
        3u,
        GL_FLOAT,
        GL_FALSE,
        sizeof(GLfloat),
        reinterpret_cast<GLvoid*>(offsetof(PointVertex, position)));
    glEnableVertexAttribArray(attributeOffset);

    return 1;
  }
};

struct PointNormalVertex
{
  GLfloat mPosition[3];
  GLfloat mNormal[3];

  static GLuint setVertexAttribute(GLuint attributeOffset = 0u)
  {
    // Necessary to use offsetof(), which only works for standard layout
    static_assert(
        std::is_standard_layout<PointNormalVertex>::value,
        "This class is not a standard layout.");

    glVertexAttribPointer(
        attributeOffset,
        3u,
        GL_FLOAT,
        GL_FALSE,
        sizeof(GLfloat),
        reinterpret_cast<GLvoid*>(offsetof(PointNormalVertex, mPosition)));
    glVertexAttribPointer(
        attributeOffset + 1u,
        3u,
        GL_FLOAT,
        GL_FALSE,
        sizeof(GLfloat),
        reinterpret_cast<GLvoid*>(offsetof(PointNormalVertex, mNormal)));
    glEnableVertexAttribArray(attributeOffset);
    glEnableVertexAttribArray(attributeOffset + 1u);

    return 2;
  }
};

// TODO(JS): Add more vertex types

} // namespace gui
} // namespace dart

#endif // DART_GUI_VERTEX_HPP_
