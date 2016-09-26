/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/gui/RenderInterface.hpp"

namespace dart {
namespace gui {

void RenderInterface::initialize()
{
}

void RenderInterface::destroy()
{
}

void RenderInterface::setViewport(int /*_x*/,int /*_y*/,int /*_width*/,int /*_height*/)
{
}

void RenderInterface::getViewport(int& /*_x*/, int& /*_y*/,int& /*_width*/,int& /*_height*/) const
{
}

void RenderInterface::clear(const Eigen::Vector3d& /*_color*/)
{
}

void RenderInterface::setMaterial(const Eigen::Vector3d& /*_diffuse*/, const Eigen::Vector3d& /*_specular*/, double /*_cosinePow*/)
{

}

void RenderInterface::getMaterial(Eigen::Vector3d& /*_diffuse*/, Eigen::Vector3d& /*_specular*/, double& /*cosinePow*/) const
{

}

void RenderInterface::setDefaultMaterial()
{

}

void RenderInterface::pushMatrix()
{
}

void RenderInterface::popMatrix()
{
}

void RenderInterface::pushName(int /*_id*/)
{
}

void RenderInterface::popName()
{
}

void RenderInterface::translate(const Eigen::Vector3d& /*_offset*/)
{
}

void RenderInterface::rotate(const Eigen::Vector3d& /*_axis*/, double /*_rad*/)
{
}

void RenderInterface::transform(const Eigen::Isometry3d& /*_transform*/)
{
}

void RenderInterface::scale(const Eigen::Vector3d& /*_scale*/)
{
}

void RenderInterface::drawSphere(double /*_radius*/)
{
}

void RenderInterface::drawEllipsoid(const Eigen::Vector3d& /*_size*/)
{
}

void RenderInterface::drawMesh(const Eigen::Vector3d& /*_scale*/, const aiScene* /*_mesh*/)
{
}

void RenderInterface::drawSoftMesh(const aiMesh* /*mesh*/)
{
  // Do nothing
}

void RenderInterface::drawList(unsigned int /*indeX*/)
{
}

void RenderInterface::drawLineSegments(const std::vector<Eigen::Vector3d>&,
                                       const Eigen::aligned_vector<Eigen::Vector2i>&)
{
}

unsigned int RenderInterface::compileDisplayList(const Eigen::Vector3d& /*_size*/, const aiScene* /*_mesh*/)
{
    return 0;
}

void RenderInterface::drawCube(const Eigen::Vector3d& /*_size*/)
{
}

void RenderInterface::drawCylinder(double /*_radius*/, double /*_height*/)
{
}

void RenderInterface::drawCapsule(double /*_radius*/, double /*_height*/)
{
}

void RenderInterface::drawCone(double /*_radius*/, double /*_height*/)
{
}

void RenderInterface::setPenColor(const Eigen::Vector4d& /*_col*/)
{
}

void RenderInterface::setPenColor(const Eigen::Vector3d& /*_col*/)
{
}

void RenderInterface::setLineWidth(float)
{
}


void RenderInterface::saveToImage(const char* /*_filename*/, DecoBufferType /*_buffType*/)
{

}

void RenderInterface::readFrameBuffer(DecoBufferType /*_buffType*/, DecoColorChannel /*_ch*/, void* /*_pixels*/)
{

}

} // namespace gui
} // namespace dart
