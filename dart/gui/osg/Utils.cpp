/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include "dart/gui/osg/Utils.hpp"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osg/Texture2D>

namespace dart::gui::osg {

//==============================================================================
Eigen::Vector3f osgToEigVec3(const ::osg::Vec3f& vec)
{
  return Eigen::Vector3f(vec[0], vec[1], vec[2]);
}

//==============================================================================
Eigen::Vector3d osgToEigVec3(const ::osg::Vec3d& vec)
{
  return Eigen::Vector3d(vec[0], vec[1], vec[2]);
}

//==============================================================================
Eigen::Vector4f osgToEigVec4(const ::osg::Vec4f& vec)
{
  return Eigen::Vector4f(vec[0], vec[1], vec[2], vec[3]);
}

//==============================================================================
Eigen::Vector4d osgToEigVec4(const ::osg::Vec4d& vec)
{
  return Eigen::Vector4d(vec[0], vec[1], vec[2], vec[3]);
}

//==============================================================================
::osg::Camera* createRttCamera(
    ::osg::Camera::BufferComponent buffer, ::osg::Texture* tex, bool isAbsolute)
{
  ::osg::ref_ptr<::osg::Camera> camera = new ::osg::Camera;
  camera->setClearColor(::osg::Vec4());
  camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  camera->setRenderTargetImplementation(::osg::Camera::FRAME_BUFFER_OBJECT);
  camera->setRenderOrder(::osg::Camera::PRE_RENDER);

  if (tex) {
    tex->setFilter(::osg::Texture2D::MIN_FILTER, ::osg::Texture2D::LINEAR);
    tex->setFilter(::osg::Texture2D::MAG_FILTER, ::osg::Texture2D::LINEAR);
    camera->setViewport(0, 0, tex->getTextureWidth(), tex->getTextureHeight());
    camera->attach(buffer, tex);
  }

  if (isAbsolute) {
    camera->setReferenceFrame(::osg::Transform::ABSOLUTE_RF);
    camera->setProjectionMatrix(::osg::Matrix::ortho2D(0.0, 1.0, 0.0, 1.0));
    camera->setViewMatrix(::osg::Matrix::identity());
    camera->addChild(createScreenQuad(1.0f, 1.0f));
  }

  return camera.release();
}

//==============================================================================
::osg::Camera* createHudCamera(
    double left, double right, double bottom, double top)
{
  ::osg::ref_ptr<::osg::Camera> camera = new ::osg::Camera;

  camera->setReferenceFrame(::osg::Transform::ABSOLUTE_RF);
  camera->setClearMask(GL_DEPTH_BUFFER_BIT);
  camera->setRenderOrder(::osg::Camera::POST_RENDER);
  camera->setAllowEventFocus(false);
  camera->setProjectionMatrix(::osg::Matrix::ortho2D(left, right, bottom, top));
  camera->getOrCreateStateSet()->setMode(
      GL_LIGHTING, ::osg::StateAttribute::OFF);

  return camera.release();
}

//==============================================================================
::osg::Geode* createScreenQuad(float width, float height, float scale)
{
  ::osg::Geometry* geom = ::osg::createTexturedQuadGeometry(
      ::osg::Vec3(),
      ::osg::Vec3(width, 0.0f, 0.0f),
      ::osg::Vec3(0.0f, height, 0.0f),
      0.0f,
      0.0f,
      width * scale,
      height * scale);

  ::osg::ref_ptr<::osg::Geode> quad = new ::osg::Geode;
  quad->addDrawable(geom);

  int values = ::osg::StateAttribute::OFF | ::osg::StateAttribute::PROTECTED;
  quad->getOrCreateStateSet()->setAttribute(
      new ::osg::PolygonMode(
          ::osg::PolygonMode::FRONT_AND_BACK, ::osg::PolygonMode::FILL),
      values);
  quad->getOrCreateStateSet()->setMode(GL_LIGHTING, values);

  return quad.release();
}

} // namespace dart::gui::osg
