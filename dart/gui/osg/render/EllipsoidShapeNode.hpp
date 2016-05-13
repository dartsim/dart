/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_GUI_OSG_RENDER_ELLIPSOIDSHAPENODE_HPP_
#define DART_GUI_OSG_RENDER_ELLIPSOIDSHAPENODE_HPP_

#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>

#include "dart/gui/osg/render/ShapeNode.hpp"

namespace dart {

namespace dynamics {
class EllipsoidShape;
} // namespace dynamics

namespace gui {
namespace osg {
namespace render {

class EllipsoidShapeGeode;
class EllipsoidShapeDrawable;

class EllipsoidShapeNode : public ShapeNode, public ::osg::MatrixTransform
{
public:

  EllipsoidShapeNode(
      std::shared_ptr<dart::dynamics::EllipsoidShape> shape,
      ShapeFrameNode* parent);

  void refresh();
  void extractData(bool firstTime);

protected:

  virtual ~EllipsoidShapeNode();

  std::shared_ptr<dart::dynamics::EllipsoidShape> mEllipsoidShape;
  EllipsoidShapeGeode* mGeode;

};

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_GUI_OSG_RENDER_ELLIPSOIDSHAPENODE_HPP_
