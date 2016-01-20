/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *            Pete Vieira <pete.vieira@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef OSGKIDO_RENDER_MESHSHAPENODE_HPP
#define OSGKIDO_RENDER_MESHSHAPENODE_HPP

#include <map>

#include <osg/MatrixTransform>
#include <osg/Material>

#include "kido/gui/osg/render/ShapeNode.hpp"

struct aiNode;

namespace kido {

namespace dynamics {
class MeshShape;
} // namespace dynamics

namespace gui {
namespace render {

class osgAiNode;
class MeshShapeGeode;
class MeshShapeGeometry;

class MeshShapeNode : public ShapeNode, public osg::MatrixTransform
{
public:

  MeshShapeNode(std::shared_ptr<kido::dynamics::MeshShape> shape,
                EntityNode* parentEntity);

  void refresh();
  void extractData(bool firstTime);

  osg::Material* getMaterial(size_t index) const;

protected:

  virtual ~MeshShapeNode();

  std::shared_ptr<kido::dynamics::MeshShape> mMeshShape;
  osgAiNode* mRootAiNode;
  std::vector< osg::ref_ptr<osg::Material> > mMaterials;

};

} // namespace render
} // namespace gui
} // namespace kido

#endif // OSGKIDO_RENDER_MESHSHAPENODE_HPP
