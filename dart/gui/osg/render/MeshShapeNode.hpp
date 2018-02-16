/*
 * Copyright (c) 2011-2017, The DART development contributors
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

#ifndef DART_GUI_OSG_RENDER_MESHSHAPENODE_HPP_
#define DART_GUI_OSG_RENDER_MESHSHAPENODE_HPP_

#include <map>

#include <osg/MatrixTransform>
#include <osg/Material>

#include "dart/gui/osg/render/ShapeNode.hpp"

struct aiNode;

namespace dart {

namespace dynamics {
class MeshShape;
} // namespace dynamics

namespace gui {
namespace osg {
namespace render {

class osgAiNode;
class MeshShapeGeode;
class MeshShapeGeometry;

class MeshShapeNode : public ShapeNode, public ::osg::MatrixTransform
{
public:

  MeshShapeNode(std::shared_ptr<dart::dynamics::MeshShape> shape,
                ShapeFrameNode* parentNode);

  void refresh();
  void extractData(bool firstTime);

  ::osg::Material* getMaterial(std::size_t index) const;
  std::vector<std::string> getTextureImagePaths(std::size_t index) const;

protected:

  virtual ~MeshShapeNode();

  std::shared_ptr<dart::dynamics::MeshShape> mMeshShape;
  osgAiNode* mRootAiNode;
  std::vector< ::osg::ref_ptr<::osg::Material> > mMaterials;
  std::vector< std::vector<std::string> > mTextureImageArrays;

};

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_GUI_OSG_RENDER_MESHSHAPENODE_HPP_
