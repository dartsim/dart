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

#include "dart/gui/osg/DebugOverlay.hpp"

#include <osg/Depth>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/StateSet>
#include <osgText/Font>
#include <osgText/Text>

namespace dart {
namespace gui {
namespace osg {

//==============================================================================
DebugOverlay::DebugOverlay()
  : mLineWidth(2.0f), mCharacterSize(0.08), mNeedUpdate(true)
{
  mGeode = new ::osg::Geode;

  // Debug primitives are annotations: draw them unlit, without depth testing,
  // and in a late render bin so they stay on top of the geometry they name
  // rather than being buried inside opaque bodies (matching DART 7's overlay).
  ::osg::StateSet* stateSet = mGeode->getOrCreateStateSet();
  stateSet->setMode(GL_LIGHTING, ::osg::StateAttribute::OFF);
  stateSet->setMode(GL_DEPTH_TEST, ::osg::StateAttribute::OFF);
  stateSet->setAttributeAndModes(
      new ::osg::Depth(::osg::Depth::ALWAYS, 0.0, 1.0, false));
  stateSet->setRenderBinDetails(11, "RenderBin");

  addChild(mGeode);
}

//==============================================================================
void DebugOverlay::setFont(const std::string& fontPath)
{
  if (mFontPath == fontPath)
    return;

  mFontPath = fontPath;
  mNeedUpdate = true;
}

//==============================================================================
const std::string& DebugOverlay::getFont() const
{
  return mFontPath;
}

//==============================================================================
void DebugOverlay::setLineWidth(float width)
{
  if (mLineWidth == width)
    return;

  mLineWidth = width;
  mNeedUpdate = true;
}

//==============================================================================
float DebugOverlay::getLineWidth() const
{
  return mLineWidth;
}

//==============================================================================
void DebugOverlay::setCharacterSize(double size)
{
  mCharacterSize = size;
}

//==============================================================================
double DebugOverlay::getCharacterSize() const
{
  return mCharacterSize;
}

//==============================================================================
std::size_t DebugOverlay::addLine(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const Eigen::Vector4d& color)
{
  mLines.push_back(Line{start, end, color});
  mNeedUpdate = true;
  return mLines.size() - 1;
}

//==============================================================================
std::size_t DebugOverlay::addLabel(
    const Eigen::Vector3d& position,
    const std::string& text,
    const Eigen::Vector4d& color)
{
  return addLabel(position, text, color, mCharacterSize);
}

//==============================================================================
std::size_t DebugOverlay::addLabel(
    const Eigen::Vector3d& position,
    const std::string& text,
    const Eigen::Vector4d& color,
    double characterSize)
{
  mLabels.push_back(Label{position, text, color, characterSize});
  mNeedUpdate = true;
  return mLabels.size() - 1;
}

//==============================================================================
void DebugOverlay::clear()
{
  if (mLines.empty() && mLabels.empty())
    return;

  mLines.clear();
  mLabels.clear();
  mNeedUpdate = true;
}

//==============================================================================
std::size_t DebugOverlay::getNumLines() const
{
  return mLines.size();
}

//==============================================================================
std::size_t DebugOverlay::getNumLabels() const
{
  return mLabels.size();
}

//==============================================================================
void DebugOverlay::rebuild()
{
  mGeode->removeDrawables(0, mGeode->getNumDrawables());

  if (!mLines.empty()) {
    ::osg::ref_ptr<::osg::Vec3Array> vertices = new ::osg::Vec3Array;
    ::osg::ref_ptr<::osg::Vec4Array> colors = new ::osg::Vec4Array;
    vertices->reserve(mLines.size() * 2);
    colors->reserve(mLines.size() * 2);
    for (const auto& line : mLines) {
      const ::osg::Vec4 color(
          static_cast<float>(line.color[0]),
          static_cast<float>(line.color[1]),
          static_cast<float>(line.color[2]),
          static_cast<float>(line.color[3]));
      vertices->push_back(::osg::Vec3(
          static_cast<float>(line.start.x()),
          static_cast<float>(line.start.y()),
          static_cast<float>(line.start.z())));
      vertices->push_back(::osg::Vec3(
          static_cast<float>(line.end.x()),
          static_cast<float>(line.end.y()),
          static_cast<float>(line.end.z())));
      colors->push_back(color);
      colors->push_back(color);
    }

    ::osg::ref_ptr<::osg::Geometry> geometry = new ::osg::Geometry;
    geometry->setVertexArray(vertices);
    geometry->setColorArray(colors, ::osg::Array::BIND_PER_VERTEX);
    geometry->addPrimitiveSet(new ::osg::DrawArrays(
        ::osg::PrimitiveSet::LINES, 0, static_cast<int>(vertices->size())));
    geometry->getOrCreateStateSet()->setAttributeAndModes(
        new ::osg::LineWidth(mLineWidth));
    mGeode->addDrawable(geometry);
  }

  if (!mLabels.empty()) {
    ::osg::ref_ptr<osgText::Font> font;
    if (!mFontPath.empty())
      font = osgText::readFontFile(mFontPath);

    for (const auto& label : mLabels) {
      ::osg::ref_ptr<osgText::Text> text = new osgText::Text;
      if (font)
        text->setFont(font);
      text->setText(label.text);
      text->setCharacterSize(static_cast<float>(label.characterSize));
      text->setPosition(::osg::Vec3(
          static_cast<float>(label.position.x()),
          static_cast<float>(label.position.y()),
          static_cast<float>(label.position.z())));
      text->setColor(::osg::Vec4(
          static_cast<float>(label.color[0]),
          static_cast<float>(label.color[1]),
          static_cast<float>(label.color[2]),
          static_cast<float>(label.color[3])));
      // Face the camera and stay upright, anchored above the world position.
      text->setAxisAlignment(osgText::TextBase::SCREEN);
      text->setAlignment(osgText::TextBase::CENTER_BOTTOM);
      text->setDataVariance(::osg::Object::DYNAMIC);
      mGeode->addDrawable(text);
    }
  }
}

//==============================================================================
void DebugOverlay::refresh()
{
  if (!mNeedUpdate)
    return;

  rebuild();
  mNeedUpdate = false;
}

} // namespace osg
} // namespace gui
} // namespace dart
