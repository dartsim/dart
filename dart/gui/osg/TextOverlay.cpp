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

#include "dart/gui/osg/TextOverlay.hpp"

#include <osg/Depth>
#include <osg/StateSet>
#include <osgText/Font>
#include <osgText/Text>

namespace dart {
namespace gui {
namespace osg {

//==============================================================================
TextOverlay::TextOverlay() : mCharacterSize(20.0), mNeedUpdate(true)
{
  mGeode = new ::osg::Geode;

  // Labels are annotations: draw them unlit, without depth testing, and in the
  // transparent bin so they stay legible on top of the geometry they name.
  ::osg::StateSet* stateSet = mGeode->getOrCreateStateSet();
  stateSet->setMode(GL_LIGHTING, ::osg::StateAttribute::OFF);
  stateSet->setMode(GL_DEPTH_TEST, ::osg::StateAttribute::OFF);
  stateSet->setRenderingHint(::osg::StateSet::TRANSPARENT_BIN);
  stateSet->setAttributeAndModes(
      new ::osg::Depth(::osg::Depth::LESS, 0.0, 1.0, false));

  addChild(mGeode);
}

//==============================================================================
void TextOverlay::setFont(const std::string& fontPath)
{
  if (mFontPath == fontPath)
    return;

  mFontPath = fontPath;
  mNeedUpdate = true;
}

//==============================================================================
const std::string& TextOverlay::getFont() const
{
  return mFontPath;
}

//==============================================================================
void TextOverlay::setCharacterSize(double size)
{
  mCharacterSize = size;
}

//==============================================================================
double TextOverlay::getCharacterSize() const
{
  return mCharacterSize;
}

//==============================================================================
std::size_t TextOverlay::addLabel(
    const Eigen::Vector3d& position,
    const std::string& text,
    const Eigen::Vector4d& color)
{
  return addLabel(position, text, color, mCharacterSize);
}

//==============================================================================
std::size_t TextOverlay::addLabel(
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
void TextOverlay::clear()
{
  if (mLabels.empty())
    return;

  mLabels.clear();
  mNeedUpdate = true;
}

//==============================================================================
std::size_t TextOverlay::getNumLabels() const
{
  return mLabels.size();
}

//==============================================================================
void TextOverlay::rebuild()
{
  mGeode->removeDrawables(0, mGeode->getNumDrawables());

  ::osg::ref_ptr<osgText::Font> font;
  if (!mFontPath.empty())
    font = osgText::readFontFile(mFontPath);

  for (const auto& label : mLabels) {
    ::osg::ref_ptr<osgText::Text> text = new osgText::Text;
    if (font)
      text->setFont(font);
    text->setText(label.text);
    // Size labels in screen pixels so they stay legible regardless of how far
    // the camera is from the anchor. The position stays world-space.
    text->setCharacterSize(static_cast<float>(label.characterSize));
    text->setCharacterSizeMode(osgText::TextBase::SCREEN_COORDS);
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

//==============================================================================
void TextOverlay::refresh()
{
  if (!mNeedUpdate)
    return;

  rebuild();
  mNeedUpdate = false;
}

} // namespace osg
} // namespace gui
} // namespace dart
