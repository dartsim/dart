/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_GUI_OSG_TRACKBALLMANIPULATOR_HPP_
#define DART_GUI_OSG_TRACKBALLMANIPULATOR_HPP_

#include <osgGA/OrbitManipulator>

namespace dart {
namespace gui {
namespace osg {

#define DART_META_Object(library,name) \
        ::osg::Object* cloneType() const override { return new name (); } \
        ::osg::Object* clone(const ::osg::CopyOp& copyop) const override { return new name (*this,copyop); } \
        bool isSameKindAs(const ::osg::Object* obj) const override { return dynamic_cast<const name *>(obj)!=NULL; } \
        const char* libraryName() const override { return #library; }\
        const char* className() const override { return #name; }
// TODO(JS): Copied from osg/Object. Due to the namespace conflict between osg
// and dart::gui::osg, we need to explicitly specify the root namespace osg as
// ::osg

class OSGGA_EXPORT TrackballManipulator : public ::osgGA::OrbitManipulator
{
public:
  /// Constructor
  TrackballManipulator(int flags=DEFAULT_SETTINGS);

  /// Copy-constructor
  TrackballManipulator(const TrackballManipulator& tm,
                       const ::osg::CopyOp& copyOp = ::osg::CopyOp::SHALLOW_COPY);

  /// Destructor
  virtual ~TrackballManipulator();

  /// Overriding behavior of left mouse button
  bool performMovementLeftMouseButton(const double eventTimeDelta,
                                      const double dx,
                                      const double dy) override;

  /// Overriding behavior of right mouse button
  bool performMovementRightMouseButton(const double eventTimeDelta,
                                       const double dx,
                                       const double dy) override;

  DART_META_Object( dart-gui-osg, TrackballManipulator )
  // TODO(MXG): Consider applying the META macros to every dart::gui::osg Node
};

} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_GUI_OSG_TRACKBALLMANIPULATOR_HPP_
