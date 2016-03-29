/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#ifndef DART_GUI_OSG_TRACKBALLMANIPULATOR_HPP_
#define DART_GUI_OSG_TRACKBALLMANIPULATOR_HPP_

#include <osgGA/OrbitManipulator>

namespace dart {
namespace gui {
namespace osg {

#define DART_META_Object(library,name) \
        virtual ::osg::Object* cloneType() const { return new name (); } \
        virtual ::osg::Object* clone(const ::osg::CopyOp& copyop) const { return new name (*this,copyop); } \
        virtual bool isSameKindAs(const ::osg::Object* obj) const { return dynamic_cast<const name *>(obj)!=NULL; } \
        virtual const char* libraryName() const { return #library; }\
        virtual const char* className() const { return #name; }
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
  virtual bool performMovementLeftMouseButton(const double eventTimeDelta,
                                              const double dx,
                                              const double dy) override;

  /// Overriding behavior of right mouse button
  virtual bool performMovementRightMouseButton(const double eventTimeDelta,
                                               const double dx,
                                               const double dy) override;

  DART_META_Object( dart-gui-osg, TrackballManipulator )
  // TODO(MXG): Consider applying the META macros to every dart::gui::osg Node
};

} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_GUI_OSG_TRACKBALLMANIPULATOR_HPP_
