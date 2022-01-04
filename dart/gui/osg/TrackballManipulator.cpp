/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/gui/osg/TrackballManipulator.hpp"

#include <osg/Version>

namespace dart {
namespace gui {
namespace osg {

//==============================================================================
TrackballManipulator::TrackballManipulator(int flags)
  : ::osgGA::OrbitManipulator(flags)
{
  setVerticalAxisFixed(false);
  setAllowThrow(false);
}

//==============================================================================
TrackballManipulator::TrackballManipulator(
    const TrackballManipulator& tm, const ::osg::CopyOp& copyOp)
  : ::osg::Object(tm, copyOp),
#if OSG_VERSION_GREATER_OR_EQUAL(3, 3, 2)
    ::osg::Callback(),
#endif
    ::osgGA::OrbitManipulator(tm, copyOp)
{
  // Do nothing
}

//==============================================================================
TrackballManipulator::~TrackballManipulator()
{
  // Do nothing
}

//==============================================================================
bool TrackballManipulator::performMovementLeftMouseButton(
    const double, const double, const double)
{
  // Do nothing
  return false;
}

//==============================================================================
bool TrackballManipulator::performMovementRightMouseButton(
    const double eventTimeDelta, const double dx, const double dy)
{
  return OrbitManipulator::performMovementLeftMouseButton(
      eventTimeDelta, dx, dy);
}

} // namespace osg
} // namespace gui
} // namespace dart
