/*
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
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

#ifndef DART_GUI_OSG_MOUSEEVENTHANDLER_HPP_
#define DART_GUI_OSG_MOUSEEVENTHANDLER_HPP_

#include "dart/common/Subject.hpp"
#include "dart/common/Observer.hpp"

#include "dart/gui/osg/DefaultEventHandler.hpp"

namespace dart {
namespace gui {
namespace osg {

class DefaultEventHandler;

class MouseEventHandler : public virtual dart::common::Subject,
                          public virtual dart::common::Observer
{
public:

  friend class DefaultEventHandler;

  inline MouseEventHandler() : mEventHandler(nullptr) { }

  /// Overload this function to set behavior that will get triggered during a
  /// mouse event
  virtual void update() = 0;

protected:

  void handleDestructionNotification(const Subject* _subject) override
  {
    if(mEventHandler == _subject)
      delete this;
  }

  DefaultEventHandler* mEventHandler;

};

} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_GUI_OSG_MOUSEEVENTHANDLER_HPP_
