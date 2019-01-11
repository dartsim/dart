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

#ifndef DART_GUI_OSG_REALTIMEWORLDNODE_HPP_
#define DART_GUI_OSG_REALTIMEWORLDNODE_HPP_

#include <osg/Timer>

#include "dart/gui/osg/WorldNode.hpp"

namespace dart {
namespace gui {
namespace osg {

class RealTimeWorldNode : public WorldNode
{
public:
  /// Construct a world node that will attempt to run a simulation with close
  /// to real-time playback. If a simulation is too computationally expensive,
  /// the simulation might not be able to keep up with real time.
  ///
  /// \param[in] world
  ///   The world to simulate
  ///
  /// \param[in] targetFrequency
  ///   The expected refresh rate. The actual refresh rate may depend on your
  ///   monitor and your computer's display settings.
  ///
  /// \param[in] targetRealTimeFactor
  ///   This factor at which the simulation should run. A value of 1.0 (default)
  ///   means it will try to run at real time. A value 2.0 means the simulation
  ///   will try to run at double real time speed (fast-forward). A value of 0.5
  ///   means the simulation will try to run at half of real-time speed (slowed
  ///   down).
  ///
  /// \param[in] shadowTech
  ///   The shading technique to use when rendering this world.
  RealTimeWorldNode(
      const std::shared_ptr<dart::simulation::World>& world = nullptr,
      const ::osg::ref_ptr<osgShadow::ShadowTechnique>& shadower = nullptr,
      double targetFrequency = 60.0,
      double targetRealTimeFactor = 1.0);

  /// Set the target refresh rate frequency
  void setTargetFrequency(double targetFrequency);

  /// Get the target refresh rate frequency
  double getTargetFrequency() const;

  /// Set the target real time factor
  void setTargetRealTimeFactor(double targetRTF);

  /// Get the target real time factor
  double getTargetRealTimeFactor() const;

  /// Get the real time factor that was achieved in the last refresh cycle. This
  /// may be smaller than target real time factor, because the simulation may
  /// not have had time to reach it.
  double getLastRealTimeFactor() const;

  /// Get the lowest real time factor that has been hit during the simulation.
  double getLowestRealTimeFactor() const;

  /// Get the highest real time factor that has been hit during the simulation.
  double getHighestRealTimeFactor() const;

  /// Turn the text displaying the real time factor on or off
  void setRealTimeFactorDisplay(bool on);

  // Documentation inherited
  void refresh() override;

protected:
  /// Reset each time the simulation is paused
  bool mFirstRefresh;

  /// Keeps track of the time between refreshes
  ::osg::Timer mRefreshTimer;

  /// The target for how much time should elapse between refreshes
  double mTargetRealTimeLapse;

  /// The target for how much simulation time should elapse between refreshes
  double mTargetSimTimeLapse;

  /// The RTF that was achieved in the last refresh cycle
  double mLastRealTimeFactor;

  /// The lowest RTF that has been achieved
  double mLowestRealTimeFactor;

  /// The highest RTF that has been achieved
  double mHighestRealTimeFactor;
};

} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_GUI_OSG_REALTIMEWORLDNODE_HPP_
