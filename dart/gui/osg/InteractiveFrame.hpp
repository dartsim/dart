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

#ifndef DART_GUI_OSG_INTERACTIVEFRAME_HPP_
#define DART_GUI_OSG_INTERACTIVEFRAME_HPP_

#include "dart/dynamics/SimpleFrame.hpp"

namespace dart {

namespace dynamics {
class MeshShape;
} // namespace dynamics

namespace gui {
namespace osg {

class InteractiveFrame;

class InteractiveTool : public dart::dynamics::SimpleFrame
{
public:

  enum Type {

    LINEAR = 0, /// Tool for linearly constrained translations
    ANGULAR,    /// Tool for rotations
    PLANAR,     /// Tool for planar translations

    NUM_TYPES
  };

  InteractiveTool(InteractiveFrame* frame, double defaultAlpha,
                  const std::string& name);

  /// Set this tool to be enabled or disabled
  void setEnabled(bool enabled);

  /// Returns true if this tool is enabled
  bool getEnabled() const;

  /// Set the alpha of this tool
  void setAlpha(double alpha);

  /// Reset the alpha of this tool back to its default value
  void resetAlpha();

  /// Set the default alpha for this tool. This alpha value will be
  /// applied immediately if reset is true.
  void setDefaultAlpha(double alpha, bool reset=true);

  /// Get the default alpha setting for this tool
  double getDefaultAlpha() const;

  /// Get the InteractiveFrame that this tool belongs to
  InteractiveFrame* getInteractiveFrame();

  /// Get the InteractiveFrame that this tool belongs to
  const InteractiveFrame* getInteractiveFrame() const;

  dart::dynamics::SimpleFrame* addShapeFrame(
      const dart::dynamics::ShapePtr& shape);

  const std::vector<dart::dynamics::SimpleFrame*> getShapeFrames();

  const std::vector<const dart::dynamics::SimpleFrame*> getShapeFrames() const;

  void removeAllShapeFrames();

protected:

  std::vector<std::unique_ptr<dart::dynamics::SimpleFrame>> mSimpleFrames;

  double mDefaultAlpha;

  bool mEnabled;

  InteractiveFrame* mInteractiveFrame;

};

class InteractiveFrame : public dart::dynamics::SimpleFrame
{
public:

  /// Constructor
  InteractiveFrame(
    dart::dynamics::Frame* referenceFrame,
    const std::string& name = "interactive_frame",
    const Eigen::Isometry3d& relativeTransform = Eigen::Isometry3d::Identity(),
    double size_scale=0.2, double thickness_scale=2.0);

  /// Destructor
  virtual ~InteractiveFrame();

  /// Recreate the visuals for this InteractiveFrame according to the specified
  /// scales.
  virtual void resizeStandardVisuals(double size_scale=0.2,
                                     double thickness_scale=2.0);

  /// Get the specified tool
  InteractiveTool* getTool(InteractiveTool::Type tool, std::size_t coordinate);

  /// Get the specified tool
  const InteractiveTool* getTool(InteractiveTool::Type tool,
                                 std::size_t coordinate) const;

  dart::dynamics::SimpleFrame* addShapeFrame(
      const dart::dynamics::ShapePtr& shape);

  const std::vector<dart::dynamics::SimpleFrame*> getShapeFrames();

  const std::vector<const dart::dynamics::SimpleFrame*> getShapeFrames() const;

  void removeAllShapeFrames();

protected:
  /// Creates the standard visualization shapes for InteractiveFrames. Overload
  /// this to create an InteractiveFrame with custom visualization shapes.
  void createStandardVisualizationShapes(double size, double thickness);

  /// Deletes all the visualization shapes held by the InteractiveFrame.
  void deleteAllVisualizationShapes();

  /// Delete all the tool entities belonging to this InteractiveFrame
  void deleteAllTools();

  /// Array of tools belonging to this InteractiveFrame
  InteractiveTool* mTools[InteractiveTool::NUM_TYPES][3];

  std::vector<std::unique_ptr<dart::dynamics::SimpleFrame>> mSimpleFrames;

};

typedef std::shared_ptr<InteractiveFrame> InteractiveFramePtr;

} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_GUI_OSG_INTERACTIVEFRAME_HPP_
