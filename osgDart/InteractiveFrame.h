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

#ifndef OSGDART_INTERACTIVEFRAME_H
#define OSGDART_INTERACTIVEFRAME_H

#include "dart/dynamics/SimpleFrame.h"

namespace dart {
namespace dynamics {
class MeshShape;
} // namespace dynamics
} // namespace dart

namespace osgDart
{

class InteractiveFrame : public dart::dynamics::SimpleFrame
{
public:

  enum class Shape : int {

    ARROW = 0,
    RING,
    PLANE,

    NUM_TYPES
  };

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
  void resizeStandardVisuals(double size_scale=0.2, double thickness_scale=2.0);

  /// Set a shape type to be enabled or disabled. Specify the shape's coordinate
  /// (x=0, y=1, z=2)
  void setShapeEnabled(Shape shape, size_t coordinate, bool enabled);

  /// Set a shape type to be enabled or disabled. This applies to all shapes of
  /// that type
  void setShapeEnabled(Shape shape, bool enabled);

  /// Returns true if the specified shape of the specified coordinate is enabled
  bool isShapeEnabled(Shape shape, size_t coordinate) const;

  /// Set the alpha of the specified shape
  void setShapeAlpha(Shape shape, size_t coordinate, double alpha);

  /// Reset the alpha of the specified shape back to its default value
  void resetShapeAlpha(Shape shape, size_t coordinate);

  /// Set the default alpha for the specified shape. This alpha value will be
  /// applied immediately if reset is true.
  void setDefaultAlpha(Shape shape, size_t coordinate,
                       double alpha, bool reset=true);

  /// Get the default alpha setting for the specified shape
  double getDefaultAlpha(Shape shape, size_t coordinate) const;

  /// Get the abstract Shape that corresponds to the specified Shape enumeration
  dart::dynamics::Shape* getShape(Shape shape, size_t coordinate) const;

  /// Get the MeshShape that corresponds to the specified Shape enumeration
  dart::dynamics::MeshShape* getMeshShape(Shape shape, size_t coordinate) const;

protected:
  /// Creates the standard visualization shapes for InteractiveFrames. Overload
  /// this to create an InteractiveFrame with custom visualization shapes.
  void createStandardVisualizationShapes(double size, double thickness);

  /// Deletes all the visualization shapes held by the InteractiveFrame.
  void deleteAllVisualizationShapes();

  /// Keeps track of which shapes are enabled
  bool mEnabledShapes[(int)Shape::NUM_TYPES][3];

  double mDefaultAlphas[(int)Shape::NUM_TYPES][3];

};

} // namespace osgDart

#endif // OSGDART_INTERACTIVEFRAME_H
