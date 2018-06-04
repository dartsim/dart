//
//  MotionBlurSimWindow.hpp
//  dart
//
//  Created by Dong Xu on 1/22/17.
//
//

#ifndef DART_GUI_GLUT_MOTIONBLURSIMWINDOW_HPP_
#define DART_GUI_GLUT_MOTIONBLURSIMWINDOW_HPP_

#include <vector>
#include <Eigen/Dense>

#include "dart/gui/glut/SimWindow.hpp"

namespace dart {
namespace gui {
namespace glut {

class MotionBlurSimWindow : public glut::SimWindow
{
public:
  /// \brief
  MotionBlurSimWindow();

  /// \brief
  virtual ~MotionBlurSimWindow();
    
  // Set the Quality of Motion Blur
  // Default is 5 (record position of every frame)
  // int from 0 (No motion blur) - 5 (Highest)
  // The function takes value smaller than 0 as 0, larger than 5 as 5
  void setMotionBlurQuality(int _val);

  // Override the render function in dart/gui/Win3D.hpp
  // To draw the motion image
  // Render function is called once per GUI display time
  // but in MotionBlurSimWindow, draw function will run in motion blur frequency
  void render() override;

  // Override the display timer,
  // Move the part of "step" in world function to the render function
  void displayTimer(int _val) override;
    
protected:
  // Determines the frequency of the motion blur
  // Default is 1, which means motion blur effect has the highest quality
  // When set to m, motion blur record data every m frames
  int mMotionBlurFrequency;

}; // End of Class Definition

} // namespace glut
} // namespace gui
} // namespace dart

#endif // DART_GUI_GLUT_MOTIONBLURSIMWINDOW_HPP_
