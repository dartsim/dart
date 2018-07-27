//
//  MotionBlurSimWindow.hpp
//  dart
//
//  Created by Dong Xu on 1/22/17.
//
//

#ifndef DART_GUI_MOTIONBLURSIMWINDOW_HPP_
#define DART_GUI_MOTIONBLURSIMWINDOW_HPP_

#pragma message("This header is deprecated as of DART 6.6. "\
         "Please use dart/gui/glut/MotionBlurSimWindow.hpp instead.")

#include "dart/gui/glut/MotionBlurSimWindow.hpp"
#include "dart/common/Deprecated.hpp"

namespace dart {
namespace gui {

using MotionBlurSimWindow DART_DEPRECATED(6.6) =
  ::dart::gui::glut::MotionBlurSimWindow;

}  // namespace gui
}  // namespace dart

#endif // DART_GUI_MOTIONBLURSIMWINDOW_HPP_
