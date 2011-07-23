/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author    Sumit Jain
  Date      07/21/2011
*/

#ifndef YUI_GLFUNCS_H
#define YUI_GLFUNCS_H

#include <Eigen/Eigen>
#include "FreeImage.h"

namespace yui {
    void drawStringOnScreen(float x, float y, const std::string& s);
    void drawArrow3D(const Eigen::Vector3d& pt, const Eigen::Vector3d& dir, const double length, const double thickness, const double arrowThickness=-1);
    void drawArrow2D(const Eigen::Vector2d& pt, const Eigen::Vector2d& vec, double thickness);
    void drawProgressBar(int currFrame, int totalFrame);

    BOOL screenShot(FREE_IMAGE_FORMAT fif, int w, int h, char *fname, bool _antialias);
    BOOL screenShot(FREE_IMAGE_FORMAT fif, int x, int y, int w, int h, char *fname, bool _antialias);
    bool screenShot(int w, int h, char *fname, bool _antialias=false);
} // namespace yui

#endif  //YUI_GLFUNCS_H
