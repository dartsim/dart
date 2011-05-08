#ifndef _GLFUNCS_
#define _GLFUNCS_

#include <Eigen/Eigen>

void drawStringOnScreen(float x, float y, std::string s);
void drawArrow(const Eigen::Vector3d& pt, const Eigen::Vector3d& dir, const double length, const double thickness, const double arrowThickness=-1);

#endif
