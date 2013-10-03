#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/SimWindow.h"

/// @brief
class MyWindow : public dart::yui::SimWindow
{
public:
    /// @brief
    MyWindow();

    /// @brief
    virtual ~MyWindow() {}

    /// @brief
    virtual void timeStepping();

    /// @brief
    virtual void keyboard(unsigned char key, int x, int y);

private:
    /// @brief
    Eigen::Vector3d mForce;
};

#endif
