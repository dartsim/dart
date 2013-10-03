#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/SimWindow.h"

class MyWindow : public dart::yui::SimWindow
{
public:
    MyWindow();
    virtual ~MyWindow();
    
    virtual void timeStepping();
    virtual void drawSkels();
    //  virtual void displayTimer(int _val);
    //  virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);

    void spawnCube(
            const Eigen::Vector3d& _position = Eigen::Vector3d(0.0, 1.0, 0.0),
            const Eigen::Vector3d& _size = Eigen::Vector3d(0.1, 0.1, 0.1),
            double _mass = 1.0);

private:
    Eigen::Vector3d mForce;
};

#endif
