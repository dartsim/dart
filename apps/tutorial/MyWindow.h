#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"
#include "simulation/SimWindow.h"

class Controller;

class MyWindow : public simulation::SimWindow
{
 public:
 MyWindow(): SimWindow() {
        mForce = Eigen::Vector3d::Zero();
        mImpulseDuration = 0;
    }
    virtual ~MyWindow() {}
    
    virtual void timeStepping();
    virtual void drawSkels();
    //  virtual void displayTimer(int _val);
    //  virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);

    void setController(Controller *_controller)
    {
        mController = _controller;
    }

 private:    
    Controller* mController;
    Eigen::Vector3d mForce;
    int mImpulseDuration;
};

#endif
