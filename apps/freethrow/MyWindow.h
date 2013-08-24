#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"
#include "simulation/SimWindow.h"
#include "Controller.h"

class MyWindow : public simulation::SimWindow
{
 public:
 MyWindow(): SimWindow() {
        mTrans[1] = 0.f;
        mController = NULL;
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
    Controller *mController;

};

#endif
