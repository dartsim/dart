#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/SimWindow.h"

class MyWindow : public dart::yui::SimWindow
{
public:
    MyWindow() : SimWindow() {}
    virtual ~MyWindow() {}
    
    virtual void timeStepping();
    //  virtual void drawSkels();
    //  virtual void displayTimer(int _val);
    //  virtual void draw();
    //  virtual void keyboard(unsigned char key, int x, int y);

private:
    Eigen::VectorXd computeDamping();
};

#endif
