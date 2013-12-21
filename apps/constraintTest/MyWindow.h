#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/SimWindow.h"

namespace dart {

namespace constraint {
    class Constraint;
}
}

class MyWindow : public dart::yui::SimWindow
{
public:
    MyWindow() : SimWindow() {
        mHeadConstraint = NULL;
        mTailConstraint = NULL;
    }
    virtual ~MyWindow() {}
    
    virtual void timeStepping();
    //  virtual void drawSkels();
    //  virtual void displayTimer(int _val);
    //  virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);

private:
    Eigen::VectorXd computeDamping();
    dart::constraint::Constraint* addHeadConstraint();
    dart::constraint::Constraint* addTailConstraint();

    dart::constraint::Constraint* mHeadConstraint;
    dart::constraint::Constraint* mTailConstraint;
};

#endif
