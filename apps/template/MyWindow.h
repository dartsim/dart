#ifndef CNTCTLRNIN_MYWINDOW_H
#define CNTCTLRNIN_MYWINDOW_H

#include "dart/dart.h"
#include "Controller.h"

class MyWindow : public dart::gui::SimWindow 
{
public:
    MyWindow(dart::simulation::WorldPtr world);
    virtual ~MyWindow();
    
    void drawSkels() override;

private:
    std::unique_ptr<Controller> mController;
};

#endif  // CNTCTLRNIN_MYWINDOW_H
