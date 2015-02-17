#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "dart/gui/Win3D.h"
#include "MyWorld.h"

class MyWindow : public dart::gui::Win3D {
 public:
 MyWindow(): Win3D()
    {
      mBackground[0] = 1.0;
      mBackground[1] = 1.0;
      mBackground[2] = 1.0;
      mBackground[3] = 1.0;
        
      mPlaying = false;
      mSimulating = false;
      mPlayFrame = 0;
      mShowMarkers = true;
            
      mPersp = 30.f;
      mTrans[1] = 1000.f;
      mZoom = 0.25f;
      mDisplayTimeout = 33;
    }

  virtual ~MyWindow() {};
    
  // Override these virtual functions defined in gui::Win3D
  virtual void displayTimer(int _val);
  virtual void draw();
  virtual void keyboard(unsigned char key, int x, int y);
    
  MyWorld* getWorld() {
    return mWorld;
  }

  void setWorld(MyWorld *_world) {
    mWorld = _world;
  }
    
 protected:
  MyWorld* mWorld;
  bool mPlaying;
  bool mSimulating;
  bool mShowMarkers;
  int mPlayFrame;
  std::vector<Eigen::VectorXd> mBakedStates; 
  void bake();
};

#endif
