//
//  MotionBlurSimWindow.cpp
//  dart
//
//  Created by Dong Xu on 1/22/17.
//
//

/////////////////////////////////////////////////////////////////////////
// OpenGL Motion blur require the Accumulate function of OpenGL
// To intergrate this class into the engine
// Change line 86 of dart/gui/glut/Window.cpp
// From  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA |
// GLUT_MULTISAMPLE); to    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE |
// GLUT_RGBA | GLUT_MULTISAMPLE | GLUT_ACCUM);
/////////////////////////////////////////////////////////////////////////

#include "dart/gui/glut/MotionBlurSimWindow.hpp"

#include "dart/dynamics/ConstraintSolver.hpp"
#include "dart/gui/GLFuncs.hpp"
#include "dart/gui/glut/LoadGlut.hpp"

namespace dart {
namespace gui {
namespace glut {

//==============================================================================
MotionBlurSimWindow::MotionBlurSimWindow() : SimWindow() {
  mMotionBlurFrequency = 1;
}

//==============================================================================
MotionBlurSimWindow::~MotionBlurSimWindow() {
  // Do nothing
}

//==============================================================================
void MotionBlurSimWindow::setMotionBlurQuality(int _val) {
  int numIter = mDisplayTimeout / (mWorld->getTimeStep() * 1000);
  if (_val < 0) {
    std::cout << "setMotionBlurQuality: input should be an int between 0-5, "
                 "Regard as 0"
              << std::endl;
    std::cout << "0: No motion blur, 5: Motion blur with highest quality"
              << std::endl;
    mMotionBlurFrequency = numIter;
  } else if (_val > 5) {
    std::cout << "setMotionBlurQuality: input should be an int between 0-5, "
                 "Regard as 5"
              << std::endl;
    std::cout << "0: No motion blur, 5: Motion blur with highest quality"
              << std::endl;
    mMotionBlurFrequency = 1;
  } else if (_val == 0)
    mMotionBlurFrequency = mDisplayTimeout / (mWorld->getTimeStep() * 1000);
  else if (_val == 1)
    mMotionBlurFrequency = std::min(numIter, 16);
  else if (_val == 2)
    mMotionBlurFrequency = std::min(numIter, 8);
  else if (_val == 3)
    mMotionBlurFrequency = std::min(numIter, 4);
  else if (_val == 4)
    mMotionBlurFrequency = std::min(numIter, 2);
  else // _val == 5
    mMotionBlurFrequency = 1;
}

//==============================================================================
void MotionBlurSimWindow::render() {
  int numIter = mDisplayTimeout / (mWorld->getTimeStep() * 1000);
  int numMotionBlurFrames = ceil(
      floor(mDisplayTimeout)
      / (mWorld->getTimeStep() * 1000 * mMotionBlurFrequency));
  if (!mPlay && mSimulating) {
    for (int i = 0; i < numIter; i += mMotionBlurFrequency) {
      for (int j = 0; j < mMotionBlurFrequency; j++) {
        if (i + j < numIter) {
          timeStepping();
          mWorld->bake();
        }
      }

      // Update the camera position before every draw
      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      gluPerspective(
          mPersp,
          static_cast<double>(mWinWidth) / static_cast<double>(mWinHeight),
          0.1,
          10.0);

      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      gluLookAt(
          mEye[0], mEye[1], mEye[2], 0.0, 0.0, -1.0, mUp[0], mUp[1], mUp[2]);
      initGL();

      mTrackBall.applyGLRotation();

      // Draw world origin indicator
      if (!mCapture) {
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_TEXTURE_2D);
        glDisable(GL_LIGHTING);
        glLineWidth(2.0);
        if (mRotate || mTranslate || mZooming) {
          glColor3f(1.0f, 0.0f, 0.0f);
          glBegin(GL_LINES);
          glVertex3f(-0.1f, 0.0f, -0.0f);
          glVertex3f(0.15f, 0.0f, -0.0f);
          glEnd();

          glColor3f(0.0f, 1.0f, 0.0f);
          glBegin(GL_LINES);
          glVertex3f(0.0f, -0.1f, 0.0f);
          glVertex3f(0.0f, 0.15f, 0.0f);
          glEnd();

          glColor3f(0.0f, 0.0f, 1.0f);
          glBegin(GL_LINES);
          glVertex3f(0.0f, 0.0f, -0.1f);
          glVertex3f(0.0f, 0.0f, 0.15f);
          glEnd();
        }
      }

      glScalef(mZoom, mZoom, mZoom);
      glTranslatef(mTrans[0] * 0.001, mTrans[1] * 0.001, mTrans[2] * 0.001);

      initLights();
      draw();

      if (i == 0) {
        // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glAccum(GL_LOAD, 1 / float(numMotionBlurFrames));
      } else {
        glAccum(GL_ACCUM, 1 / float(numMotionBlurFrames));
      }
    } // for loop
  }   // if simulating
  else if (mWorld->getRecording()->getNumFrames() == 0) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(
        mPersp,
        static_cast<double>(mWinWidth) / static_cast<double>(mWinHeight),
        0.1,
        10.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(
        mEye[0], mEye[1], mEye[2], 0.0, 0.0, -1.0, mUp[0], mUp[1], mUp[2]);
    initGL();

    mTrackBall.applyGLRotation();

    // Draw world origin indicator
    if (!mCapture) {
      glEnable(GL_DEPTH_TEST);
      glDisable(GL_TEXTURE_2D);
      glDisable(GL_LIGHTING);
      glLineWidth(2.0);
      if (mRotate || mTranslate || mZooming) {
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(-0.1f, 0.0f, -0.0f);
        glVertex3f(0.15f, 0.0f, -0.0f);
        glEnd();

        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(0.0f, -0.1f, 0.0f);
        glVertex3f(0.0f, 0.15f, 0.0f);
        glEnd();

        glColor3f(0.0f, 0.0f, 1.0f);
        glBegin(GL_LINES);
        glVertex3f(0.0f, 0.0f, -0.1f);
        glVertex3f(0.0f, 0.0f, 0.15f);
        glEnd();
      }
    }

    glScalef(mZoom, mZoom, mZoom);
    glTranslatef(mTrans[0] * 0.001, mTrans[1] * 0.001, mTrans[2] * 0.001);

    initLights();
    draw();

    glAccum(GL_LOAD, 1.0f);
  }

  // Draw trackball indicator
  // Currently, trackball is not counted into the motion blur
  if (mRotate && !mCapture)
    mTrackBall.draw(mWinWidth, mWinHeight);

  // Clear an return the buffer
  glAccum(GL_RETURN, 1.0f);
  glutSwapBuffers();

  if (mCapture)
    screenshot();
}

//==============================================================================
void MotionBlurSimWindow::displayTimer(int _val) {
  if (mPlay) {
    mPlayFrame += 16;
    if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
      mPlayFrame = 0;
  }
  glutPostRedisplay();
  glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

} // namespace glut
} // namespace gui
} // namespace dart
