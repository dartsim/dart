#include "MyWindow.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/Marker.h"
#include <iostream>

using namespace Eigen;
using namespace dart::dynamics;

void MyWindow::displayTimer(int _val)
{
    mWorld->solve();
    glutPostRedisplay();
    glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw()
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    // Draw the body nodes of the skeleton.
    mWorld->getSkel()->draw(mRI);
    // Draw the markers on the skeleton.
    Vector4d color;
    color << 0.95, 0.25, 0.25, 1.0;
    mWorld->getSkel()->drawMarkers(mRI, color, false);
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y)
{
    switch(_key){
    default:
        Win3D::keyboard(_key, _x, _y);
    }
    glutPostRedisplay();
}

void MyWindow::click(int _button, int _state, int _x, int _y) {
    mMouseDown = !mMouseDown;
    int mask = glutGetModifiers();
    if(mMouseDown){
        if (_button == GLUT_LEFT_BUTTON){
            if(mask == GLUT_ACTIVE_SHIFT) {
                mZooming = true;
            } else if (mask == GLUT_ACTIVE_ALT) {
                mActiveMarker = coordsToMarker(_x, _y);
                if (mActiveMarker != -1) {
                    mWorld->createConstraint(mActiveMarker);
                }
            } else {
                mRotate = true;
                mTrackBall.startBall(_x, mWinHeight - _y);
            }
        } else if (_button == GLUT_RIGHT_BUTTON || _button == GLUT_MIDDLE_BUTTON) {
            if(mask == GLUT_ACTIVE_ALT) {
                mActiveMarker = coordsToMarker(_x, _y);                
                if (mActiveMarker != -1) 
                    mWorld->removeConstraint(mActiveMarker);
            } else {
                mTranslate = true;
            }
        } else if (_button == 3 && _state == GLUT_DOWN) { 
            mZoom += 0.1;
        } else if (_button == 4 && _state == GLUT_DOWN) { 
            mZoom -= 0.1;
        }
        mMouseX = _x;
        mMouseY = _y;
    }else{
        mTranslate = false;
        mRotate = false;
        mZooming = false;
        if (mask == GLUT_ACTIVE_ALT)
            mActiveMarker = -1;
    }
    glutPostRedisplay();
}

void MyWindow::drag(int _x, int _y) {
    double deltaX = _x - mMouseX;
    double deltaY = _y - mMouseY;
    mMouseX = _x;
    mMouseY = _y;
    if (mActiveMarker != -1) {
        Vector3d deltaP = reverseProjection(deltaX, deltaY);
        mWorld->modifyConstraint(deltaP);
    }

    if (mRotate) {
        if(deltaX != 0 || deltaY != 0)
            mTrackBall.updateBall(_x, mWinHeight - _y);
    }
    if (mTranslate) {
        Matrix3d rot = mTrackBall.getRotationMatrix();
        mTrans += rot.transpose() * Vector3d(deltaX, -deltaY, 0.0);
    }
    if (mZooming) {
        mZoom += deltaY * 0.01;
    }

    glutPostRedisplay();
}

int MyWindow::coordsToMarker(int _x, int _y) {
    GLuint selectBuf[512];
    GLint hits;
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    glSelectBuffer(512, selectBuf);
    glRenderMode(GL_SELECT);
    glInitNames();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPickMatrix( _x, mWinHeight -_y, 2.0, 2.0, viewport);
    gluPerspective(mPersp, (double)mWinWidth/(double)mWinHeight,0.1,10.0);
    gluLookAt(mEye[0],mEye[1],mEye[2],0.0,0.0,-1.0, mUp[0], mUp[1], mUp[2]);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    mTrackBall.applyGLRotation();
    glScalef(mZoom,mZoom,mZoom);
    glTranslatef(mTrans[0]*0.001, mTrans[1]*0.001, mTrans[2]*0.001);
    mWorld->getSkel()->drawMarkers(mRI);

    hits = glRenderMode(GL_RENDER);

    if(hits) {
        return processHits(hits, selectBuf);
    } else {
        return -1;
    }
}

Vector3d MyWindow::reverseProjection(double _x, double _y) {
    Matrix3d rot = mTrackBall.getRotationMatrix();
    Vector3d deltaP = rot.transpose() * Vector3d(_x, -_y, 0.0) / 1000.0;
    deltaP /= mZoom;
    return deltaP;
}

int MyWindow::processHits(GLint _hits, GLuint _buffer[]) {

    GLfloat minz = 9999999;
    GLfloat tempz;
    GLuint *ptr;
    ptr = _buffer;
    int selectedMarker;

    for(int i = 0; i < _hits; i++){
        ptr++;
        tempz = (float)*ptr / 0x7fffffff;

        if(tempz < minz){
            minz = tempz;
            selectedMarker =* (ptr + 2); 
        }
        ptr += 3;
    }
    return selectedMarker;
}
