#include "OpenGLRenderInterface.h"
#include "utils/LoadOpengl.h"

namespace renderer {

    void OpenGLRenderInterface::initialize() {
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glCullFace(GL_FRONT);
        glDisable(GL_LIGHTING);
        glEnable(GL_DEPTH_TEST);
        //glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
        glShadeModel(GL_SMOOTH);
        clear(Vector3d(1.0, 1.0, 1.0));
    }
    void OpenGLRenderInterface::destroy() {

    }

    void OpenGLRenderInterface::setViewport(int X,int Y,int Width,int Height) {
        glViewport(X, Y, Width, Height);
        mViewportX = X;
        mViewportY = Y;
        mViewportWidth = Width;
        mViewportHeight = Height;

    }
    void OpenGLRenderInterface::getViewport(int& X, int& Y, int &Width, int& Height) const {
        X = mViewportX;
        Y = mViewportY;
        Width = mViewportWidth;
        Height =mViewportHeight;
    }

    void OpenGLRenderInterface::clear(const Vector3d& color) {
        glClearColor((GLfloat)color[0], (GLfloat)color[1], (GLfloat)color[2], 1.0);
        glClear(GL_COLOR_BUFFER_BIT);
    }

    void OpenGLRenderInterface::setDefaultLight() {

    }

    void OpenGLRenderInterface::turnLightsOff() {
        glDisable(GL_LIGHTING);
    }
    void OpenGLRenderInterface::turnLightsOn() {
        //not finished yet
        glEnable(GL_LIGHTING);	
    }
    void OpenGLRenderInterface::setPenColor(const Vector4d& col) {
        glColor4d(col[0], col[1], col[2], col[3]);
    }
    void OpenGLRenderInterface::setPenColor(const Vector3d& col) {
        glColor4d(col[0], col[1], col[2], 1.0);
    }
    void OpenGLRenderInterface::drawCube(const Vector3d& size) {
        glScaled(size(0), size(1), size(2));
        glutSolidCube(1.0);
    }
    void OpenGLRenderInterface::drawEllipsoid(const Vector3d& size) {
        glScaled(size(0), size(1), size(2));
        glutSolidSphere(0.5, 16, 16);
    }
    void OpenGLRenderInterface::pushMatrix() {
        glPushMatrix();
    }
    void OpenGLRenderInterface::popMatrix() {
        glPopMatrix();
    }
    void OpenGLRenderInterface::pushName(int id) {
        glPushName(id);
    }
    void OpenGLRenderInterface::popName() {
        glPopName();
    }
    void OpenGLRenderInterface::translate(const Vector3d& offset) {
        glTranslated(offset[0], offset[1], offset[2]);
    }

    void OpenGLRenderInterface::scale(const Vector3d& scale) {
        glScaled(scale[0], scale[1], scale[2]);
    }
    void OpenGLRenderInterface::rotate(const Vector3d& axis, double rad) {
        glRotated(rad, axis[0], axis[1], axis[2]);
    }

    void OpenGLRenderInterface::saveToImage(const char * filename, DecoBufferType buffType) {

    }
    void OpenGLRenderInterface::readFrameBuffer(DecoBufferType buffType, DecoColorChannel ch, void* pixels) {

    }
    void OpenGLRenderInterface::setMaterial(const Vector3d& diffuse, const Vector3d& specular, double cosinePow) {

    }
    void OpenGLRenderInterface::getMaterial(Vector3d& diffuse, Vector3d& specular, double& cosinePow) const {

    }
    void OpenGLRenderInterface::setDefaultMaterial() {

    }
}
