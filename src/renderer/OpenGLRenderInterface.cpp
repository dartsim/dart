/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author     Jie Tan
  Date      07/18/2011
*/

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

    void OpenGLRenderInterface::setViewport(int _x,int _y,int _width,int _height) {
        glViewport(_x, _y, _width, _height);
        mViewportX = _x;
        mViewportY = _y;
        mViewportWidth = _width;
        mViewportHeight = _height;
    }

    void OpenGLRenderInterface::getViewport(int& _x, int& _y, int& _width, int& _height) const {
        _x = mViewportX;
        _y = mViewportY;
        _width = mViewportWidth;
        _height =mViewportHeight;
    }

    void OpenGLRenderInterface::clear(const Vector3d& _color) {
        glClearColor((GLfloat)_color[0], (GLfloat)_color[1], (GLfloat)_color[2], 1.0);
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

	void OpenGLRenderInterface::setMaterial(const Vector3d& _diffuse, const Vector3d& _specular, double _cosinePow) {

	}

	void OpenGLRenderInterface::getMaterial(Vector3d& _diffuse, Vector3d& _specular, double& _cosinePow) const {

	}

	void OpenGLRenderInterface::setDefaultMaterial() {

	}

	void OpenGLRenderInterface::pushMatrix() {
		glPushMatrix();
	}

	void OpenGLRenderInterface::popMatrix() {
		glPopMatrix();
	}

	void OpenGLRenderInterface::pushName(int _id) {
		glPushName(_id);
	}

	void OpenGLRenderInterface::popName() {
		glPopName();
	}

	void OpenGLRenderInterface::translate(const Vector3d& _offset) {
		glTranslated(_offset[0], _offset[1], _offset[2]);
	}

	void OpenGLRenderInterface::rotate(const Vector3d& _axis, double _rad) {
		glRotated(_rad, _axis[0], _axis[1], _axis[2]);
	}

	void OpenGLRenderInterface::scale(const Vector3d& _scale) {
		glScaled(_scale[0], _scale[1], _scale[2]);
	}

	void OpenGLRenderInterface::drawEllipsoid(const Vector3d& _size) {
		glScaled(_size(0), _size(1), _size(2));
		glutSolidSphere(0.5, 16, 16);
	}

	void OpenGLRenderInterface::drawCube(const Vector3d& _size) {
		glScaled(_size(0), _size(1), _size(2));
		glutSolidCube(1.0);
	}

    void OpenGLRenderInterface::setPenColor(const Vector4d& _col) {
        glColor4d(_col[0], _col[1], _col[2], _col[3]);
    }

    void OpenGLRenderInterface::setPenColor(const Vector3d& _col) {
        glColor4d(_col[0], _col[1], _col[2], 1.0);
    }

    void OpenGLRenderInterface::readFrameBuffer(DecoBufferType _buffType, DecoColorChannel _ch, void *_pixels) {

	}

	void OpenGLRenderInterface::saveToImage(const char *_filename, DecoBufferType _buffType) {

	}

} // namespace renderer
