/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author     Jie Tan
  Date      07/18/2011
*/

#ifndef RENDERER_OPENGLRENDERINTERFACE_H
#define RENDERER_OPENGLRENDERINTERFACE_H

#include <list>
#include <vector>
#include "RenderInterface.h"

using namespace std;

namespace renderer {
    class OpenGLRenderInterface : public RenderInterface {

    public:
        OpenGLRenderInterface(){}
        virtual ~OpenGLRenderInterface(){}

        virtual void initialize();
        virtual void destroy();

        virtual void setViewport(int _x,int _y,int _width,int _height);
        virtual void getViewport(int& _x, int& _y, int& _width, int& _height) const;

        virtual void clear(const Vector3d& _color);

        virtual void setDefaultLight();
        virtual void turnLightsOff();
        virtual void turnLightsOn();

        virtual void setMaterial(const Vector3d& _diffuse, const Vector3d& _specular, double _cosinePow);
        virtual void getMaterial(Vector3d& _diffuse, Vector3d& _specular, double& _cosinePow) const;
        virtual void setDefaultMaterial();

        virtual void pushMatrix();
        virtual void popMatrix();
        virtual void pushName(int _id);
        virtual void popName();

        virtual void translate(const Vector3d& _offset); //glTranslate 
        virtual void rotate(const Vector3d& _axis, double _rad); //glRotate
        virtual void scale(const Vector3d& _scale); //glScale

        virtual void drawEllipsoid(const Vector3d& _size);
        virtual void drawCube(const Vector3d& _size);

        virtual void setPenColor(const Vector4d& _col);
        virtual void setPenColor(const Vector3d& _col);

        virtual void saveToImage(const char *_filename, DecoBufferType _buffType = BT_Back);
        virtual void readFrameBuffer(DecoBufferType _buffType, DecoColorChannel _ch, void *_pixels);

    private:
        int mViewportX, mViewportY, mViewportWidth, mViewportHeight;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace renderer


#endif // #ifndef RENDERER_OPENGLRENDERINTERFACE_H
