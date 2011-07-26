/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author     Jie Tan
  Date      07/18/2011
*/

#ifndef RENDERER_RENDERINTERFACE_H
#define RENDERER_RENDERINTERFACE_H

#include <vector>
#include "Light.h"
#include "Camera.h"

namespace renderer{
    enum DecoBufferType {
        BT_Front,
        BT_Back
    };

    enum DecoColorChannel {
        CC_R,
        CC_G,
        CC_B,
        CC_A,
        CC_RGB,
        CC_RGBA
    };

    enum DecoDrawType {
        DT_WireFrame,
        DT_SolidPolygon,
        DT_FrontPolygon,
        DT_BackPolygon,
        DT_Max
    };

    class RenderInterface {
    public:
        RenderInterface(){}
        virtual ~RenderInterface(){}

        virtual void initialize();
        virtual void destroy();

        virtual void setViewport(int _x,int _y,int _width,int _height);
        virtual void getViewport(int& _x, int& _y,int& _width,int& _height) const;

        virtual void clear(const Eigen::Vector3d& _color);

        virtual void setDefaultLight();
        virtual void addLight(Light *_light);
        virtual void eraseAllLights();
        virtual void turnLightsOff();
        virtual void turnLightsOn();

        virtual void setMaterial(const Eigen::Vector3d& _diffuse, const Eigen::Vector3d& _specular, double _cosinePow);
        virtual void getMaterial(Eigen::Vector3d& _diffuse, Eigen::Vector3d& _specular, double& _cosinePow) const;
        virtual void setDefaultMaterial();

        virtual void pushMatrix();
        virtual void popMatrix();
        virtual void pushName(int _id);
        virtual void popName();

        virtual void translate(const Eigen::Vector3d& _offset); //glTranslate 
        virtual void rotate(const Eigen::Vector3d& _axis, double _rad); //glRotate
        virtual void scale(const Eigen::Vector3d& _scale); //glScale

        virtual void drawEllipsoid(const Eigen::Vector3d& _size);
        virtual void drawCube(const Eigen::Vector3d& _size);

        virtual void setPenColor(const Eigen::Vector4d& _col);
        virtual void setPenColor(const Eigen::Vector3d& _col);

        virtual void saveToImage(const char *_filename, DecoBufferType _buffType = BT_Back);
        virtual void readFrameBuffer(DecoBufferType _buffType, DecoColorChannel _ch, void *_pixels);

        virtual Camera* getCamera();

    protected:
        Camera* mCamera;
        std::vector<Light*> mLightList;
    };
} // namespace renderer

#endif // #ifndef RENDERER_RENDERINTERFACE_H
