#ifndef RENDERINTERFACE_H
#define RENDERINTERFACE_H

#include <vector>
#include "light.h"
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

        virtual void setViewport(int X,int Y,int Width,int Height);
        virtual void getViewport(int& X, int& Y, int &Width, int& Height) const;

        virtual void clear(const Vector3d& color);

        virtual void setDefaultLight();
        virtual void addLight(Light* light);
        virtual void eraseAllLights();
        virtual void turnLightsOff();
        virtual void turnLightsOn();

        virtual void setMaterial(const Vector3d& diffuse, const Vector3d& specular, double cosinePow);
        virtual void getMaterial(Vector3d& diffuse, Vector3d& specular, double& cosinePow) const;
        virtual void setDefaultMaterial();

        virtual void pushMatrix();
        virtual void popMatrix();
        virtual void pushName(int id);
        virtual void popName();

        virtual void translate(const Vector3d& offset); //glTranslate 
        virtual void rotate(const Vector3d& axis, double rad); //glRotate
        virtual void scale(const Vector3d& scale); //glScale

        virtual void drawEllipsoid(const Vector3d& size);
        virtual void drawCube(const Vector3d& size);

        virtual void setPenColor(const Vector4d& col);
        virtual void setPenColor(const Vector3d& col);

        virtual void saveToImage(const char * filename, DecoBufferType buffType = BT_Back);
        virtual void readFrameBuffer(DecoBufferType buffType, DecoColorChannel ch, void* pixels);

        virtual Camera* getCamera() {
            return mCamera;
        }
    protected:
        Camera* mCamera;
        std::vector<Light*> mLightList;
    };
}

#endif // RENDERINTERFACE_H
