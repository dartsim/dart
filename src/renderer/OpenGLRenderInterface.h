#ifndef OPENGL_RENDERINTERFACE_H
#define OPENGL_RENDERINTERFACE_H

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

        virtual void setViewport(int X,int Y,int Width,int Height);
        virtual void getViewport(int& X, int& Y, int &Width, int& Height) const;

        virtual void clear(const Vector3d& color);

        virtual void setDefaultLight();
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

    private:
        int mViewportX, mViewportY, mViewportWidth, mViewportHeight;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace renderer


#endif // OPENGL_RENDERINTERFACE_H
