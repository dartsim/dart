#ifndef OPENGL_RENDER_INTERFACE_H
#define OPENGL_RENDER_INTERFACE_H


#include <list>
#include <vector>
#include "RenderInterface.h"

using namespace std;

namespace renderer {


	class OpenGLRenderInterface : public RenderInterface
    {

    public:
        OpenGLRenderInterface(){}
        virtual ~OpenGLRenderInterface(){}

        virtual void Initialize();
        virtual void Destroy();

        virtual void SetViewport(int X,int Y,int Width,int Height);
        virtual void GetViewport(int& X, int& Y, int &Width, int& Height) const;

        virtual void Clear(const Vector3d& color);

        virtual void SetDefaultLight();
        virtual void TurnOffLights();
        virtual void TurnOnLights();

        virtual void SetMaterial(const Vector3d& diffuse, const Vector3d& specular, double cosinePow);
        virtual void GetMaterial(Vector3d& diffuse, Vector3d& specular, double& cosinePow) const;
        virtual void SetDefaultMaterial();

        virtual void PushMatrix();
        virtual void PopMatrix();
        virtual void PushName(int id);
        virtual void PopName();

        virtual void Translate(const Vector3d& offset); //glTranslate 
        virtual void Rotate(const Vector3d& axis, double rad); //glRotate
        virtual void Scale(const Vector3d& scale); //glScale

        virtual void DrawEllipsoid(const Vector3d& size);
        virtual void DrawCube(const Vector3d& size);

        virtual void SetPenColor(const Vector4d& col);
        virtual void SetPenColor(const Vector3d& col);

        virtual void SaveToImage(const char * filename, DecoBufferType buffType = BT_Back);
        virtual void ReadFrameBuffer(DecoBufferType buffType, DecoColorChannel ch, void* pixels);

    private:
        int mViewportX, mViewportY, mViewportWidth, mViewportHeight;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace renderer


#endif
