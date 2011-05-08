#ifndef OPENGL_RENDER_intERFACE_H
#define OPENGL_RENDER_intERFACE_H


#include <list>
#include <vector>
#include "light.h"
#include "Camera.h"
using namespace std;

namespace Renderer
{
	enum DecoBufferType
	{
		BT_Front,
		BT_Back
	};

	enum DecoColorChannel
	{
		CC_R,
		CC_G,
		CC_B,
		CC_A,
		CC_RGB,
		CC_RGBA
	};

	enum DecoDrawType
	{
		DT_WireFrame,
		DT_SolidPolygon,
		DT_FrontPolygon,
		DT_BackPolygon,
		DT_Max
	};

	class OpenGLRenderInterface
	{

	public:
		OpenGLRenderInterface();
		~OpenGLRenderInterface();

		void Initialize();
		void Destroy();

		void SetViewport(int X,int Y,int Width,int Height);
		void GetViewport(int& X, int& Y, int &Width, int& Height);

		void Clear(const Vector3d& color);

		void SetDefaultLight();
		void AddLight(const Light& light);
		void EraseAllLights();
		void TurnOffLights();
		void TurnOnLights();

		void SetMaterial(const Vector3d& diffuse, const Vector3d& specular, float cosinePow);
		void GetMaterial(Vector3d& diffuse, Vector3d& specular, float& cosinePow) const;
		void SetDefaultMaterial();

		void PushMatrix();
		void PopMatrix();
		void PushName(int id);
		void PopName();

		void SetLocalToWorldTransform(const Matrix4d& Matrix);
		Matrix4d GetLocalToWorldTransform() const;

		void Translate(const Vector3d& offset); //glTranslate 
		void Rotate(const Vector3d& axis, float rad); //glRotate
		void Scale(const Vector3d& scale); //glScale

		void DrawEllipsoid(const Vector3d& size);
		void DrawCube(const Vector3d& size);
		void DrawPlane(const Vector2d& size);

		void SetPenColor(const Vector4d& col);
		void SetPenColor(const Vector3d& col);

		void SetAntialiasingOption(bool bAntialiasing);
		void SetDrawingOption(DecoDrawType drawingOption);

		void SaveToImage(const char * filename, DecoBufferType buffType = BT_Back);
		void ReadFrameBuffer(DecoBufferType buffType, DecoColorChannel ch, void* pixels);

		Camera& GetCamera()
		{
			return mCamera;
		}

	private:
		int mViewportX, mViewportY, mViewportWidth, mViewportHeight;
		bool mAntialiasingOption;
		DecoDrawType mDrawingOption;
		Matrix4d mLocalToWorld;
//		vector<Matrix4d> mMatrixStack;
		std::vector<Light> mLightList;
		Camera mCamera;
	};
}


#endif
