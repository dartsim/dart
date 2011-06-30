#ifndef _RENDER_INTERFACE_H
#define _RENDER_INTERFACE_H

#include <vector>
#include "light.h"
#include "Camera.h"

namespace renderer
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
	class RenderInterface
	{
	public:
		RenderInterface(){}
		virtual ~RenderInterface(){}

		virtual void Initialize();
		virtual void Destroy();

		virtual void SetViewport(int X,int Y,int Width,int Height);
		virtual void GetViewport(int& X, int& Y, int &Width, int& Height) const;

		virtual void Clear(const Vector3d& color);

		virtual void SetDefaultLight();
		virtual void AddLight(Light* light);
		virtual void EraseAllLights();
		virtual void TurnOffLights();
		virtual void TurnOnLights();

		virtual void SetMaterial(const Vector3d& diffuse, const Vector3d& specular, float cosinePow);
		virtual void GetMaterial(Vector3d& diffuse, Vector3d& specular, float& cosinePow) const;
		virtual void SetDefaultMaterial();

		virtual void PushMatrix();
		virtual void PopMatrix();
		virtual void PushName(int id);
		virtual void PopName();

		virtual void Translate(const Vector3d& offset); //glTranslate 
		virtual void Rotate(const Vector3d& axis, float rad); //glRotate
		virtual void Scale(const Vector3d& scale); //glScale

		virtual void DrawEllipsoid(const Vector3d& size);
		virtual void DrawCube(const Vector3d& size);

		virtual void SetPenColor(const Vector4d& col);
		virtual void SetPenColor(const Vector3d& col);

		virtual void SaveToImage(const char * filename, DecoBufferType buffType = BT_Back);
		virtual void ReadFrameBuffer(DecoBufferType buffType, DecoColorChannel ch, void* pixels);

		virtual Camera* GetCamera()
		{
			return mCamera;
		}
	protected:
        Camera* mCamera;
		std::vector<Light*> mLightList;
	};
}


#endif