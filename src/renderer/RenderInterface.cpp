#include "RenderInterface.h"

namespace renderer
{
	void RenderInterface::Initialize()
	{
	}
	void RenderInterface::Destroy()
	{
	}

	void RenderInterface::SetViewport(int X,int Y,int Width,int Height)
	{
	}
	void RenderInterface::GetViewport(int& X, int& Y, int &Width, int& Height) const
	{
	}

	void RenderInterface::Clear(const Vector3d& color)
	{
	}

	void RenderInterface::SetDefaultLight()
	{
	}
	void RenderInterface::AddLight(Light* light)
	{
		mLightList.push_back(light);
	}
	void RenderInterface::EraseAllLights()
	{
		mLightList.clear();
	}
	void RenderInterface::TurnOffLights()
	{
	}
	void RenderInterface::TurnOnLights()
	{
	}
	void RenderInterface::SetPenColor(const Vector4d& col)
	{
	}
	void RenderInterface::SetPenColor(const Vector3d& col)
	{
	}
	void RenderInterface::DrawCube(const Vector3d& size)
	{
	}
	void RenderInterface::DrawEllipsoid(const Vector3d& size)
	{
	}
	void RenderInterface::PushMatrix()
	{
	}
	void RenderInterface::PopMatrix()
	{
	}
	void RenderInterface::PushName(int id)
	{
	}
	void RenderInterface::PopName()
	{
	}
	void RenderInterface::Translate(const Vector3d& offset)
	{
	}
	void RenderInterface::Scale(const Vector3d& scale)
	{
	}
	void RenderInterface::Rotate(const Vector3d& axis, double rad)
	{
	}
	void RenderInterface::SaveToImage(const char * filename, DecoBufferType buffType)
	{

	}
	void RenderInterface::ReadFrameBuffer(DecoBufferType buffType, DecoColorChannel ch, void* pixels)
	{

	}
	void RenderInterface::SetMaterial(const Vector3d& diffuse, const Vector3d& specular, double cosinePow)
	{

	}
	void RenderInterface::GetMaterial(Vector3d& diffuse, Vector3d& specular, double& cosinePow) const
	{

	}
	void RenderInterface::SetDefaultMaterial()
	{

	}

}
