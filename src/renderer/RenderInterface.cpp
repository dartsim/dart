#include "RenderInterface.h"

namespace renderer {
	void RenderInterface::initialize()
	{
	}
	void RenderInterface::destroy()
	{
	}

	void RenderInterface::setViewport(int X,int Y,int Width,int Height)
	{
	}
	void RenderInterface::getViewport(int& X, int& Y, int &Width, int& Height) const
	{
	}

	void RenderInterface::clear(const Vector3d& color)
	{
	}

	void RenderInterface::setDefaultLight()
	{
	}
	void RenderInterface::addLight(Light* light)
	{
		mLightList.push_back(light);
	}
	void RenderInterface::eraseAllLights()
	{
		mLightList.clear();
	}
	void RenderInterface::turnLightsOff()
	{
	}
	void RenderInterface::turnLightsOn()
	{
	}
	void RenderInterface::setPenColor(const Vector4d& col)
	{
	}
	void RenderInterface::setPenColor(const Vector3d& col)
	{
	}
	void RenderInterface::drawCube(const Vector3d& size)
	{
	}
	void RenderInterface::drawEllipsoid(const Vector3d& size)
	{
	}
	void RenderInterface::pushMatrix()
	{
	}
	void RenderInterface::popMatrix()
	{
	}
	void RenderInterface::pushName(int id)
	{
	}
	void RenderInterface::popName()
	{
	}
	void RenderInterface::translate(const Vector3d& offset)
	{
	}
	void RenderInterface::scale(const Vector3d& scale)
	{
	}
	void RenderInterface::rotate(const Vector3d& axis, double rad)
	{
	}
	void RenderInterface::saveToImage(const char * filename, DecoBufferType buffType)
	{

	}
	void RenderInterface::readFrameBuffer(DecoBufferType buffType, DecoColorChannel ch, void* pixels)
	{

	}
	void RenderInterface::setMaterial(const Vector3d& diffuse, const Vector3d& specular, double cosinePow)
	{

	}
	void RenderInterface::getMaterial(Vector3d& diffuse, Vector3d& specular, double& cosinePow) const
	{

	}
	void RenderInterface::setDefaultMaterial()
	{

	}

}
