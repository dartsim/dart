/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author     Jie Tan
  Date      07/18/2011
*/

#include "RenderInterface.h"

namespace renderer {
	void RenderInterface::initialize()
	{
	}

	void RenderInterface::destroy()
	{
	}

	void RenderInterface::setViewport(int _x,int _y,int _width,int _height)
	{
	}

	void RenderInterface::getViewport(int& _x, int& _y,int& _width,int& _height) const
	{
	}

	void RenderInterface::clear(const Vector3d& _color)
	{
	}

	void RenderInterface::setDefaultLight()
	{
	}

	void RenderInterface::addLight(Light *_light)
	{
		mLightList.push_back(_light);
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

	void RenderInterface::setMaterial(const Vector3d& _diffuse, const Vector3d& _specular, double _cosinePow)
	{

	}

	void RenderInterface::getMaterial(Vector3d& _diffuse, Vector3d& _specular, double& cosinePow) const
	{

	}

	void RenderInterface::setDefaultMaterial()
	{

	}

	void RenderInterface::pushMatrix()
	{
	}

	void RenderInterface::popMatrix()
	{
	}

	void RenderInterface::pushName(int _id)
	{
	}

	void RenderInterface::popName()
	{
	}

	void RenderInterface::translate(const Vector3d& _offset)
	{
	}

	void RenderInterface::rotate(const Vector3d& _axis, double _rad)
	{
	}

	void RenderInterface::scale(const Vector3d& _scale)
	{
	}

	void RenderInterface::drawEllipsoid(const Vector3d& _size)
	{
	}

	void RenderInterface::drawCube(const Vector3d& _size)
	{
	}

	void RenderInterface::setPenColor(const Vector4d& _col)
	{
	}

	void RenderInterface::setPenColor(const Vector3d& _col)
	{
	}


	void RenderInterface::saveToImage(const char *_filename, DecoBufferType _buffType)
	{

	}

	void RenderInterface::readFrameBuffer(DecoBufferType _buffType, DecoColorChannel _ch, void *_pixels)
	{

	}

	Camera*  RenderInterface::getCamera(){
		return mCamera;
	}

} // namespace renderer
