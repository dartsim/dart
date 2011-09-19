/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jie (Jay) Tan <jtan34@cc.gatech.edu>
 * Date: 07/18/2011
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "RenderInterface.h"

using namespace Eigen;

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
