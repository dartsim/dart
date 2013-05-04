/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/01/2013
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

#ifndef COLLISION_SIMPLE_H____
#define COLLISION_SIMPLE_H____

#include <vector>

#include "collision/CollisionDetector.h"
#include "collision/CollisionNode.h"
#include "collision/simple/LieGroup.h"

namespace collision
{

class ContactConstraint
{
public:
	void HowAboutThisPoint(Vec3&, Vec3&, double) {}
};

struct spContact
{
	Vec3 point;
	Vec3 normal;
	double penetrationDepth;
};

struct spResult
{
	std::vector<spContact> contacts;
};

class CollisionPair
{
public:
	CollisionNode* pLeftCol;
	CollisionNode* pRightCol;

	ContactConstraint *pContactConstraint;

	bool		PRESTEP_Find_NarrowPhase_Algorithm();
	bool		PRESTEP_Find_RoughCheck__Algorithm();

	bool		RoughCheck____________________________________MARK7();
	int			NarrowPhase_Algorithm_________________________MARK7();

protected:
	bool	(CollisionPair::*m_pfn_RoughCheck_Algoritm)();

	//	bool	_PlaneOthers________RoughCheck_TEMP();
	//	bool	_OthersPlane________RoughCheck_TEMP();
	//	bool	_OthersOthers_______RoughCheck_TEMP();


	// Under construction
	/*
	bool	_BoxBox_____________RoughCheck();
	bool	_BoxSphere__________RoughCheck();
	bool	_BoxCylinder________RoughCheck();
	bool	_BoxCapsule_________RoughCheck();

	bool	_SphereBox__________RoughCheck();
	bool	_SphereSphere_______RoughCheck();
	bool	_SphereCylinder_____RoughCheck();
	bool	_SphereCapsule______RoughCheck();

	bool	_CylinderBox________RoughCheck();
	bool	_CylinderSphere_____RoughCheck();
	bool	_CylinderCylinder___RoughCheck();
	bool	_CylinderCapsule____RoughCheck();

	bool	_CapsuleBox_________RoughCheck();
	bool	_CapsuleSphere______RoughCheck();
	bool	_CapsuleCylinder____RoughCheck();
	bool	_CapsuleCapsule_____RoughCheck();

	bool	_PlaneBox___________RoughCheck();
	bool	_PlaneSphere________RoughCheck();
	bool	_PlaneCylinder______RoughCheck();
	bool	_PlaneCapsule_______RoughCheck();

	bool	_BoxPlane___________RoughCheck();
	bool	_SpherePlane________RoughCheck();
	bool	_CylinderPlane______RoughCheck();
	bool	_CapsulePlane_______RoughCheck();
	*/



	int		(CollisionPair::*m_pfn_NarrowPhase_Algoritm)();
};


unsigned int collide(const CollisionNode* _collNode1,/* const Eigen::Matrix4d& _T1,*/
			const CollisionNode* _collNode2,/* const Eigen::Matrix4d& _T2,*/
			spResult& _result);

int		_BoxBox_____________MARK8(const Vec3& _size1, const SE3& _T1,
								  const Vec3& _size2, const SE3& _T2,
								  spResult& _result);
int		_BoxSphere__________MARK8(const Vec3& _size1, const SE3& _T1,
								  double _radius2, const SE3& _T2,
								  spResult& _result);
//	int		_BoxCylinder________MARK8();
//	int		_BoxCapsule_________MARK8();

int		_SphereBox__________MARK8(double _radius1, const SE3& _T1,
								  const Vec3& _size2, const SE3& _T2,
								  spResult& _result);
int		_SphereSphere_______MARK8(double _radius1, const SE3& _T1,
								  double _radius2, const SE3& _T2,
								  spResult& _result);
//	int		_SphereCylinder_____MARK8();
//	int		_SphereCapsule______MARK8();

//	int		_CylinderBox________MARK8();
//	int		_CylinderSphere_____MARK8();
//	int		_CylinderCylinder___MARK8();
//	int		_CylinderCapsule____MARK8();

//	int		_CapsuleBox_________MARK8();
//	int		_CapsuleSphere______MARK8();
//	int		_CapsuleCylinder____MARK8();
//	int		_CapsuleCapsule_____MARK8();

//	int		_PlaneBox___________MARK8();
//	int		_PlaneSphere________MARK8();
//	int		_PlaneCylinder______MARK8();
//	int		_PlaneCapsule_______MARK8();

//	int		_BoxPlane___________MARK8();
//	int		_SpherePlane________MARK8();
//	int		_CylinderPlane______MARK8();
//	int		_CapsulePlane_______MARK8();




}

#endif // COLLISION_SIMPLE_H

