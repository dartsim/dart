#ifndef _COLLISION_SHAPES_H_
#define _COLLISION_SHAPES_H_


#include "collision.h"



namespace collision_checking
{
	template<class BV>
	BVHModel<BV>* createCube(float sizeX, float sizeY, float sizeZ)
	{
		float n[6][3] =
		{
			{-1.0, 0.0, 0.0},
			{0.0, 1.0, 0.0},
			{1.0, 0.0, 0.0},
			{0.0, -1.0, 0.0},
			{0.0, 0.0, 1.0},
			{0.0, 0.0, -1.0}
		};
		int faces[6][4] =
		{
			{0, 1, 2, 3},
			{3, 2, 6, 7},
			{7, 6, 5, 4},
			{4, 5, 1, 0},
			{5, 6, 2, 1},
			{7, 4, 0, 3}
		};
		float v[8][3];
		int i;

		v[0][0] = v[1][0] = v[2][0] = v[3][0] = -sizeX / 2;
		v[4][0] = v[5][0] = v[6][0] = v[7][0] = sizeX / 2;
		v[0][1] = v[1][1] = v[4][1] = v[5][1] = -sizeY / 2;
		v[2][1] = v[3][1] = v[6][1] = v[7][1] = sizeY / 2;
		v[0][2] = v[3][2] = v[4][2] = v[7][2] = -sizeZ / 2;
		v[1][2] = v[2][2] = v[5][2] = v[6][2] = sizeZ / 2;

		BVHModel<BV>* model = new BVHModel<BV>;
		Vec3f p1,p2, p3;
		model->beginModel();

		for(int i=0;i<6;i++)
		{
			p1 = Vec3f(v[faces[i][0]][0], v[faces[i][0]][1], v[faces[i][0]][2]);
			p2 = Vec3f(v[faces[i][1]][0], v[faces[i][1]][1], v[faces[i][1]][2]);
			p3 = Vec3f(v[faces[i][2]][0], v[faces[i][2]][1], v[faces[i][2]][2]);
			model->addTriangle(p1, p2, p3);
			p1 = Vec3f(v[faces[i][0]][0], v[faces[i][0]][1], v[faces[i][0]][2]);
			p2 = Vec3f(v[faces[i][2]][0], v[faces[i][2]][1], v[faces[i][2]][2]);
			p3 = Vec3f(v[faces[i][3]][0], v[faces[i][3]][1], v[faces[i][3]][2]);
			model->addTriangle(p1, p2, p3);
		}
		model->endModel();
		return model;
	}

	//template<class BV>

 }

#endif