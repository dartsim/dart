#pragma once

#include <string>
#include <vector>
#include <memory>

using namespace std;

struct aiMaterial;
struct aiMesh;
struct aiNode;
struct aiScene;
    
struct Triangle
{
	double v1[3];
	double v2[3];
	double v3[3];
};

class Model3D
{

public:
	Model3D(void);
	~Model3D(void);

	bool loadModel(string fileName);
	void getTriangles(vector<Triangle> &triangles);
	void drawScene();

	const aiScene* getScene() {return scene;}

private:
	void getTrianglesForMesh(const aiMesh *mesh, vector<Triangle> &triangles, int startPosition);

	void recursiveRender(const struct aiScene *sc, const struct aiNode* nd);
	void applyMaterial(const struct aiMaterial *mtl);

	const aiScene* scene;

};

