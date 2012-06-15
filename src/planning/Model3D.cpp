#include "Model3D.h"

#include <iostream>

#include "GL\glut.h"

// helper functions
void color4_to_float4(const struct aiColor4D *c, float f[4])
{
	f[0] = c->r;
	f[1] = c->g;
	f[2] = c->b;
	f[3] = c->a;
}

void set_float4(float f[4], float a, float b, float c, float d)
{
	f[0] = a;
	f[1] = b;
	f[2] = c;
	f[3] = d;
}

Model3D::Model3D(void) :
	scene(NULL)
{
}

Model3D::~Model3D(void)
{
}

bool Model3D::loadModel(string fileName)
{
	cout << "Loading file: " << fileName << endl;

	// clean up old scene
	if(scene)
	{
		importer.FreeScene();
	}

	// And have it read the given file with some example postprocessing
	// Usually - if speed is not the most important aspect for you - you'll 
	// propably to request more postprocessing than we do in this example.
	scene = importer.ReadFile(fileName, 
		aiProcess_CalcTangentSpace       | 
		aiProcess_Triangulate            |
		aiProcess_JoinIdenticalVertices  |
		aiProcess_SortByPType);
  
	// If the import failed, report it
	if(!scene)
	{
		// error in loading
		cout << "Error loading file." << endl;
		return false;
	}

	cout << "File loaded." << endl;
	return true;
}

void Model3D::getTriangles(vector<Triangle> &triangles)
{
	triangles.clear();

	// Now we can access the file's contents. 
	if(!scene->HasMeshes())
	{
		cout << "Scene has no meshes." << endl;
		return;
	}

	unsigned int numMeshes = scene->mNumMeshes;
	unsigned int numFaces = 0;
	unsigned int numVertices = 0;
	
	vector<int> startPositions;
	startPositions.resize(numMeshes);

	// calculate the number of faces, vertices, and the starting positions for each meshes faces
	for(unsigned int i = 0; i < numMeshes; i++)
	{	
		startPositions.at(i) = numFaces;

		aiMesh *mesh = scene->mMeshes[i];
		
		numFaces += mesh->mNumFaces;
		numVertices += mesh->mNumVertices;
	}
	
	triangles.resize(numFaces);

	for(unsigned int i = 0; i < numMeshes; i++)
	{
		getTrianglesForMesh(scene->mMeshes[i], triangles, startPositions.at(i));
	}
	
	cout << "Generated vector of " << triangles.size() << " triangles from " << numMeshes << " meshes." << endl;

}

void Model3D::getTrianglesForMesh(const aiMesh *mesh, vector<Triangle> &triangles, const int startPosition)
{
	for(unsigned int i = 0; i < mesh->mNumFaces; i++)
	{
		Triangle t;

		aiFace *face = &mesh->mFaces[i];

		for(unsigned int j = 0; j < 3; j++)
		{
			// get the index for the vertice from the face
			unsigned int index = face->mIndices[j];
			aiVector3D *v = &mesh->mVertices[index];
			
			// fill in the appropriate vertex in the triangle
			if(j == 0)
			{	
				t.v1[0] = v->x;
				t.v1[1] = v->y;
				t.v1[2] = v->z;
			}
			else if(j == 1)
			{
				t.v2[0] = v->x;
				t.v2[1] = v->y;
				t.v2[2] = v->z;
			}
			else if(j == 2)
			{
				t.v3[0] = v->x;
				t.v3[1] = v->y;
				t.v3[2] = v->z;
			}
		}

		triangles.at(startPosition + i) = t;
	}
}

void Model3D::drawScene()
{
	this->recursiveRender(this->scene, this->scene->mRootNode);
}

void Model3D::recursiveRender(const struct aiScene *sc, const struct aiNode* nd)
{
	unsigned int i;
	unsigned int n = 0, t;
	struct aiMatrix4x4 m = nd->mTransformation;

	// update transform
	aiTransposeMatrix4(&m);
	glPushMatrix();
	glMultMatrixf((float*)&m);

	// draw all meshes assigned to this node
	for (; n < nd->mNumMeshes; ++n) {
		const struct aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];

		applyMaterial(sc->mMaterials[mesh->mMaterialIndex]);

		if(mesh->mNormals == NULL) {
			glDisable(GL_LIGHTING);
		} else {
			glEnable(GL_LIGHTING);
		}

		for (t = 0; t < mesh->mNumFaces; ++t) {
			const struct aiFace* face = &mesh->mFaces[t];
			GLenum face_mode;

			switch(face->mNumIndices) {
				case 1: face_mode = GL_POINTS; break;
				case 2: face_mode = GL_LINES; break;
				case 3: face_mode = GL_TRIANGLES; break;
				default: face_mode = GL_POLYGON; break;
			}

			glBegin(face_mode);

			for(i = 0; i < face->mNumIndices; i++) {
				int index = face->mIndices[i];
				if(mesh->mColors[0] != NULL)
					glColor4fv((GLfloat*)&mesh->mColors[0][index]);
				if(mesh->mNormals != NULL) 
					glNormal3fv(&mesh->mNormals[index].x);
				glVertex3fv(&mesh->mVertices[index].x);
			}

			glEnd();
		}

	}

	// draw all children
	for (n = 0; n < nd->mNumChildren; ++n) {
		recursiveRender(sc, nd->mChildren[n]);
	}

	glPopMatrix();
}

void Model3D::applyMaterial(const struct aiMaterial *mtl)
{
	float c[4];

	GLenum fill_mode;
	int ret1, ret2;
	struct aiColor4D diffuse;
	struct aiColor4D specular;
	struct aiColor4D ambient;
	struct aiColor4D emission;
	float shininess, strength;
	int two_sided;
	int wireframe;
	unsigned int max;

	set_float4(c, 0.8f, 0.8f, 0.8f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
		color4_to_float4(&diffuse, c);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, c);

	set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular))
		color4_to_float4(&specular, c);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);

	set_float4(c, 0.2f, 0.2f, 0.2f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient))
		color4_to_float4(&ambient, c);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, c);

	set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
	if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission))
		color4_to_float4(&emission, c);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, c);

	max = 1;
	ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
	if(ret1 == AI_SUCCESS) {
		max = 1;
		ret2 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
		if(ret2 == AI_SUCCESS)
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess * strength);
		else
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
	}
	else {
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
		set_float4(c, 0.0f, 0.0f, 0.0f, 0.0f);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);
	}

	max = 1;
	if(AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_ENABLE_WIREFRAME, &wireframe, &max))
		fill_mode = wireframe ? GL_LINE : GL_FILL;
	else
		fill_mode = GL_FILL;
	glPolygonMode(GL_FRONT_AND_BACK, fill_mode);

	max = 1;
	if((AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_TWOSIDED, &two_sided, &max)) && two_sided)
		glDisable(GL_CULL_FACE);
	else 
		glEnable(GL_CULL_FACE);
}