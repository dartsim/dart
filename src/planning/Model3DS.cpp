//////////////////////////////////////////////////////////////////////
//
// 3D Studio Model Class
// by: Matthew Fairfax
//
// Model3DS.cpp: implementation of the Model3DS class.
// This is a simple class for loading and viewing
// 3D Studio model files (.3ds). It supports models
// with multiple objects. It also supports multiple
// textures per object. It does not support the animation
// for 3D Studio models b/c there are simply too many
// ways for an artist to animate a 3D Studio model and
// I didn't want to impose huge limitations on the artists.
// However, I have imposed a limitation on how the models are
// textured:
// 1) Every faces must be assigned a material
// 2) If you want the face to be textured assign the
//    texture to the Diffuse Color map
// 3) The texture must be supported by the GLTexture class
//    which only supports bitmap and targa right now
// 4) The texture must be located in the same directory as
//    the model
//
// Support for non-textured faces is done by reading the color
// from the material's diffuse color.
//
// Some models have problems loading even if you follow all of
// the restrictions I have stated and I don't know why. If you
// can import the 3D Studio file into Milkshape 3D
// (http://www.swissquake.ch/chumbalum-soft) and then export it
// to a new 3D Studio file. This seems to fix many of the problems
// but there is a limit on the number of faces and vertices Milkshape 3D
// can read.
//
// Usage:
// Model3DS m;
//
// m.Load("model.3ds"); // Load the model
// m.Draw();			// Renders the model to the screen
//
// // If you want to show the model's normals
// m.shownormals = true;
//
// // If the model is not going to be lit then set the lit
// // variable to false. It defaults to true.
// m.lit = false;
//
// // You can disable the rendering of the model
// m.visible = false;
//
// // You can move and rotate the model like this:
// m.rot.x = 90.0f;
// m.rot.y = 30.0f;
// m.rot.z = 0.0f;
//
// m.pos.x = 10.0f;
// m.pos.y = 0.0f;
// m.pos.z = 0.0f;
//
// // If you want to move or rotate individual objects
// m.Objects[0].rot.x = 90.0f;
// m.Objects[0].rot.y = 30.0f;
// m.Objects[0].rot.z = 0.0f;
//
// m.Objects[0].pos.x = 10.0f;
// m.Objects[0].pos.y = 0.0f;
// m.Objects[0].pos.z = 0.0f;
//
//////////////////////////////////////////////////////////////////////

//#include "Transform.h"
#include "Model3DS.h"
#include <iostream>
#include <math.h>
#include <string>

#define SAFE_DELETE(p) { if(p) { delete (p); (p)=NULL; } }
#define SAFE_DELETE_ARRAY(p) { if(p) { delete[] (p); (p)=NULL; } }

using namespace std;
using namespace Eigen;

// This is used to generate a warning from the compiler
#define _QUOTE(x) # x
#define QUOTE(x) _QUOTE(x)
#define __FILE__LINE__ __FILE__ "(" QUOTE(__LINE__) ") : "
#define warn( x )  message( __FILE__LINE__ #x "\n" )

// The chunk's id numbers
#define MAIN3DS				0x4D4D
 #define MAIN_VERS			0x0002
 #define EDIT3DS			0x3D3D
  #define MESH_VERS			0x3D3E
  #define OBJECT			0x4000
   #define TRIG_MESH		0x4100
    #define VERT_LIST		0x4110
    #define FACE_DESC		0x4120
     #define FACE_MAT		0x4130
    #define TEX_VERTS		0x4140
     #define SMOOTH_GROUP	0x4150
    #define LOCAL_COORDS	0x4160
  #define MATERIAL			0xAFFF
   #define MAT_NAME			0xA000
   #define MAT_AMBIENT		0xA010
   #define MAT_DIFFUSE		0xA020
   #define MAT_SPECULAR		0xA030
   #define SHINY_PERC		0xA040
   #define SHINY_STR_PERC	0xA041
   #define TRANS_PERC		0xA050
   #define TRANS_FOFF_PERC	0xA052
   #define REF_BLUR_PERC	0xA053
   #define RENDER_TYPE		0xA100
   #define SELF_ILLUM		0xA084
   #define MAT_SELF_ILPCT	0xA08A
   #define WIRE_THICKNESS	0xA087
   #define MAT_TEXMAP		0xA200
    #define MAT_MAPNAME		0xA300
  #define ONE_UNIT			0x0100
 #define KEYF3DS			0xB000
  #define FRAMES			0xB008
  #define MESH_INFO			0xB002
   #define HIER_POS			0xB030
   #define HIER_FATHER		0xB010
   #define PIVOT_PT			0xB013
   #define TRACK00			0xB020
   #define TRACK01			0xB021
   #define TRACK02			0xB022
#define	COLOR_RGB			0x0010
#define COLOR_TRU			0x0011
#define COLOR_TRUG			0x0012
#define COLOR_RGBG			0x0013
#define PERC_INT			0x0030
#define PERC_double			0x0031

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

extern bool individualCOM;
extern bool treeCOMS;
extern bool robotFloorCOM;
extern bool showPrimitive;

Model3DS::Model3DS()
{
	// Initialization
	Materials = 0;
	Objects = 0;

	// Don't show the normals by default
	shownormals = false;

	// The model is lit by default
	lit = true;

	// The model is visible by default
	visible = true;

	// Set up the default position
	pos.x = 0.0;
	pos.y = 0.0;
	pos.z = 0.0;
	// Set up the default rotation
	rot.x = 0.0;
	rot.y = 0.0;
	rot.z = 0.0;

	// Set up the path
	//path = new char[ PATH_LENGTH ];

	// Zero out our counters for MFC
	numObjects = 0;
	numMaterials = 0;

	// Set the scale to one
	scale = 1.0f;
}

Model3DS::~Model3DS()
{
	cout << "DELETING Model: " << name << endl;
	//SAFE_DELETE_ARRAY( path );
	SAFE_DELETE_ARRAY( Materials );
	for (int k = 0; k < numObjects; k++) {
		SAFE_DELETE_ARRAY( Objects[k].TexCoords );
		SAFE_DELETE_ARRAY( Objects[k].Vertexes );
		SAFE_DELETE_ARRAY( Objects[k].Normals );
		SAFE_DELETE_ARRAY( Objects[k].Faces );
		for( int l=0; l<Objects[k].numMatFaces; l++ )
			SAFE_DELETE_ARRAY( Objects[k].MatFaces[l].subFaces );
		SAFE_DELETE_ARRAY( Objects[k].MatFaces );
	}
	SAFE_DELETE_ARRAY( Objects );
	glDeleteLists(modelDL,1);
//	while(ret == GL_INVALID_VALUE || ret == GL_INVALID_OPERATION)
//		glDeleteLists(modelDL,1);
}


void Model3DS::Load(string fullpath)
{
	// holds the main chunk header
	ChunkHeader main;

	path = fullpath.substr( 0, fullpath.rfind("/")+1 );
	name = fullpath.substr(fullpath.rfind("/")+1, fullpath.size());

	// Load the file

#ifdef WIN32
	fopen_s(&bin3ds,fullpath.c_str(),"rb");
#else
	bin3ds = fopen(fullpath.c_str(), "rb");							// Open the TGA file
#endif

	if (bin3ds==NULL) {
		std::cout << "Error: no model file found at '" << fullpath << "'" << std::endl;
		// exit nicely, or let it segfault below...
	}

	// Make sure we are at the beginning
	fseek(bin3ds, 0, SEEK_SET);

	// Load the Main Chunk's header
	fread(&main.id,sizeof(main.id),1,bin3ds);
    fread(&main.len,sizeof(main.len),1,bin3ds);

	// Start Processing
	MainChunkProcessor(main.len, ftell(bin3ds));

	// Don't need the file anymore so close it
	fclose(bin3ds);
	bin3ds = NULL;

	// Calculate the vertex normals
	CalculateNormals();

	// For future reference
	//modelname = name;
	modelname = name;


	// Find the total number of faces and vertices
	totalFaces = 0;
	totalVerts = 0;

	for (int i = 0; i < numObjects; i ++)
	{
		totalFaces += Objects[i].numFaces/3;
		totalVerts += Objects[i].numVerts;
	}



	// If the object doesn't have any texcoords generate some
	for (int k = 0; k < numObjects; k++)
	{
		if (Objects[k].numTexCoords == 0)
		{
			//cout << "K " << k << " VERTS" << Objects[k].numVerts << endl;
			// Set the number of texture coords
			Objects[k].numTexCoords = Objects[k].numVerts;


			// Allocate an array to hold the texture coordinates
			Objects[k].TexCoords = new GLdouble[Objects[k].numTexCoords * 2];

			// Make some texture coords
			for (int m = 0; m < Objects[k].numTexCoords; m++)
			{
				Objects[k].TexCoords[2*m] = Objects[k].Vertexes[3*m];
				Objects[k].TexCoords[2*m+1] = Objects[k].Vertexes[3*m+1];
			}
		}
	}

	// Let's build simple colored textures for the materials w/o a texture
	for (int j = 0; j < numMaterials; j++)
	{
		if (Materials[j].textured == false)
		{
			unsigned char r = Materials[j].color.r;
			unsigned char g = Materials[j].color.g;
			unsigned char b = Materials[j].color.b;
			Materials[j].tex.BuildColorTexture(r, g, b);
			Materials[j].textured = true;
		}
	}

	modelDL = glGenLists(1);
	glNewList(modelDL,GL_COMPILE);
	glDisableClientState( GL_VERTEX_ARRAY );
	glDisableClientState( GL_INDEX_ARRAY );
	glDisableClientState( GL_COLOR_ARRAY );
	glDisableClientState( GL_TEXTURE_COORD_ARRAY );
	glDisableClientState( GL_EDGE_FLAG_ARRAY );
	glDisableClientState( GL_NORMAL_ARRAY );
	Draw();
	glEnd();
	glEndList();
}

void Model3DS::ReportTriangles(vector<Triangle> *trigs)
{
	/* // For testing Collision Model
	colDL = glGenLists(1);
	glNewList(colDL,GL_COMPILE);
	glPushMatrix();*/

	for (int i = 0; i < numObjects; i++){ //numObjects
		Eigen::Transform<double, 3, Eigen::Affine> obTrans;

		Matrix3d rotMat;
		rotMat = AngleAxisd(Objects[i].rot.z, Vector3d::UnitZ())
				  * AngleAxisd(Objects[i].rot.y, Vector3d::UnitY())
				  * AngleAxisd(Objects[i].rot.x, Vector3d::UnitX());
		Vector3d transVec = Vector3d(Objects[i].pos.x,Objects[i].pos.y,Objects[i].pos.z);

		obTrans = rotMat;
		obTrans.translation() = transVec;

		for (int k = 0; k < Objects[i].numMatFaces; k++){
			for(int j=0; j<Objects[i].MatFaces[k].numSubFaces; j+=3){

				unsigned short f1 = Objects[i].MatFaces[k].subFaces[j]*3;
				unsigned short f2 = Objects[i].MatFaces[k].subFaces[j+1]*3;
				unsigned short f3 = Objects[i].MatFaces[k].subFaces[j+2]*3;

				Triangle tr;

				//For Transform
				Vector3d p1 = Vector3d(Objects[i].Vertexes[f1],Objects[i].Vertexes[f1+1],Objects[i].Vertexes[f1+2]);
				Vector3d p2 = Vector3d(Objects[i].Vertexes[f2],Objects[i].Vertexes[f2+1],Objects[i].Vertexes[f2+2]);
				Vector3d p3 = Vector3d(Objects[i].Vertexes[f3],Objects[i].Vertexes[f3+1],Objects[i].Vertexes[f3+2]);

				p1 = obTrans*p1;
				p2 = obTrans*p2;
				p3 = obTrans*p3;

				tr.v1[0] = p1[0]; tr.v1[1] = p1[1]; tr.v1[2] = p1[2];
				tr.v2[0] = p2[0]; tr.v2[1] = p2[1]; tr.v2[2] = p2[2];
				tr.v3[0] = p3[0]; tr.v3[1] = p3[1]; tr.v3[2] = p3[2];

				trigs->push_back(tr);

				/* //For testing collision models
				glBegin(GL_TRIANGLES);
				glVertex3d(tr.v1[0], tr.v1[1], tr.v1[2]);
				glVertex3d(tr.v2[0], tr.v2[1], tr.v2[2]);
				glVertex3d(tr.v3[0], tr.v3[1], tr.v3[2]);
				glEnd();*/
			
			}
		}
	}
	// For testing collision model
	//glPopMatrix();
	//glEndList();
}

void Model3DS::Draw()
{
	if (visible)
	{
	glPushMatrix();

		// Move the model
		/*glTranslated(pos.x, pos.y, pos.z);

		// Rotate the model
		glRotated(rot.x, 1.0f, 0.0f, 0.0f);
		glRotated(rot.y, 0.0f, 1.0f, 0.0f);
		glRotated(rot.z, 0.0f, 0.0f, 1.0f);

		glScalef(scale, scale, scale);*/

		// Loop through the objects
		for (int i = 0; i < numObjects; i++)
		{
			//glEnable(GL_TEXTURE_2D);
			// Enable texture coordiantes, normals, and vertices arrays
			if (Objects[i].textured)
				glEnableClientState(GL_TEXTURE_COORD_ARRAY);
			if (lit)
				glEnableClientState(GL_NORMAL_ARRAY);
			glEnableClientState(GL_VERTEX_ARRAY);

			// Point them to the objects arrays
			if (Objects[i].textured)
				glTexCoordPointer(2, GL_DOUBLE, 0, Objects[i].TexCoords);
			if (lit)
				glNormalPointer(GL_DOUBLE, 0, Objects[i].Normals);
			glVertexPointer(3, GL_DOUBLE, 0, Objects[i].Vertexes);

			// Loop through the faces as sorted by material and draw them
			for (int j = 0; j < Objects[i].numMatFaces; j++)
			{
				// Use the material's texture
				Materials[Objects[i].MatFaces[j].MatIndex].tex.Use();

				glPushMatrix();

					// Move the model
					glTranslated(Objects[i].pos.x, Objects[i].pos.y, Objects[i].pos.z);

					// Rotate the model
					//glRotated(Objects[i].rot.x, 1.0f, 0.0f, 0.0f);
					//glRotated(Objects[i].rot.y, 0.0f, 1.0f, 0.0f);
					//glRotated(Objects[i].rot.z, 0.0f, 0.0f, 1.0f);

					glRotated(Objects[i].rot.z, 0.0f, 0.0f, 1.0f);
					glRotated(Objects[i].rot.y, 0.0f, 1.0f, 0.0f);
					glRotated(Objects[i].rot.x, 1.0f, 0.0f, 0.0f);

					// Draw the faces using an index to the vertex array

					//cout << "FACES: " << Objects[i].MatFaces[j].numSubFaces << endl;
					/*for(int q=0; q<Objects[i].MatFaces[j].numSubFaces; q++){
						cout << " : " << Objects[i].MatFaces[j].subFaces[q];
					}
					cout << endl;*/

					glDrawElements(GL_TRIANGLES, Objects[i].MatFaces[j].numSubFaces, GL_UNSIGNED_SHORT, Objects[i].MatFaces[j].subFaces);

				glPopMatrix();
			}

			// Show the normals?
			if (shownormals)
			{
				// Loop through the vertices and normals and draw the normal
				for (int k = 0; k < Objects[i].numVerts * 3; k += 3)
				{
					// Disable texturing
					//glDisable(GL_TEXTURE_2D);
					// Disbale lighting if the model is lit
					if (lit)
						glDisable(GL_LIGHTING);
					// Draw the normals blue
					glColor3f(0.0f, 0.0f, 1.0f);

					// Draw a line between the vertex and the end of the normal
					glBegin(GL_LINES);
						glVertex3d(Objects[i].Vertexes[k], Objects[i].Vertexes[k+1], Objects[i].Vertexes[k+2]);
						glVertex3d(Objects[i].Vertexes[k]+Objects[i].Normals[k], Objects[i].Vertexes[k+1]+Objects[i].Normals[k+1], Objects[i].Vertexes[k+2]+Objects[i].Normals[k+2]);
					glEnd();

					// Reset the color to white
					glColor3f(1.0f, 1.0f, 1.0f);
					// If the model is lit then renable lighting
					if (lit)
						glEnable(GL_LIGHTING);
				}
			}
		}

	glPopMatrix();
	}
}

void Model3DS::CalculateNormals()
{
	// Let's build some normals
	for (int i = 0; i < numObjects; i++)
	{
		for (int g = 0; g < Objects[i].numVerts; g++)
		{
			// Reduce each vert's normal to unit
			double length;
			Vector unit;

			unit.x = Objects[i].Normals[g*3];
			unit.y = Objects[i].Normals[g*3+1];
			unit.z = Objects[i].Normals[g*3+2];

			length = (double)sqrt((unit.x*unit.x) + (unit.y*unit.y) + (unit.z*unit.z));

			if (length == 0.0f)
				length = 1.0f;

			unit.x /= length;
			unit.y /= length;
			unit.z /= length;

			Objects[i].Normals[g*3]   = unit.x;
			Objects[i].Normals[g*3+1] = unit.y;
			Objects[i].Normals[g*3+2] = unit.z;
		}
	}
}

void Model3DS::MainChunkProcessor(int length, int findex)
{
	ChunkHeader h;

	// move the file pointer to the beginning of the main
	// chunk's data findex + the size of the header
	fseek(bin3ds, findex, SEEK_SET);

	while (ftell(bin3ds) < (findex + length - 6))
	{
		fread(&h.id,sizeof(h.id),1,bin3ds);
		fread(&h.len,sizeof(h.len),1,bin3ds);

		//cerr << ftell(bin3ds) << endl;
		switch (h.id)
		{
			// This is the mesh information like vertices, faces, and materials
			case EDIT3DS	:
				EditChunkProcessor(h.len, ftell(bin3ds));
				break;
			// I left this in case anyone gets very ambitious
			case KEYF3DS	:
				//KeyFrameChunkProcessor(h.len, ftell(bin3ds));
				break;
			default			:
				break;
		}

		fseek(bin3ds, (h.len - 6), SEEK_CUR);
	}

	// move the file pointer back to where we got it so
	// that the ProcessChunk() which we interrupted will read
	// from the right place
	fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::EditChunkProcessor(int length, int findex)
{
	ChunkHeader h;

	// move the file pointer to the beginning of the main
	// chunk's data findex + the size of the header
	fseek(bin3ds, findex, SEEK_SET);

	// First count the number of Objects and Materials
	while (ftell(bin3ds) < (findex + length - 6))
	{
		fread(&h.id,sizeof(h.id),1,bin3ds);
		fread(&h.len,sizeof(h.len),1,bin3ds);

		switch (h.id)
		{
			case OBJECT	:
				numObjects++;
				break;
			case MATERIAL	:
				numMaterials++;
				break;
			default			:
				break;
		}

		fseek(bin3ds, (h.len - 6), SEEK_CUR);
	}

	// Now load the materials
	if (numMaterials > 0)
	{
		Materials = new Material[numMaterials];

		// Material is set to untextured until we find otherwise
		for (int d = 0; d < numMaterials; d++)
			Materials[d].textured = false;

		fseek(bin3ds, findex, SEEK_SET);

		int i = 0;

		while (ftell(bin3ds) < (findex + length - 6))
		{
			fread(&h.id,sizeof(h.id),1,bin3ds);
			fread(&h.len,sizeof(h.len),1,bin3ds);

			switch (h.id)
			{
				case MATERIAL	:
					MaterialChunkProcessor(h.len, ftell(bin3ds), i);
					i++;
					break;
				default			:
					break;
			}

			fseek(bin3ds, (h.len - 6), SEEK_CUR);
		}
	}

	// Load the Objects (individual meshes in the whole model)
	if (numObjects > 0)
	{
		Objects = new Object[numObjects];

		// Set the textured variable to false until we find a texture
		for (int k = 0; k < numObjects; k++)
			Objects[k].textured = false;

		// Zero the objects position and rotation
		for (int m = 0; m < numObjects; m++)
		{
			Objects[m].pos.x = 0.0;
			Objects[m].pos.y = 0.0;
			Objects[m].pos.z = 0.0;

			Objects[m].rot.x = 0.0;
			Objects[m].rot.y = 0.0;
			Objects[m].rot.z = 0.0;
		}

		// Zero out the number of texture coords
		for (int n = 0; n < numObjects; n++)
			Objects[n].numTexCoords = 0;

		fseek(bin3ds, findex, SEEK_SET);

		int j = 0;

		while (ftell(bin3ds) < (findex + length - 6))
		{
			fread(&h.id,sizeof(h.id),1,bin3ds);
			fread(&h.len,sizeof(h.len),1,bin3ds);

			switch (h.id)
			{
				case OBJECT	:
					ObjectChunkProcessor(h.len, ftell(bin3ds), j);
					j++;
					break;
				default			:
					break;
			}

			fseek(bin3ds, (h.len - 6), SEEK_CUR);
		}
	}

	// move the file pointer back to where we got it so
	// that the ProcessChunk() which we interrupted will read
	// from the right place
	fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::MaterialChunkProcessor(int length, int findex, int matindex)
{
	ChunkHeader h;

	// move the file pointer to the beginning of the main
	// chunk's data findex + the size of the header
	fseek(bin3ds, findex, SEEK_SET);

	while (ftell(bin3ds) < (findex + length - 6))
	{
		fread(&h.id,sizeof(h.id),1,bin3ds);
		fread(&h.len,sizeof(h.len),1,bin3ds);

		switch (h.id)
		{
			case MAT_NAME	:
				// Loads the material's names
				MaterialNameChunkProcessor(h.len, ftell(bin3ds), matindex);
				break;
			case MAT_AMBIENT	:
				//ColorChunkProcessor(h.len, ftell(bin3ds));
				break;
			case MAT_DIFFUSE	:
				DiffuseColorChunkProcessor(h.len, ftell(bin3ds), matindex);
				break;
			case MAT_SPECULAR	:
				//ColorChunkProcessor(h.len, ftell(bin3ds));
			case MAT_TEXMAP	:
				// Finds the names of the textures of the material and loads them
				TextureMapChunkProcessor(h.len, ftell(bin3ds), matindex);
				break;
			default			:
				break;
		}

		fseek(bin3ds, (h.len - 6), SEEK_CUR);
	}

	// move the file pointer back to where we got it so
	// that the ProcessChunk() which we interrupted will read
	// from the right place
	fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::MaterialNameChunkProcessor(int length, int findex, int matindex)
{
	// move the file pointer to the beginning of the main
	// chunk's data findex + the size of the header
	fseek(bin3ds, findex, SEEK_SET);

	// Read the material's name
	//bin3ds >> Materials[matindex].name;
	Materials[matindex].name.clear();

	length = PATH_LENGTH; // Remove Warnings
	for (int i = 0; i < PATH_LENGTH; i++)
	{
		Materials[matindex].name.push_back(fgetc(bin3ds));
		if (Materials[matindex].name[i] == 0)
		{
			Materials[matindex].name[i] = NULL;
			break;
		}
	}

	// move the file pointer back to where we got it so
	// that the ProcessChunk() which we interrupted will read
	// from the right place
	fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::DiffuseColorChunkProcessor(int length, int findex, int matindex)
{
	ChunkHeader h;

	// move the file pointer to the beginning of the main
	// chunk's data findex + the size of the header
	fseek(bin3ds, findex, SEEK_SET);

	while (ftell(bin3ds) < (findex + length - 6))
	{
		fread(&h.id,sizeof(h.id),1,bin3ds);
		fread(&h.len,sizeof(h.len),1,bin3ds);

		// Determine the format of the color and load it
		switch (h.id)
		{
			case COLOR_RGB	:
				// A rgb double color chunk
				doubleColorChunkProcessor(h.len, ftell(bin3ds), matindex);
				break;
			case COLOR_TRU	:
				// A rgb int color chunk
				IntColorChunkProcessor(h.len, ftell(bin3ds), matindex);
				break;
			case COLOR_RGBG	:
				// A rgb gamma corrected double color chunk
				doubleColorChunkProcessor(h.len, ftell(bin3ds), matindex);
				break;
			case COLOR_TRUG	:
				// A rgb gamma corrected int color chunk
				IntColorChunkProcessor(h.len, ftell(bin3ds), matindex);
				break;
			default			:
				break;
		}

		fseek(bin3ds, (h.len - 6), SEEK_CUR);
	}

	// move the file pointer back to where we got it so
	// that the ProcessChunk() which we interrupted will read
	// from the right place
	fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::doubleColorChunkProcessor(int length, int findex, int matindex)
{
	double r;
	double g;
	double b;

	length = PATH_LENGTH; // Remove warnings
	// move the file pointer to the beginning of the main
	// chunk's data findex + the size of the header
	fseek(bin3ds, findex, SEEK_SET);

	fread(&r,sizeof(r),1,bin3ds);
	fread(&g,sizeof(g),1,bin3ds);
	fread(&b,sizeof(b),1,bin3ds);

	Materials[matindex].color.r = (unsigned char)(r*255.0f);
	Materials[matindex].color.g = (unsigned char)(g*255.0f);
	Materials[matindex].color.b = (unsigned char)(b*255.0f);
	Materials[matindex].color.a = 255;

	// move the file pointer back to where we got it so
	// that the ProcessChunk() which we interrupted will read
	// from the right place
	fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::IntColorChunkProcessor(int length, int findex, int matindex)
{
	unsigned char r;
	unsigned char g;
	unsigned char b;

	// move the file pointer to the beginning of the main
	// chunk's data findex + the size of the header
	fseek(bin3ds, findex, SEEK_SET);

	fread(&r,sizeof(r),1,bin3ds);
	fread(&g,sizeof(g),1,bin3ds);
	fread(&b,sizeof(b),1,bin3ds);

	Materials[matindex].color.r = r;
	Materials[matindex].color.g = g;
	Materials[matindex].color.b = b;
	Materials[matindex].color.a = 255;

	length = PATH_LENGTH; // Remove Warnings
	// move the file pointer back to where we got it so
	// that the ProcessChunk() which we interrupted will read
	// from the right place
	fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::TextureMapChunkProcessor(int length, int findex, int matindex)
{
	ChunkHeader h;

	// move the file pointer to the beginning of the main
	// chunk's data findex + the size of the header
	fseek(bin3ds, findex, SEEK_SET);

	while (ftell(bin3ds) < (findex + length - 6))
	{
		fread(&h.id,sizeof(h.id),1,bin3ds);
		fread(&h.len,sizeof(h.len),1,bin3ds);

		switch (h.id)
		{
			case MAT_MAPNAME:
				// Read the name of texture in the Diffuse Color map
				MapNameChunkProcessor(h.len, ftell(bin3ds), matindex);
				break;
			default:
				break;
		}

		fseek(bin3ds, (h.len - 6), SEEK_CUR);
	}

	// move the file pointer back to where we got it so
	// that the ProcessChunk() which we interrupted will read
	// from the right place
	fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::MapNameChunkProcessor(int length, int findex, int matindex)
{
	string name;

	// move the file pointer to the beginning of the main
	// chunk's data findex + the size of the header
	fseek(bin3ds, findex, SEEK_SET);

	// Read the name of the texture
	name.clear();
	length = PATH_LENGTH; // Remove Warnings
	for (int i = 0; i < PATH_LENGTH; i++)
	{
		name.push_back(fgetc(bin3ds));
		if (name[i] == 0)
		{
			name[i] = NULL;
			break;
		}
	}

	// Load the name and indicate that the material has a texture
	//char fullname[PATH_LENGTH];
	string fullname = path + name;
    //	sprintf_s(fullname,PATH_LENGTH, "%s%s", path, name);
	Materials[matindex].tex.Load(fullname.c_str());
	Materials[matindex].textured = true;


	// move the file pointer back to where we got it so
	// that the ProcessChunk() which we interrupted will read
	// from the right place
	fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::ObjectChunkProcessor(int length, int findex, int objindex)
{
	ChunkHeader h;

	// move the file pointer to the beginning of the main
	// chunk's data findex + the size of the header
	fseek(bin3ds, findex, SEEK_SET);

	// Load the object's name
	Objects[objindex].name.clear();
	for (int i = 0; i < PATH_LENGTH; i++)
	{
		Objects[objindex].name.push_back(fgetc(bin3ds));
		if (Objects[objindex].name[i] == 0)
		{
			Objects[objindex].name[i] = NULL;
			break;
		}
	}

	while (ftell(bin3ds) < (findex + length - 6))
	{
		fread(&h.id,sizeof(h.id),1,bin3ds);
		fread(&h.len,sizeof(h.len),1,bin3ds);

		switch (h.id)
		{
			case TRIG_MESH	:
				// Process the triangles of the object
				TriangularMeshChunkProcessor(h.len, ftell(bin3ds), objindex);
				break;
			default			:
				break;
		}

		fseek(bin3ds, (h.len - 6), SEEK_CUR);
	}

	// move the file pointer back to where we got it so
	// that the ProcessChunk() which we interrupted will read
	// from the right place
	fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::TriangularMeshChunkProcessor(int length, int findex, int objindex)
{
	ChunkHeader h;

	// move the file pointer to the beginning of the main
	// chunk's data findex + the size of the header
	fseek(bin3ds, findex, SEEK_SET);

	while (ftell(bin3ds) < (findex + length - 6))
	{
		fread(&h.id,sizeof(h.id),1,bin3ds);
		fread(&h.len,sizeof(h.len),1,bin3ds);

		switch (h.id)
		{
			case VERT_LIST	:
				// Load the vertices of the onject
				VertexListChunkProcessor(h.len, ftell(bin3ds), objindex);
				break;
			case LOCAL_COORDS	:
				//LocalCoordinatesChunkProcessor(h.len, ftell(bin3ds));
				break;
			case TEX_VERTS	:
				// Load the texture coordinates for the vertices
				TexCoordsChunkProcessor(h.len, ftell(bin3ds), objindex);
				Objects[objindex].textured = true;
				break;
			default			:
				break;
		}

		fseek(bin3ds, (h.len - 6), SEEK_CUR);
	}

	// After we have loaded the vertices we can load the faces
	fseek(bin3ds, findex, SEEK_SET);

	while (ftell(bin3ds) < (findex + length - 6))
	{
		fread(&h.id,sizeof(h.id),1,bin3ds);
		fread(&h.len,sizeof(h.len),1,bin3ds);

		switch (h.id)
		{
			case FACE_DESC	:
				// Load the faces of the object
				FacesDescriptionChunkProcessor(h.len, ftell(bin3ds), objindex);
				break;
			default			:
				break;
		}

		fseek(bin3ds, (h.len - 6), SEEK_CUR);
	}

	// move the file pointer back to where we got it so
	// that the ProcessChunk() which we interrupted will read
	// from the right place
	fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::VertexListChunkProcessor(int length, int findex, int objindex)
{
	unsigned short numVerts;

	// move the file pointer to the beginning of the main
	// chunk's data findex + the size of the header
	fseek(bin3ds, findex, SEEK_SET);

	// Read the number of vertices of the object
	fread(&numVerts,sizeof(numVerts),1,bin3ds);

	// Allocate arrays for the vertices and normals
	Objects[objindex].Vertexes = new GLdouble[numVerts * 3];
	Objects[objindex].Normals = new GLdouble[numVerts * 3];

	// Assign the number of vertices for future use
	Objects[objindex].numVerts = numVerts;

	// Zero out the normals array
	for (int j = 0; j < numVerts * 3; j++)
		Objects[objindex].Normals[j] = 0.0f;

	// Read the vertices, switching the y and z coordinates and changing the sign of the z coordinate
	for (int i = 0; i < numVerts * 3; i+=3)
	{
		//fread(&Objects[objindex].Vertexes[i],sizeof(GLdouble),1,bin3ds);
		//fread(&Objects[objindex].Vertexes[i+1],sizeof(GLdouble),1,bin3ds);
		//fread(&Objects[objindex].Vertexes[i+2],sizeof(GLdouble),1,bin3ds);
		float v1,v2,v3;

		fread(&v1,sizeof(GLfloat),1,bin3ds);
		fread(&v2,sizeof(GLfloat),1,bin3ds);
		fread(&v3,sizeof(GLfloat),1,bin3ds);
		Objects[objindex].Vertexes[i] = v1;
		Objects[objindex].Vertexes[i+2] = v2;
		Objects[objindex].Vertexes[i+1] = v3;

		// Change the sign of the z coordinate
		Objects[objindex].Vertexes[i+2] = -Objects[objindex].Vertexes[i+2];
	}

	length = PATH_LENGTH; // Remove Warnings
	// move the file pointer back to where we got it so
	// that the ProcessChunk() which we interrupted will read
	// from the right place
	fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::TexCoordsChunkProcessor(int length, int findex, int objindex)
{
	// The number of texture coordinates
	unsigned short numCoords;

	// move the file pointer to the beginning of the main
	// chunk's data findex + the size of the header
	fseek(bin3ds, findex, SEEK_SET);

	// Read the number of coordinates
	fread(&numCoords,sizeof(numCoords),1,bin3ds);

	// Allocate an array to hold the texture coordinates
	Objects[objindex].TexCoords = new GLdouble[numCoords * 2];

	// Set the number of texture coords
	Objects[objindex].numTexCoords = numCoords;

	// Read teh texture coordiantes into the array
	for (int i = 0; i < numCoords * 2; i+=2)
	{
		float p1,p2;
		fread(&p1,sizeof(GLfloat),1,bin3ds);
		fread(&p2,sizeof(GLfloat),1,bin3ds);
		Objects[objindex].TexCoords[i] = p1;
		Objects[objindex].TexCoords[i+1] = p2;
	}

	length = PATH_LENGTH; // Remove Warnings
	// move the file pointer back to where we got it so
	// that the ProcessChunk() which we interrupted will read
	// from the right place
	fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::FacesDescriptionChunkProcessor(int length, int findex, int objindex)
{
	ChunkHeader h;
	unsigned short numFaces;	// The number of faces in the object
	unsigned short vertA;		// The first vertex of the face
	unsigned short vertB;		// The second vertex of the face
	unsigned short vertC;		// The third vertex of the face
	unsigned short flags;		// The winding order flags
	int subs;			// Holds our place in the file
	int numMatFaces = 0;		// The number of different materials

	// move the file pointer to the beginning of the main
	// chunk's data findex + the size of the header
	fseek(bin3ds, findex, SEEK_SET);

	// Read the number of faces
	fread(&numFaces,sizeof(numFaces),1,bin3ds);

	// Allocate an array to hold the faces
	Objects[objindex].Faces = new GLushort[numFaces * 3];
	// Store the number of faces
	Objects[objindex].numFaces = numFaces * 3;

	// Read the faces into the array
	for (int i = 0; i < numFaces * 3; i+=3)
	{
		// Read the vertices of the face
		fread(&vertA,sizeof(vertA),1,bin3ds);
		fread(&vertB,sizeof(vertB),1,bin3ds);
		fread(&vertC,sizeof(vertC),1,bin3ds);
		fread(&flags,sizeof(flags),1,bin3ds);

		// Place them in the array
		Objects[objindex].Faces[i]   = vertA;
		Objects[objindex].Faces[i+1] = vertB;
		Objects[objindex].Faces[i+2] = vertC;

		// Calculate the face's normal
		Vector n;
		Vertex v1;
		Vertex v2;
		Vertex v3;

		v1.x = Objects[objindex].Vertexes[vertA*3];
		v1.y = Objects[objindex].Vertexes[vertA*3+1];
		v1.z = Objects[objindex].Vertexes[vertA*3+2];
		v2.x = Objects[objindex].Vertexes[vertB*3];
		v2.y = Objects[objindex].Vertexes[vertB*3+1];
		v2.z = Objects[objindex].Vertexes[vertB*3+2];
		v3.x = Objects[objindex].Vertexes[vertC*3];
		v3.y = Objects[objindex].Vertexes[vertC*3+1];
		v3.z = Objects[objindex].Vertexes[vertC*3+2];

/*

		Model3DS::Stri[0][0]=v1.x;
		Model3DS::Stri[0][1]=v1.y;
		Model3DS::Stri[0][2]=v1.z;
		Model3DS::Stri[1][0]=v2.x;
		Model3DS::Stri[1][1]=v2.y;
		Model3DS::Stri[1][2]=v2.z;
		Model3DS::Stri[2][0]=v3.x;
		Model3DS::Stri[2][1]=v3.y;
		Model3DS::Stri[2][2]=v3.z;

		*/

		// calculate the normal
		double u[3], v[3];

		// V2 - V3;
		u[0] = v2.x - v3.x;
		u[1] = v2.y - v3.y;
		u[2] = v2.z - v3.z;

		// V2 - V1;
		v[0] = v2.x - v1.x;
		v[1] = v2.y - v1.y;
		v[2] = v2.z - v1.z;

		n.x = (u[1]*v[2] - u[2]*v[1]);
		n.y = (u[2]*v[0] - u[0]*v[2]);
		n.z = (u[0]*v[1] - u[1]*v[0]);

		// Add this normal to its verts' normals
		Objects[objindex].Normals[vertA*3]   += n.x;
		Objects[objindex].Normals[vertA*3+1] += n.y;
		Objects[objindex].Normals[vertA*3+2] += n.z;
		Objects[objindex].Normals[vertB*3]   += n.x;
		Objects[objindex].Normals[vertB*3+1] += n.y;
		Objects[objindex].Normals[vertB*3+2] += n.z;
		Objects[objindex].Normals[vertC*3]   += n.x;
		Objects[objindex].Normals[vertC*3+1] += n.y;
		Objects[objindex].Normals[vertC*3+2] += n.z;
	}

	// Store our current file position
	subs = ftell(bin3ds);

	// Check to see how many materials the faces are split into
	while (ftell(bin3ds) < (findex + length - 6))
	{
		fread(&h.id,sizeof(h.id),1,bin3ds);
		fread(&h.len,sizeof(h.len),1,bin3ds);

		switch (h.id)
		{
			case FACE_MAT	:
				//FacesMaterialsListChunkProcessor(h.len, ftell(bin3ds), objindex);
				numMatFaces++;
				break;
			default			:
				break;
		}

		fseek(bin3ds, (h.len - 6), SEEK_CUR);
	}

	// Split the faces up according to their materials
	if (numMatFaces > 0)
	{
		// Allocate an array to hold the lists of faces divided by material
		Objects[objindex].MatFaces = new MaterialFaces[numMatFaces];
		// Store the number of material faces
		Objects[objindex].numMatFaces = numMatFaces;

		fseek(bin3ds, subs, SEEK_SET);

		int j = 0;

		// Split the faces up
		while (ftell(bin3ds) < (findex + length - 6))
		{
			fread(&h.id,sizeof(h.id),1,bin3ds);
			fread(&h.len,sizeof(h.len),1,bin3ds);

			switch (h.id)
			{
				case FACE_MAT	:
					// Process the faces and split them up
					FacesMaterialsListChunkProcessor(h.len, ftell(bin3ds), objindex, j);
					j++;
					break;
				default			:
					break;
			}

			fseek(bin3ds, (h.len - 6), SEEK_CUR);
		}
	}

	// move the file pointer back to where we got it so
	// that the ProcessChunk() which we interrupted will read
	// from the right place
	fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::FacesMaterialsListChunkProcessor(int length, int findex, int objindex, int subfacesindex)
{
	//char name[PATH_LENGTH];				// The material's name
	string name;
	unsigned short numEntries;	// The number of faces associated with this material
	unsigned short Face;		// Holds the faces as they are read
	int material;				// An index to the Materials array for this material

	// move the file pointer to the beginning of the main
	// chunk's data findex + the size of the header
	fseek(bin3ds, findex, SEEK_SET);

	// Read the material's name
	//bin3ds >> name;
	name.clear();
	length = PATH_LENGTH; // Remove Warnings
	for (int i = 0; i < PATH_LENGTH; i++)
	{
		name.push_back(fgetc(bin3ds));
		if (name[i] == 0)
		{
			name[i] = NULL;
			break;
		}
	}

	// Find the material's index in the Materials array
	for (material = 0; material < numMaterials; material++)
	{
		if (name == Materials[material].name)
			break;
	}

	// Store this value for later so that we can find the material
	Objects[objindex].MatFaces[subfacesindex].MatIndex = material;

	// Read the number of faces associated with this material
	fread(&numEntries,sizeof(numEntries),1,bin3ds);

	// Allocate an array to hold the list of faces associated with this material
	Objects[objindex].MatFaces[subfacesindex].subFaces = new GLushort[numEntries * 3];
	// Store this number for later use
	Objects[objindex].MatFaces[subfacesindex].numSubFaces = numEntries * 3;

	// Read the faces into the array
	for (int i = 0; i < numEntries * 3; i+=3)
	{
		// read the face
		fread(&Face,sizeof(Face),1,bin3ds);
		// Add the face's vertices to the list
		Objects[objindex].MatFaces[subfacesindex].subFaces[i] = Objects[objindex].Faces[Face * 3];
		Objects[objindex].MatFaces[subfacesindex].subFaces[i+1] = Objects[objindex].Faces[Face * 3 + 1];
		Objects[objindex].MatFaces[subfacesindex].subFaces[i+2] = Objects[objindex].Faces[Face * 3 + 2];
	}

	// move the file pointer back to where we got it so
	// that the ProcessChunk() which we interrupted will read
	// from the right place
	fseek(bin3ds, findex, SEEK_SET);
}
