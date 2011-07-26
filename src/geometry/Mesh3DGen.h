/*
    RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
    All rights reserved.

    Author	Sumit Jain
    Date	07/21/2011
*/

#ifndef GEOMETRY_MESH3DGEN_H
#define GEOMETRY_MESH3DGEN_H

#include <vector>

using namespace std;
#include <Eigen/dense>

namespace geometry { 
    class Mesh3DGen{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        enum MeshFormat {OBJ, OFF, CORNERTABLE};
        char mFileName[1024];

        unsigned int mNumVertices;
        unsigned int mNumFaces;
        vector<unsigned int> mNumFaceVertices;

        vector<Eigen::Vector2d> mVertexTextures;
        vector<Eigen::Vector3d> mVertexNormals;	// may not be same as the number of vertices

        Eigen::VectorXd mVertexPos;	// xyz in sequence
        Eigen::VectorXd mVertexVel;	// xyz in sequence
        vector<int> mFaces;	// list of 3 or more vertex indices in sequence for each face
        vector<vector<int> > mFaceTextureIndices;
        vector<vector<int> > mFaceNormalIndices;	// should exactly correspond to mFaces; indexes into the mVertexNormals

        Mesh3DGen(){
            mNumVertices = 0;
            mNumFaces=0;
            mVertexPos = Eigen::VectorXd(0);
            mVertexVel = Eigen::VectorXd(0);
            mVertexTextures.clear();
            mFaces.clear();
            mFileName[0]='\0';
        }
        virtual bool readMesh(const char *_file, MeshFormat _format);
        virtual bool writeMesh(const char *_file, MeshFormat _format);

        virtual double computeVolume()=0;

        virtual void draw(const Eigen::Vector4d& _color, bool _drawWireFrame, bool _drawSmooth=true)=0;

    };  // Mesh3DGen

} // namespace geometry

#endif  //GEOMETRY_MESH3DGEN_H

