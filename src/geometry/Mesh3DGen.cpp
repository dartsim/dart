#include "Mesh3DGen.h"
#include <string>
#include <fstream>
#include <cassert>
#include "utils/UtilsCode.h"

using namespace Eigen;

namespace geometry {

    Vector3d readObjVertex(vector<string> &_tokens){
        Vector3d v = Vector3d::Zero();
        assert(_tokens.size()==4);	// v px py pz
        for(int i=0; i<3; i++){
            istringstream issf (_tokens[i+1], istringstream::in);
            issf>>v[i];
        }
        return v;
    }

    Vector2d readObjVertexTexture(vector<string> &_tokens){
        Vector2d v = Vector2d::Zero();
        assert(_tokens.size()==3);	// vt px py
        for(int i=0; i<2; i++){
            istringstream issf (_tokens[i+1], istringstream::in);
            issf>>v[i];
        }
        return v;
    }

    Vector3d readObjVertexNormal(vector<string> &_tokens){
        Vector3d v = Vector3d::Zero();
        assert(_tokens.size()==4);	// vn px py pz
        for(int i=0; i<3; i++){
            istringstream issf (_tokens[i+1], istringstream::in);
            issf>>v[i];
        }
        return v;
    }

    void readObjFace(vector<string> &_tokens, vector<int> &_face, vector<int> &_faceTexture, vector<int> &_faceNormal){
        _face.clear();
        _faceTexture.clear();
        _faceNormal.clear();
        assert(_tokens.size()>3);	// f v1/vt1/vn1  ...
        for(unsigned int i=0; i<_tokens.size()-1; i++){
            string ti = _tokens[i+1];
            vector<string> tis;
            utils::tokenize(_tokens[i+1], tis, "/");
            int v=0;
            int vt=-1;
            int vn=-1;
            istringstream issd(tis[0], istringstream::in);
            issd>>v;
            if(tis.size()==3){ // v/vt/vn format
                // both vertex texture and normal are present
                istringstream issvt(tis[1], istringstream::in);
                issvt>>vt;
                istringstream issvn(tis[2], istringstream::in);
                issvn>>vn;
            }
            else if(tis.size()==2){	// v/vt or v//vn
                // either vertex texture exists or normal
                if(ti.find_first_of('/')==ti.find_last_of('/')){
                    istringstream issvt(tis[1], istringstream::in);
                    issvt>>vt;
                }
                else {
                    istringstream issvn(tis[1], istringstream::in);
                    issvn>>vn;
                }
            }
            else if(tis.size()==1){	// v
                // nothing else to find
            }
            if(vt!=-1) _faceTexture.push_back(vt);
            if(vn!=-1) _faceNormal.push_back(vn);
            _face.push_back(v);
        }
    }

    bool Mesh3DGen::readMesh(const char *_file, MeshFormat _format){
        bool success = true;
        ifstream inFile(_file);
        if(inFile.fail()==1) {
            cout<<"unable to load file: "<<_file<<endl;
            return false;
        }

        strcpy(mFileName, _file);

        if(_format==OFF){
            string line;
            vector<string> tokens;
            {	// OFF file header
                tokens.clear();
                getline(inFile, line);
                utils::tokenize(line, tokens);
                assert(tokens.size()==1 && tokens[0].compare("OFF")==0);
            }
            {	// read the number of vertices and faces
                tokens.clear();
                getline(inFile, line);
                utils::tokenize(line, tokens);
                assert(tokens.size()==3);
                istringstream issf (tokens[0], istringstream::in);
                issf>>mNumVertices;
                istringstream issd (tokens[1], istringstream::in);
                issd>>mNumFaces;
                cout<<"Number of vertices: "<<mNumVertices<<endl;
                cout<<"Number of faces: "<<mNumFaces<<endl;
                mNumFaceVertices.resize(mNumFaces, -1);
                mVertexPos = VectorXd(3*mNumVertices);	// xyz in sequence
                mVertexPos.setZero();
                mVertexVel = mVertexPos;	// xyz in sequence
                mFaces.clear();// resize faces later
            }
            // read the vertices
            for(unsigned int i=0; i<mNumVertices; i++){
                tokens.clear();
                while(getline(inFile, line) && line.find_first_not_of(" \t\v\r\n")==string::npos);
                utils::tokenize(line, tokens);
                assert(tokens.size()==3);
                vector<double> val = utils::stringToVec<double>(line);
                mVertexPos[3*i+0] = val[0];
                mVertexPos[3*i+1] = val[1];
                mVertexPos[3*i+2] = val[2];
            }
            // read the faces
            for(unsigned int i=0; i<mNumFaces; i++){
                tokens.clear();
                while(getline(inFile, line) && line.find_first_not_of(" \t\v\r\n")==string::npos);
                utils::tokenize(line, tokens);
                vector<int> vind= utils::stringToVec<int>(line);
                assert(vind.size()==vind[0]+1);
                mNumFaceVertices[i] = vind[0];
                for(int j=0; j<vind[0]; j++) mFaces.push_back(vind[1+j]);
            }
            success = true;
        }
        else if(_format==OBJ){
            success = true;
            mNumVertices = 0;
            mNumFaces = 0;
            vector<Vector3d> vertices;
            mFaces.clear();

            string line;
            vector<string> tokens;
            while(1){
                // read the line: ignore any comments or blank lines
                while(getline(inFile, line) && line.find_first_not_of(" \t\v\r\n")==string::npos);
                if(line.empty() || inFile.eof()) break;

                tokens.clear();
                utils::tokenize(line, tokens, " ");
                assert(tokens.size()>0);
                if(tokens[0].compare("#")==0) continue;
                else if(tokens[0].compare("v")==0) {
                    Vector3d v = readObjVertex(tokens);
                    vertices.push_back(v);
                }
                else if(tokens[0].compare("f")==0) {
                    vector<int> f;
                    vector<int> ft;
                    vector<int> fn;
                    readObjFace(tokens, f, ft, fn);
                    mNumFaceVertices.push_back(f.size());
                    for(unsigned int i=0; i<f.size(); i++) mFaces.push_back(f[i]-1);	// c++ indexing from 0
                    mFaceTextureIndices.push_back(ft);
                    mFaceNormalIndices.push_back(fn);
                    mNumFaces++;
                }
                else if(tokens[0].compare("vt")==0) {
                    Vector2d vt = readObjVertexTexture(tokens);
                    mVertexTextures.push_back(vt);
                }
                else if(tokens[0].compare("vn")==0) {
                    Vector3d vn = readObjVertexNormal(tokens);
                    mVertexNormals.push_back(vn);
                }
                else {
                    cout<<"Token: "<<tokens[0]<<" ignored\n";
                    //success = false;
                    //break;
                }
            }
            // assemble the vertices
            if(success){
                mNumVertices = vertices.size();
                mVertexPos = VectorXd(3*mNumVertices);	// xyz in sequence
                mVertexPos.setZero();
                mVertexVel = mVertexPos;	// xyz in sequence
                for(unsigned int i=0; i<mNumVertices; i++){
                    mVertexPos[3*i+0] = vertices[i][0];
                    mVertexPos[3*i+1] = vertices[i][1];
                    mVertexPos[3*i+2] = vertices[i][2];
                }
                vertices.clear();
            }
        }
        else if(_format==CORNERTABLE) {
            cout<<"Mesh3DGen: Only triangle meshes can read CORNERTABLE format\n";
            success = false;
        }
        inFile.close();
        if(success) cout<<"Loaded mesh file: "<<_file<<endl;
        return success;
    }

    bool Mesh3DGen::writeMesh(const char *_file, MeshFormat _format){
        bool success = false;
        ofstream outFile(_file);
        if(outFile.fail()==1) {
            cout<<"unable to open file for writing: "<<_file<<endl;
            return false;
        }

        if(_format==OFF){
            cout<<"TODO\n";
            success = false;
        }
        else if(_format==OBJ){
            // write header
            outFile<<"# OBJ FILE Generated by Mesh3DGen::writeMesh()\n";
            outFile<<"# num vertices: "<<mNumVertices<<endl;
            outFile<<"# num faces: "<<mNumFaces<<endl;
            outFile<<"# num texture coords: "<<mVertexTextures.size()<<endl;
            outFile<<"# num vertex normals: "<<mVertexNormals.size()<<endl;
            // write vertices
            for(unsigned int i=0; i<mNumVertices; i++){
                outFile<<"v";
                for(int k=0; k<3; k++) outFile<<" "<<mVertexPos[3*i+k];
                outFile<<endl;
            }
            //// write texture if available
            //for(int k=0; k<mVertexTextures.size(); k++){
            //	//if(mFaceTextureIndices[i][k]>=faceIndexCount) 
            //	outFile<<"vt"<<" "<<mVertexTextures[k][0]<<" "<<mVertexTextures[k][1]<<endl;
            //}
            // ignore vn - write s 1 for smooth shading
            outFile<<"s 1\n";

            // write faces
            int faceIndexCount=0;
            int vtIndexCount=0;
            int vnIndexCount=0;
            for(unsigned int i=0; i<mNumFaces; i++){
                // write texture if available
                for(unsigned int k=0; k<mFaceTextureIndices[i].size(); k++){
                    //if(mFaceTextureIndices[i][k]-1>=vtIndexCount) {
                    //	outFile<<"vt"<<" "<<mVertexTextures[mFaceTextureIndices[i][k]-1][0]<<" "<<mVertexTextures[mFaceTextureIndices[i][k]-1][1]<<endl;
                    //	vtIndexCount = mFaceTextureIndices[i][k];
                    //}
                    while(mFaceTextureIndices[i][k]-1>=vtIndexCount) {
                        outFile<<"vt"<<" "<<mVertexTextures[vtIndexCount][0]<<" "<<mVertexTextures[vtIndexCount][1]<<endl;
                        vtIndexCount++;
                    }
                }
                // write normal if available
                for(unsigned int k=0; k<mFaceNormalIndices[i].size(); k++){
                    //if(mFaceNormalIndices[i][k]-1>=vnIndexCount) {
                    //	outFile<<"vn"<<" "<<mVertexNormals[mFaceNormalIndices[i][k]-1][0]<<" "<<mVertexNormals[mFaceNormalIndices[i][k]-1][1]<<" "<<mVertexNormals[mFaceNormalIndices[i][k]-1][2]<<endl;
                    //	vnIndexCount = mFaceNormalIndices[i][k];
                    //}
                    while(mFaceNormalIndices[i][k]-1>=vnIndexCount) {
                        outFile<<"vn"<<" "<<mVertexNormals[vnIndexCount][0]<<" "<<mVertexNormals[vnIndexCount][1]<<" "<<mVertexNormals[vnIndexCount][2]<<endl;
                        vnIndexCount++;
                    }
                }
                outFile<<"f";
                for(unsigned int k=0; k<mNumFaceVertices[i]; k++) {
                    outFile<<" "<<mFaces[faceIndexCount+k]+1;	// c++ indexing + 1 = obj indexing
                    if(k<mFaceTextureIndices[i].size()) outFile<<"/"<<mFaceTextureIndices[i][k];
                    if(k<mFaceNormalIndices[i].size()) {
                        if(k>=mFaceTextureIndices[i].size()) outFile<<"/";	// no texture present
                        outFile<<"/"<<mFaceNormalIndices[i][k];
                    }
                }
                faceIndexCount+=mNumFaceVertices[i];
                outFile<<endl;
            }
            success = true;
        }
        else if(_format==CORNERTABLE) {
            cout<<"Mesh3DGen: Only triangle meshes can write CORNERTABLE format\n";
            success = false;
        }
        outFile.close();
        if(success) cout<<"Saved mesh file: "<<_file<<endl;
        return success;
    }

}   // namespace geometry
