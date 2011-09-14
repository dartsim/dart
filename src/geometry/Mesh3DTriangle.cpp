#include "Mesh3DTriangle.h"
#include "CornerTable.h"
#include <cassert>
#include <fstream>
#include <iostream>
#include "utils/LoadOpengl.h"
#include "utils/UtilsCode.h"

using namespace Eigen;

namespace geometry {
    Mesh3DTriangle::Mesh3DTriangle() {
        mCornerTable = NULL;
    }

    Mesh3DTriangle::Mesh3DTriangle( const char *_file, MeshFormat _format ) {
        // load obj file
        readMesh(_file, _format);
    }

    bool Mesh3DTriangle::readMesh(const char *_file, Mesh3DGen::MeshFormat _format){
        if(_format==Mesh3DGen::CORNERTABLE){
            return readCorners(_file);
        }
        else{
            if(!Mesh3DGen::readMesh(_file, _format)) return false;
        }
        if(mVertexPos.size()!=mVertexVel.size()) mVertexVel = VectorXd::Zero(mVertexPos.size());
        mCornerTable = new CornerTable(this);
        mCornerTable->mNumCorners = 3*mNumFaces;
        assert(mCornerTable->mNumCorners==mFaces.size());
        // compute opposites and others
        mCornerTable->initCornerTable();
        fprintf(stderr, "Computed corner table\n");
        return true;
    }

    bool Mesh3DTriangle::writeMesh(const char *_file, Mesh3DGen::MeshFormat _format){
        assert(mFaces.size()==3*mNumFaces);	// triangle face
        if(_format==Mesh3DGen::CORNERTABLE){
            return writeCorners(_file);
        }
        else{
            return Mesh3DGen::writeMesh(_file, _format);
        }
    }

    bool Mesh3DTriangle::readCorners(const char *fileName) {
        FILE *file = fopen(fileName, "r");
        if (file) {
            mCornerTable = new CornerTable(this);
            fprintf(stderr, "reading from file: %s\n", fileName);

            int result = 0;
            result = fscanf(file, "%d", &mNumVertices);
            
            mVertexPos = VectorXd::Zero(3*mNumVertices);
            mVertexVel = VectorXd::Zero(3*mNumVertices);
            for (unsigned int i = 0; i < mNumVertices; i++) {
                result = fscanf(file, "%lf %lf %lf", &mVertexPos[3*i+0], &mVertexPos[3*i+1], &mVertexPos[3*i+2]);
            }

            result = fscanf(file, "%d", &mNumFaces);
            mCornerTable->mNumCorners = mNumFaces * 3;
            mFaces.resize(mCornerTable->mNumCorners, -1);
            for (unsigned int i = 0; i < mNumFaces; i++) {
                result = fscanf(file, "%d %d %d", &mFaces[3*i+0], &mFaces[3*i+1], &mFaces[3*i+2]);
            }

            result = fscanf(file, "%d", &mNumFaces);
            mCornerTable->mIndOppCorners.resize(mCornerTable->mNumCorners, -1);
            for (unsigned int i = 0; i < mNumFaces; i++) {
                result = fscanf(file, "%d %d %d", &mCornerTable->mIndOppCorners[3*i+0], &mCornerTable->mIndOppCorners[3*i+1], &mCornerTable->mIndOppCorners[3*i+2]);
            }
            fclose(file);
            return true;
        }
        else {
            fprintf(stderr, "Couldn't open file: %s for reading CornerTable\n", fileName);
            return false;
        }
    }

    bool Mesh3DTriangle::writeCorners(const char *fileName) {
        FILE *file = fopen(fileName, "w");
        if (file && mCornerTable) {
            fprintf(file, "%d\n", mNumVertices);
            for (unsigned int i = 0; i < mNumVertices; i++) {
                fprintf(file, "%f %f %f\n", mVertexPos[3*i+0], mVertexPos[3*i+1], mVertexPos[3*i+2]);
            }

            fprintf(file, "%d\n", mNumFaces);
            for (unsigned int i = 0; i < mNumFaces; i++) {
                fprintf(file, "%d %d %d\n", mFaces[3*i+0], mFaces[3*i+1], mFaces[3*i+2]);
            }

            fprintf(file, "%d\n", mNumFaces);
            for (unsigned int i = 0; i < mNumFaces; i++) {
                fprintf(file, "%d %d %d\n", mCornerTable->mIndOppCorners[3*i+0], mCornerTable->mIndOppCorners[3*i+1], mCornerTable->mIndOppCorners[3*i+2]);
            }

            fclose(file);
            return true;
        } 
        else {
            fprintf(stderr, "Couldn't open file: %s for writing CornerTable\n", fileName);
            return false;
        }
    }


    double Mesh3DTriangle::computeVolume(){
        double volume = 0;
        for (unsigned int i=0; i<mNumFaces; i++){
            const double *p1 = &mVertexPos[3*mFaces[3*i+0]];
            const double *p2 = &mVertexPos[3*mFaces[3*i+1]];
            const double *p3 = &mVertexPos[3*mFaces[3*i+2]];

            // compute the volume of the tetrahedra relative to the origin
            volume += p1[0]*p2[1]*p3[2] + p2[0]*p3[1]*p1[2] + p3[0]*p1[1]*p2[2] -p1[0]*p3[1]*p2[2] - p2[0]*p1[1]*p3[2] - p3[0]*p2[1]*p1[2];
        }

        volume*=(1.0/6.0);
        //if (volume < 0) volume=-volume;
        return volume;
    }

    void Mesh3DTriangle::computeVolDer(VectorXd &_der){
        _der.resize(mNumVertices*3);
        _der.setZero();
        for (unsigned int i=0; i<mNumFaces; i++){
            const double *p1 = &mVertexPos[3*mFaces[3*i+0]];
            const double *p2 = &mVertexPos[3*mFaces[3*i+1]];
            const double *p3 = &mVertexPos[3*mFaces[3*i+2]];

            // wrt p1
            _der[3*mFaces[3*i+0] + 0] += p2[1]*p3[2] - p3[1]*p2[2];
            _der[3*mFaces[3*i+0] + 1] += p3[0]*p2[2] - p2[0]*p3[2];
            _der[3*mFaces[3*i+0] + 2] += p2[0]*p3[1] - p3[0]*p2[1];
            // wrt p2
            _der[3*mFaces[3*i+1] + 0] += p3[1]*p1[2] - p1[1]*p3[2];
            _der[3*mFaces[3*i+1] + 1] += p1[0]*p3[2] - p3[0]*p1[2];
            _der[3*mFaces[3*i+1] + 2] += p3[0]*p1[1] - p1[0]*p3[1];
            // wrt p3
            _der[3*mFaces[3*i+2] + 0] += p1[1]*p2[2] - p2[1]*p1[2];
            _der[3*mFaces[3*i+2] + 1] += p2[0]*p1[2] - p1[0]*p2[2];
            _der[3*mFaces[3*i+2] + 2] += p1[0]*p2[1] - p2[0]*p1[1];

        }
        _der*=(1.0/6.0);
    }


    void Mesh3DTriangle::draw(const Vector4d& _color, bool _drawWireFrame, bool _drawSmooth){
        GLboolean lightstate = glIsEnabled(GL_LIGHTING);
        if(!lightstate) glEnable( GL_LIGHTING );
        if(!_drawWireFrame){
            static vector<Vector3d> vnormal(mNumVertices, Vector3d::Zero());
            if(_drawSmooth){
                // make all zero
                for(unsigned int i=0; i<mNumVertices; i++)	vnormal[i].setZero();
                // sum up normals at each vertex
                for(unsigned int i=0; i<mNumFaces; i++){
                    Vector3d v0 = getVertex(mFaces[3*i+0]);
                    Vector3d v1 = getVertex(mFaces[3*i+1]);
                    Vector3d v2 = getVertex(mFaces[3*i+2]);
                    Vector3d nf = ((v1-v0).cross(v2-v1)).normalized();
                    vnormal[mFaces[3*i+0]] += nf;
                    vnormal[mFaces[3*i+1]] += nf;
                    vnormal[mFaces[3*i+2]] += nf;
                }
                // normalize all
                for(unsigned int i=0; i<mNumVertices; i++)	vnormal[i] = vnormal[i].normalized();
            }

            glColor4dv(_color.data());
            glBegin(GL_TRIANGLES);						// Drawing Using Triangles
            for(unsigned int i=0; i<mNumFaces; i++){
                Vector3d v0 = getVertex(mFaces[3*i+0]);
                Vector3d v1 = getVertex(mFaces[3*i+1]);
                Vector3d v2 = getVertex(mFaces[3*i+2]);
                if(!_drawSmooth){
                    Vector3d nf = ((v1-v0).cross(v2-v1)).normalized();
                    glNormal3dv(nf.data());
                }
                if(_drawSmooth) glNormal3dv(vnormal[mFaces[3*i+0]].data());
                glVertex3d(v0[0], v0[1], v0[2]);
                if(_drawSmooth) glNormal3dv(vnormal[mFaces[3*i+1]].data());
                glVertex3d(v1[0], v1[1], v1[2]);
                if(_drawSmooth) glNormal3dv(vnormal[mFaces[3*i+2]].data());
                glVertex3d(v2[0], v2[1], v2[2]);
            }
            glEnd();
        }
        else{
            glColor4dv(_color.data());
            glLineWidth(2.0);
            for(unsigned int i=0; i<mNumFaces; i++){
                glBegin(GL_LINE_LOOP);
                Vector3d v0 = getVertex(mFaces[3*i+0]);
                Vector3d v1 = getVertex(mFaces[3*i+1]);
                Vector3d v2 = getVertex(mFaces[3*i+2]);
                glVertex3d(v0[0], v0[1], v0[2]);
                glVertex3d(v1[0], v1[1], v1[2]);
                glVertex3d(v2[0], v2[1], v2[2]);
                glEnd();
            }
        }
        if(!lightstate) glDisable( GL_LIGHTING );
    }

    vector<double> Mesh3DTriangle::getBarycentricCoords(const Vector3d& _p, const Vector3d& _v1, const Vector3d& _v2, const Vector3d& _v3){
        // barycentric coords
        MatrixXd B = MatrixXd::Zero(3, 2);
        for(int ri=0; ri<3; ri++) B(ri, 0) = _v1[ri]-_v3[ri];
        for(int ri=0; ri<3; ri++) B(ri, 1) = _v2[ri]-_v3[ri];
        VectorXd rhs = VectorXd::Zero(3);
        for(int ri=0; ri<3; ri++) rhs[ri] = _p[ri]-_v3[ri];
        JacobiSVD<MatrixXd> svdB(B, ComputeThinU | ComputeThinV);
        VectorXd brv = svdB.solve(rhs);
        //MatrixXd Binv = MatrixXd::Zero(2, 3);
        //svdB.pseudoInv(Binv);
        //VectorXd brv = Binv*rhs;
        vector<double> br(3, 0.0);
        br[0] = brv[0];
        br[1] = brv[1];
        br[2] = 1-(brv[0]+brv[1]);
        return br;
    }

    vector<pair<int, int> > Mesh3DTriangle::getVertexCorres(Mesh3DTriangle *_mesh1, Mesh3DTriangle *_mesh2){
        vector<pair<int, int> > vcorr;
        vcorr.clear();

        for(unsigned int i=0; i<_mesh1->mNumVertices; i++){
            //if(!_mesh1->mCornerTable->isVertexBoundaryVertex(i)) continue;
            for(unsigned int j=0; j<_mesh2->mNumVertices; j++){
                //if(!_mesh2->mCornerTable->isVertexBoundaryVertex(j)) continue;
                if((_mesh1->getVertex(i)-_mesh2->getVertex(j)).norm()<1.0e-6){
                    vcorr.push_back(pair<int,int>(i,j));
                }
            }
        }

        return vcorr;

    }

    void Mesh3DTriangle::writeCorres(const char *_file, Mesh3DTriangle *_m1, Mesh3DTriangle *_m2, const vector<pair<int, int> > &_corres){
        cout<<"Num corres: "<<_corres.size()<<endl<<endl;
        ofstream fc(_file);
        fc<<"# "<<_m1->mFileName<<" "<<_m2->mFileName<<endl;
        fc<<_corres.size()<<endl;
        for(unsigned int i=0; i<_corres.size(); i++){
            fc<<_corres[i].first<<" "<<_corres[i].second<<endl;
        }
        fc.close();
    }

    void Mesh3DTriangle::readCorres(const char *_file, vector<pair<int, int> > &_corres){
        _corres.clear();
        ifstream fc(_file);

        string line;
        vector<string> tokens;
        tokens.clear();
        // ignore the first line
        getline(fc, line);
        // read the number of corres
        getline(fc, line);
        utils::tokenize(line, tokens);
        assert(tokens.size()==1);
        int numCorres = -1;
        istringstream issf (tokens[0], istringstream::in);
        issf>>numCorres;
        for(int i=0; i<numCorres; i++){
            tokens.clear();
            getline(fc, line);
            utils::tokenize(line, tokens);
            assert(tokens.size()==2);
            int v1=-1;
            int v2=-1;
            {
                istringstream issf (tokens[0], istringstream::in);
                issf>>v1;
            }
            {
                istringstream issf (tokens[1], istringstream::in);
                issf>>v2;
            }
            _corres.push_back(pair<int,int>(v1,v2));
        }
        fc.close();
        cout<<"Num corres read: "<<_corres.size()<<endl<<endl;
    }


}   // namespace geometry
