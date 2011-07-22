#include "CornerTable.h"
#include "Mesh3DTriangle.h"
#include <map>

namespace geometry {

#define PREVCORNER(c) (c%3 == 0 ? c+2 : c-1)
#define NEXTCORNER(c) (c%3 == 2 ? c-2 : c+1)

    CornerTable::CornerTable( Mesh3DTriangle *_mesh3d ) {
        mMesh = _mesh3d;
    }

    void CornerTable::computeCornerOpposites() {
        mIndOppCorners.resize(mNumCorners, -1);

        //use hashing to accomplish this? ok.
        map<pair<int, int> , int> edgeCorners;
        map<pair<int, int> , int>::iterator it;

        edgeCorners.clear();
        for (int c = 0; c < mNumCorners; c++) {
            int v1 = v(n(c));
            int v2 = v(p(c));

            if (v2 > v1) {
                int a = v1;
                v1 = v2;
                v2 = a;
            }

            pair<int, int> edge = make_pair(v1, v2);

            it = edgeCorners.find(edge);
            if (it != edgeCorners.end()) { //if opposite corner found
                int oc = it->second;
                mIndOppCorners[c] = oc;
                mIndOppCorners[oc] = c;
                edgeCorners.erase(it);
            }
            else { //if opposite corner not found
                edgeCorners[edge] = c;
            }
        }
    }

    void CornerTable::checkConsistentOrientation() {
        for (int c = 0; c < mNumCorners; c++) {
            int cv1 = v(n(c));
            int cv2 = v(p(c));

            if (!b(c)) {
                int oc = o(c);
                int ocv1 = v(n(oc));
                int ocv2 = v(p(oc));

                if (!(ocv1 == cv2 && ocv2 == cv1)) {
                    fprintf(stderr, "The mesh is not consistently oriented\n");
                    fprintf(stderr, "c=%d\n", c);
                    fprintf(stderr, "oc=%d\n", oc);

                    fprintf(stderr, "1 (%d,%d)\n", cv1, cv2);
                    fprintf(stderr, "2 (%d,%d)\n", ocv1, ocv2);

                    exit(-1);
                }
            }
        }
    }

    void CornerTable::computeVertexToCornerLookup() {
        mIndCorners.resize(mMesh->mNumVertices, -1);
        for (int c = 0; c < mNumCorners; c++) {
            int vid = mMesh->mFaces[c];
            if(mIndCorners[vid]==-1) {
                mIndCorners[vid] = c;
            }
        }
    }

    bool CornerTable::isCornerBoundaryVertex(int sc) {
        bool rv = false;

        int c = sc;
        while (true) {
            c = sr(c);
            if (c == sc) break;
            if (c == -1) break;
        }
        if (c == -1) rv = true;

        return rv;
    }

    vector<int> CornerTable::verticesAroundCorner(int sc) {
        int c = sc;

        vector<int> vs;
        vs.clear();

        vs.push_back(v(n(sc)));

        while (true) {
            c = sr(c);
            if (c == sc)
                break;
            if (c == -1)
                break;
            vs.push_back(v(n(c)));
        }

        if (c == -1) {
            vector<int> tvs;
            tvs.clear();

            tvs.push_back(v(p(sc)));
            c = sc;
            while (true) {
                c = sl(c);
                if (c == sc)
                    break;
                if (c == -1)
                    break;

                tvs.push_back(v(p(c)));
            }

            vector<int> ttvs;
            ttvs.clear();
            for(int i=tvs.size()-1;i>=0;i--) {
                ttvs.push_back(tvs[i]);
            }
            for(unsigned int i=0;i<vs.size();i++) {
                ttvs.push_back(vs[i]);
            }

            vs = ttvs;
        }

        return vs;
    }

    int CornerTable::v( int c ){
        return mMesh->mFaces[c];
    }

    int CornerTable::n( int c ){
        return (c==-1?-1:NEXTCORNER(c));
    }

    int CornerTable::t( int c ){
        return c/3;
    }

    int CornerTable::o( int c ){
        if (c==-1) return -1;
        return mIndOppCorners[c];
    }

    Eigen::Vector3d CornerTable::gc( int _c ){
        int vind = v(_c);
        return Eigen::Vector3d(mMesh->mVertexPos[3*vind+0], mMesh->mVertexPos[3*vind+1], mMesh->mVertexPos[3*vind+2]);
    }

    int CornerTable::p( int c ){
        return (c==-1?-1:PREVCORNER(c));
    }

    int CornerTable::r( int c ){
        return o(p(c));
    }

    int CornerTable::l( int c ){
        return o(n(c));
    }

    int CornerTable::sr( int c ){
        return p(r(c));
    }

    int CornerTable::sl( int c ) {
        return n(l(c));
    }

    int CornerTable::s( int c ) {
        return sr(c);
    }

    Eigen::Vector3d CornerTable::gv( int _v ) {
        return Eigen::Vector3d(mMesh->mVertexPos[3*_v+0], mMesh->mVertexPos[3*_v+1], mMesh->mVertexPos[3*_v+2]);
    }

    bool CornerTable::b( int c ) {
        return o(c)==-1;
    }

} // namespace geometry
