/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sumit Jain
  Date		07/24/2011
*/
#ifndef MAYAEXPORTS_MAYAEXPORTMOTION_H
#define MAYAEXPORTS_MAYAEXPORTMOTION_H

#include <fstream>

namespace kinematics{
    class BodyNode;
    class Skeleton;
    class FileInfoDof;
}

namespace utils {
    namespace mayaexports {

        class MayaExportMotion{
        public:
            MayaExportMotion(kinematics::Skeleton *_skel, kinematics::FileInfoDof *_dofData){
                mSkel = _skel;
                mDofData = _dofData;
            }
            bool exportMayaAnim( const char* _fName, int _start, int _end, const std::string &_nodesPrefix, int _writeNumNodes );
    
        private:
            kinematics::FileInfoDof *mDofData;;
            kinematics::Skeleton *mSkel;
            bool exportMayaAnimSegment(std::ofstream &outFile0, int _first, int _last, kinematics::BodyNode *b, const std::string &_nodesPrefix, int _writeNumNodes, int level);
            // just write everything in common framework
            bool exportMayaAnimSegment2(std::ofstream &outFile0, int _first, int _last, kinematics::BodyNode *_b, const std::string &_nodesPrefix, int _writeNumNodes, int level);
        };
    }   // namespace mayaexports
} // namespace utils

#endif //MAYAEXPORTS_MAYAEXPORTMOTION_H
