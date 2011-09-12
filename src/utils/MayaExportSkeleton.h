/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sumit Jain
  Date		07/24/2011
*/
#ifndef MAYAEXPORTS_MAYAEXPORTSKELETON_H
#define MAYAEXPORTS_MAYAEXPORTSKELETON_H

#include <fstream>
#include <string>
#include "utils/UtilsRotation.h"

using namespace std;

namespace kinematics{
    class Skeleton;
    class BodyNode;
}   // namespace kinematics

namespace utils {
    namespace mayaexports {

        class MayaExportSkeleton{
        public:    
            static bool exportMayaAscii( kinematics::Skeleton *_model, ofstream &_outFile );
    
        private:
            friend class MayaExportMotion;
        
            static const string mSuffixPrim;
            static const string mSuffixShape;
            static const string mSuffixGeom;
            static const utils::rotation::RotationOrder mRotOrder;

            static bool exportMayaAsciiHeader(ofstream &_outFile );
            static bool exportMayaAsciiPrimitive(kinematics::BodyNode* b, ofstream &_outFile, const string &_prefix, const string &_suffix);
            static bool exportMayaAsciiSegmentSubtree(kinematics::BodyNode *_b, const int _writeNumNodes, ofstream &_outFile, const string &_prefix, const string &_suffix );
            // do not write the const stuff - all keyed now
            static bool exportMayaAsciiSegmentSubtree2(kinematics::BodyNode *_b, const int _writeNumNodes, ofstream &_outFile, const string &_prefix, const string &_suffix );
            static bool exportMayaAsciiSegmentFooter(kinematics::BodyNode* _b, const int _writeNumNodes, ofstream &_outFile, const string &_prefix, const string &_suffix  );

        };
    } // namespace mayaexports
} // namespace utils

#endif  // MAYAEXPORTS_MAYAEXPORTSKELETON_H
