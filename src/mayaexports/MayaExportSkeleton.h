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
#include "utils/RotationConversion.h"

using namespace std;

namespace model3d{
    class Skeleton;
    class BodyNode;
}   // namespace model3d

namespace mayaexports{

    class MayaExportSkeleton{
    public:    
        static bool exportMayaAscii( model3d::Skeleton *_model, ofstream &_outFile );
    
    private:
        friend class MayaExportMotion;
        
        static const string mSuffixPrim;
        static const string mSuffixShape;
        static const string mSuffixGeom;
        static const utils::rot_conv::RotationOrder mRotOrder;

        static bool exportMayaAsciiHeader(ofstream &_outFile );
        static bool exportMayaAsciiPrimitive(model3d::BodyNode* b, ofstream &_outFile, const string &_prefix, const string &_suffix);
        static bool exportMayaAsciiSegmentSubtree(model3d::BodyNode *_b, const int _writeNumNodes, ofstream &_outFile, const string &_prefix, const string &_suffix );
        // do not write the const stuff - all keyed now
        static bool exportMayaAsciiSegmentSubtree2(model3d::BodyNode *_b, const int _writeNumNodes, ofstream &_outFile, const string &_prefix, const string &_suffix );
        static bool exportMayaAsciiSegmentFooter(model3d::BodyNode* _b, const int _writeNumNodes, ofstream &_outFile, const string &_prefix, const string &_suffix  );

    };
}
#endif  // MAYAEXPORTS_MAYAEXPORTSKELETON_H
