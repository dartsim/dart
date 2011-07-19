/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/


#ifndef MODEL3D_FILEINFO_SKEL_H
#define MODEL3D_FILEINFO_SKEL_H

#include <fstream>
#include <Eigen/Dense>
#include "renderer/RenderInterface.h"

namespace model3d {
    
    class BodyNode;
    class Skeleton;

    class FileInfoSkel {
    public:
  	enum FileType {
            VSK,
            SKEL
	};
  
    public:
        FileInfoSkel();
        virtual ~FileInfoSkel();
	
        void draw(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color=Eigen::Vector4d::Ones(), bool _useDefaultColor = true) const; ///< Note: _color is not used when _useDefaultColor=false
        bool loadFile(const char* _filename, FileType _type);
        bool saveFile(const char* _filename) const;
        
        Skeleton* getSkel() const { return mSkel; }

    protected:
        Skeleton* mSkel;
        char mFileName[256]; // could use string instead

        // used in saveFile to write the subtree of _b
        void saveBodyNodeTree(BodyNode* _b, std::ofstream &_outfile, int _numLinks) const;
    };

} // namespace model3d

#endif // #ifndef MODEL3D_FILEINFO_SKEL_H

