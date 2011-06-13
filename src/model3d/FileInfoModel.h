/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/


#ifndef MODEL3D_FILEINFO_MODEL_H
#define MODEL3D_FILEINFO_MODEL_H

//#include "fileinfo_base.h"
#include "Skeleton.h"
#include <fstream>
#include <Eigen/Dense>

class BodyNode;

namespace model3d {

    class FileInfoModel /*: public FileInfoBase*/ {
    public:
  	enum FileType {
            VSK,
            SKEL
	};
  
    public:
        FileInfoModel();
        ~FileInfoModel();
	
        void draw(Renderer::OpenGLRenderInterface* RI, const Eigen::Vector4d& color, bool _default = true); // whether to use the default color stored in Skeleton; color is used only when default = false
        bool loadFile(const char* _filename, FileType _type);
        bool saveFile(const char* _filename);
        inline Skeleton* getSkel() const { return mSkel; }

    protected:
        Skeleton* mSkel;
        char mFileName[256]; // could use string instead

        // used in saveFile to write the subtree of _b
        void saveBodyNodeTree(BodyNode* _b, std::ofstream &_outfile, int _numLinks);
    };

} // namespace model3d

#endif // #ifndef MODEL3D_FILEINFO_MODEL_H

