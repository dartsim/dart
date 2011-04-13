#ifndef SRC_MODEL3D_FILEINFO_MODEL_H
#define SRC_MODEL3D_FILEINFO_MODEL_H

#include "fileinfo_base.h"

namespace model3d {

  class FileInfoModel : public FileInfoBase {
  public:
    FileInfoModel();
    ~FileInfoModel();
	
    virtual void draw(int, bool _default = true);
    virtual bool loadFile(const char* const);
  };

} // namespace model3d

#endif // #ifndef SRC_MODEL3D_FILEINFO_MODEL_H

