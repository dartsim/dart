#ifndef SRC_MODEL3D_PARSER_VSK_H
#define SRC_MODEL3D_PARSER_VSK_H

namespace model3d {
  class Skeleton;
#define VERBOSE false
#define VSK_OK 0
#define VSK_ERROR 1
  int readVSKFile(const char* const filename, Skeleton* skel);
} // namespace model3d



#endif // #ifndef SRC_MODEL3D_PARSER_VSK_H

