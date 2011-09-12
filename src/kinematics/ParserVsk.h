/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/


#ifndef KINEMATICS_PARSER_VSK_H
#define KINEMATICS_PARSER_VSK_H

namespace kinematics {
    class Skeleton;
} // namespace kinematics

#define VERBOSE false
#define VSK_OK 0
#define VSK_ERROR 1
int readVSKFile(const char* const filename, kinematics::Skeleton* skel);

#endif // #ifndef KINEMATICS_PARSER_VSK_H

