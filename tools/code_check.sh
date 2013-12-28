#!/bin/sh

# This shell script is for checking code style using "cpplint"
# https://code.google.com/p/google-styleguide/

CHECK_DIRS="#../dart/collision\
            ../dart/common\
            ../dart/constraint\
            #../dart/dynamics\
            #../dart/gui\
            #../dart/integration\
            #../dart/lcpsolver\
            #../dart/math\
            #../dart/optimizer\
            #../dart/planning\
            #../dart/renderer\
            #../dart/simulation\
            #../dart/utils\
            #../apps/cubes\
            #../apps/meshCollision"
NOT_CHECK_DIRS=""
CHECK_FILES=`\
  find $CHECK_DIRS\
  -name "*.cpp"\
  -not -name "DARTCollide.cpp"\
  -not -name "BulletCollisionDetector.cpp"\
  -not -name "lodepng.cpp"\
  -not -name "error.cpp"\
  -not -name "fastdot.cpp"\
  -not -name "fastldlt.cpp"\
  -not -name "fastlsolve.cpp"\
  -not -name "fastltsolve.cpp"\
  -not -name "lcp.cpp"\
  -not -name "matrix.cpp"\
  -not -name "misc.cpp"\
  -or -name "*.hh"\
  -or -name "*.c"\
  -or -name "*.h"\
  -not -name "tri_tri_intersection_test.h"\
  -not -name "Jitter.h"\
  -not -name "lodepng.h"\
  -not -name "common.h"\
  -not -name "error.h"\
  -not -name "lcp.h"\
  -not -name "matrix.h"\
  -not -name "misc.h"\
  -not -name "odeconfig.h"\
  -not -name "btBulletCollisionCommon.h"\
  | grep -v "../dart/collision/bullet/LinearMath/"\
  | grep -v "../dart/collision/bullet/BulletCollision/"`

# cpplint
echo $CHECK_FILES | xargs python cpplint.py 2>&1
