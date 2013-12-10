#!/bin/sh

# This shell script is for checking code style using "cpplint"
# https://code.google.com/p/google-styleguide/

CHECK_DIRS="../dart/common\
            ../dart/constraint\
            ../dart/math\
            ../dart/dynamics\
            ../dart/gui\
            ../dart/simulation"
CHECK_FILES=`\
  find $CHECK_DIRS\
  -name "*.cpp"\
  -not -name "lodepng.cpp"\
  -or -name "*.hh" -or -name "*.c"\
  -or -name "*.h"\
  -not -name "Jitter.h"\
  -not -name "lodepng.h"`

# cpplint
echo $CHECK_FILES | xargs python cpplint.py 2>&1
