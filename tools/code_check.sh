#!/bin/sh

# This shell script is for checking code style using "cpplint"
# https://code.google.com/p/google-styleguide/

CHECK_DIRS="../dart"
CHECK_FILES=`\
  find $CHECK_DIRS -name "*.cpp" -o -name "*.hh" -o -name "*.c" -o -name "*.h"`

# cpplint
echo $CHECK_FILES | xargs python cpplint.py 2>&1
