#!/usr/bin/env bash
set -ex

num_threads=4

# https://unix.stackexchange.com/a/129401
while getopts ":j:" opt; do
  case $opt in
    j) num_threads="$OPTARG"
    ;;
    \?) echo "Invalid option -$OPTARG" >& 2
    ;;
  esac
done

if [ "$COMPILER" = "gcc" ]; then
  export CC=gcc
  export CXX=g++
elif [ "$COMPILER" = "clang" ]; then
  export CC=clang
  export CXX=clang++
fi

# Skip Xenial and Bionic in push builds
if [ "$IS_PULL_REQUEST" = "false" ]; then
  if [ "$DOCKER_FILE" ]; then
    exit 0;
  fi
fi

if [ $BUILD_NAME = DOCS ]; then
  . "${BUILD_DIR}/.ci/travis/build_docs.sh"
  exit 0
fi

mkdir build && cd build

if [ "$BUILD_NAME" = "BIONIC_DEBUG" ]; then
  cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DDART_VERBOSE=ON -DDART_TREAT_WARNINGS_AS_ERRORS=ON -DDART_BUILD_EXTRAS=ON -DDART_CODECOV=ON ..
else
  cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DDART_VERBOSE=ON -DDART_TREAT_WARNINGS_AS_ERRORS=ON -DDART_BUILD_EXTRAS=ON -DDART_CODECOV=OFF ..
fi

if [ "$OS_NAME" = "linux" ]; then
  make -j$num_threads all tutorials examples tests
else
  make -j$num_threads all tests
fi

if [ "$OS_NAME" = "linux" ] && [ $(lsb_release -sc) = "bionic" ]; then
  make check-format
fi

if [ $CODECOV = ON ]; then
  make -j4 codecov
else
  ctest --output-on-failure -j4
fi

# Make sure we can install with no issues
$SUDO make -j4 install

# Build an example using installed DART
cd $BUILD_DIR/examples/rigidCubes
mkdir build && cd build
cmake ..
make -j4

# Uploading report to CodeCov
if [ "$BUILD_NAME" = "BIONIC_DEBUG" ]; then
  bash <(curl -s https://codecov.io/bash) || echo "Codecov did not collect coverage reports"
fi
