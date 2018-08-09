#!/usr/bin/env bash
set -ex

# Skip Xenial and Bionic in push builds
if [ "$TRAVIS_PULL_REQUEST" = "false" ]; then
  if [ "$DOCKER_FILE" ]; then
    exit 0;
  fi
fi

if [ $BUILD_NAME = DOCS ]; then
  . "${TRAVIS_BUILD_DIR}/.ci/build_docs.sh"
  exit 0
fi

mkdir build && cd build

if [ "$BUILD_NAME" = "TRUSTY_DEBUG" ]; then
  cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DDART_VERBOSE=ON -DDART_TREAT_WARNINGS_AS_ERRORS=ON -DDART_CODECOV=ON ..
else
  cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DDART_VERBOSE=ON -DDART_TREAT_WARNINGS_AS_ERRORS=ON -DDART_CODECOV=OFF ..
fi

if [ "$TRAVIS_OS_NAME" = "linux" ]; then
  make -j4 tutorials examples tests
else
  make -j4 tests
fi

if [ "$TRAVIS_OS_NAME" = "linux" ] && [ $(lsb_release -sc) = "trusty" ]; then
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
cd $TRAVIS_BUILD_DIR/examples/rigidCubes
mkdir build && cd build
cmake ..
make -j4
