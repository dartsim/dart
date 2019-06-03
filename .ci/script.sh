#!/usr/bin/env bash
set -ex

# Sanity checks for required environment variables.
if [ -z "$BUILD_TYPE" ]; then
  echo "Error: Environment variable BUILD_TYPE is unset."
  exit 1
fi

if [ -z "$BUILD_DARTPY" ]; then
  echo "Info: Environment variable BUILD_DARTPY is unset. Using OFF by default."
  BUILD_DARTPY=OFF
fi

if [ -z "$BUILD_DOCS" ]; then
  echo "Info: Environment variable BUILD_DOCS is unset. Using OFF by default."
  BUILD_DOCS=OFF
fi

if [ -z "$CODECOV" ]; then
  echo "Info: Environment variable CODECOV is unset. Using OFF by default."
  CODECOV=OFF
fi

if [ -z "$OS_NAME" ]; then
  echo "Error: Environment variable OS_NAME is unset."
  exit 1
fi

if [ -z "$MEMCHECK" ]; then
  echo "Info: Environment variable MEMCHECK is unset. Using OFF by default."
  MEMCHECK=OFF
fi

# Set number of threads for parallel build
# Ref: https://unix.stackexchange.com/a/129401
num_threads=4
while getopts ":j:" opt; do
  case $opt in
    j) num_threads="$OPTARG"
    ;;
    \?) echo "Invalid option -$OPTARG" >& 2
    ;;
  esac
done

# Set compilers
if [ "$COMPILER" = "gcc" ]; then
  export CC=gcc
  export CXX=g++
elif [ "$COMPILER" = "clang" ]; then
  export CC=clang
  export CXX=clang++
else
  echo "Info: Compiler isn't specified. Using the system default."
fi

# Skip Xenial and Bionic in push builds
if [ "$IS_PULL_REQUEST" = "false" ]; then
  if [ "$DOCKER_FILE" ]; then
    exit 0;
  fi
fi

if [ $BUILD_DOCS = "ON" ]; then
  . "${BUILD_DIR}/.ci/travis/build_docs.sh"
  exit 0
fi

mkdir build && cd build

if [ "$OS_NAME" = "linux" ]; then
  install_prefix_option="-DCMAKE_INSTALL_PREFIX=/usr/"
fi

cmake .. \
  -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
  -DDART_BUILD_DARTPY=$BUILD_DARTPY \
  -DDART_VERBOSE=ON \
  -DDART_TREAT_WARNINGS_AS_ERRORS=ON \
  -DDART_BUILD_EXTRAS=ON \
  -DDART_CODECOV=$CODECOV \
  ${install_prefix_option}

if [ "$BUILD_DARTPY" = "ON" ]; then
  make -j$num_threads dartpy
  make pytest
else
  if [ "$CODECOV" = "ON" ]; then
    make -j$num_threads all tests
  else
    make -j$num_threads all tutorials examples tests
  fi

  if [ "$OS_NAME" = "linux" ] && [ $(lsb_release -sc) = "bionic" ]; then
    make check-format
  fi

  if [ $CODECOV = "ON" ]; then
    make -j$num_threads codecov
  else
    ctest --output-on-failure -j$num_threads
    if [ $MEMCHECK = "ON" ]; then
      ctest --test-action memcheck --output-on-failure -j4
    fi
  fi
fi

# Make sure we can install with no issues
$SUDO make -j$num_threads install

if [ "$BUILD_DARTPY" = "ON" ]; then
  # Run a python example (experimental)
  if [ "$BUILD_DARTPY" = "ON" ]; then
    cd $BUILD_DIR/python/examples/hello_world
    python3 main.py
  fi
else
  # Build an example using installed DART
  cd $BUILD_DIR/examples/hello_world
  mkdir build && cd build
  cmake ..
  make -j$num_threads
fi
