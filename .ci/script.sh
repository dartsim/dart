#!/usr/bin/env bash
set -ex

# Sanity checks for required environment variables.
if [ -z "$BUILD_TYPE" ]; then
  echo "Error: Environment variable BUILD_TYPE is unset."
  exit 1
fi

if [ -z "$RUN_TESTS" ]; then
  echo "Info: Environment variable RUN_TESTS is unset. Using ON by default."
  RUN_TESTS=ON
fi

if [ -z "$RUN_INSTALL_TEST" ]; then
  echo "Info: Environment variable RUN_INSTALL_TEST is unset. Using ON by default."
  RUN_INSTALL_TEST=ON
fi

if [ -z "$BUILD_DARTPY" ]; then
  echo "Info: Environment variable BUILD_DARTPY is unset. Using OFF by default."
  BUILD_DARTPY=OFF
fi

if [ -z "$BUILD_DOCS" ]; then
  echo "Info: Environment variable BUILD_DOCS is unset. Using OFF by default."
  BUILD_DOCS=OFF
fi

if [ -z "$COMPILER" ]; then
  echo "Info: Environment variable COMPILER is unset. Using gcc by default."
  COMPILER=gcc
fi

if [ -z "$CODECOV" ]; then
  echo "Info: Environment variable CODECOV is unset. Using OFF by default."
  CODECOV=OFF
fi

if [ -z "$BUILD_DIR" ]; then
  echo "Error: Environment variable BUILD_DIR is unset. Using $PWD by default."
  BUILD_DIR=$PWD
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

if [ "$OSTYPE" = "linux-gnu" ]; then
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
  elif [ "$RUN_TESTS" = "ON" ]; then
    make -j$num_threads all tutorials examples tests
  fi

  if [ "$OSTYPE" = "linux-gnu" ] && [ $(lsb_release -sc) = "bionic" ]; then
    make check-format
  fi

  if [ $CODECOV = "ON" ]; then
    make -j$num_threads codecov
  elif [ "$RUN_TESTS" = "ON" ]; then
    ctest --output-on-failure -j$num_threads
  fi
fi

if [ "$RUN_INSTALL_TEST" = "ON" ]; then
  # Make sure we can install with no issues
  make -j$num_threads install

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
fi
