#!/usr/bin/env bash
set -e

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

if [ -f /etc/os-release ]; then
  # freedesktop.org and systemd
  . /etc/os-release
  OS=$NAME
  VER=$VERSION_ID
elif type lsb_release >/dev/null 2>&1; then
  # linuxbase.org
  OS=$(lsb_release -si)
  VER=$(lsb_release -sr)
elif [ -f /etc/lsb-release ]; then
  # For some versions of Debian/Ubuntu without lsb_release command
  . /etc/lsb-release
  OS=$DISTRIB_ID
  VER=$DISTRIB_RELEASE
elif [ -f /etc/debian_version ]; then
  # Older Debian/Ubuntu/etc.
  OS=Debian
  VER=$(cat /etc/debian_version)
elif [ -f /etc/SuSe-release ]; then
  # Older SuSE/etc.
  echo "Not supported"
  exit 1
elif [ -f /etc/redhat-release ]; then
  # Older Red Hat, CentOS, etc.
  echo "Not supported"
  exit 1
else
  # Fall back to uname, e.g. "Linux <version>", also works for BSD, etc.
  OS=$(uname -s)
  VER=$(uname -r)
fi

# Set number of threads for parallel build
# Ref: https://unix.stackexchange.com/a/129401
if [ "$OSTYPE" = "linux-gnu" ]; then
  num_available_threads=$(nproc)
elif [ "$OSTYPE" = "darwin" ]; then
  num_available_threads=$(sysctl -n hw.logicalcpu)
else
  num_available_threads=1
  echo "$OSTYPE is not supported to detect the number of logical CPU cores."
fi
num_threads=$num_available_threads
while getopts ":j:" opt; do
  case $opt in
  j)
    num_threads="$OPTARG"
    ;;
  \?)
    echo "Invalid option -$OPTARG" >&2
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

# Build API documentation and exit
if [ $BUILD_DOCS = "ON" ]; then
  . "${BUILD_DIR}/.ci/build_docs.sh"
  exit 0
fi

echo "====================================="
echo ""
echo " [ SYSTEM INFO ]"
echo ""
echo " OS      : $OS $VER ($(uname -m))"
echo " Cores   : $num_threads / $num_available_threads"
echo " Compiler: $COMPILER $($CXX --version | perl -pe '($_)=/([0-9]+([.][0-9]+)+)/')"
echo " CMake   : $(cmake --version | perl -pe '($_)=/([0-9]+([.][0-9]+)+)/')"
echo ""
echo "====================================="

# Run CMake
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

# Check format
if [ "$OSTYPE" = "linux-gnu" ] && [ $(lsb_release -sc) = "bionic" ]; then
  make check-format
fi

# DART: build, test, and install
make -j$num_threads all tutorials examples tests
ctest --output-on-failure -j$num_threads
make -j$num_threads install

# dartpy: build, test, and install
if [ "$BUILD_DARTPY" = "ON" ]; then
  make -j$num_threads dartpy
  make pytest
  make -j$num_threads install-dartpy
fi

# Codecov
if [ "$CODECOV" = "ON" ]; then
  lcov --directory . --capture --output-file coverage.info
  # filter out system and extra files.
  # To also not include test code in coverage add them with full path to the patterns: '*/tests/*'
  lcov --remove coverage.info '/usr/*' "${HOME}"'/.cache/*' --output-file coverage.info
  # output coverage data for debugging (optional)
  lcov --list coverage.info
  # Uploading to CodeCov
  # '-f' specifies file(s) to use and disables manual coverage gathering and file search which has already been done above
  bash <(curl -s https://codecov.io/bash) -f coverage.info || echo "Codecov did not collect coverage reports"
fi

# DART: build an C++ example using installed DART
cd $BUILD_DIR/examples/hello_world
mkdir build && cd build
cmake ..
make -j$num_threads

# dartpy: run a Python example using installed dartpy
if [ "$BUILD_DARTPY" = "ON" ]; then
  cd $BUILD_DIR/python/examples/hello_world
  python3 main.py
fi
