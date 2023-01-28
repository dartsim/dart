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

if [ -z "$CHECK_FORMAT" ]; then
  echo "Info: Environment variable CHECK_FORMAT is unset. Using OFF by default."
  CHECK_FORMAT=OFF
fi

if [ -z "$NUM_CORES" ]; then
  echo "Info: Environment variable NUM_CORES is unset. Using MAX by default."
  NUM_CORES=MAX
fi

if [ -z "$BUILD_EXAMPLES" ]; then
  echo "Info: Environment variable BUILD_EXAMPLES is unset. Using ON by default."
  BUILD_EXAMPLES=ON
fi

if [ -z "$BUILD_TUTORIALS" ]; then
  echo "Info: Environment variable BUILD_TUTORIALS is unset. Using ON by default."
  BUILD_TUTORIALS=ON
fi

if [ -z "$TEST_INSTALLATION" ]; then
  echo "Info: Environment variable TEST_INSTALLATION is unset. Using ON by default."
  TEST_INSTALLATION=ON
fi

if [ -z "$IN_CI" ]; then
  echo "Info: Environment variable IN_CI is unset. Using OFF by default."
  IN_CI=OFF
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
elif [[ $OSTYPE = darwin* ]]; then
  num_available_threads=$(sysctl -n hw.logicalcpu)
else
  num_available_threads=1
  echo "[WARN] $OSTYPE is not supported to detect the number of logical CPU cores."
fi

if [ "$NUM_CORES" = "MAX" ]; then
  num_threads=$num_available_threads
else
  num_threads=$NUM_CORES
fi

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
echo " OSTYPE  : $OSTYPE"
echo " Cores   : $num_threads / $num_available_threads"
echo " Compiler: $COMPILER $($CXX --version | perl -pe '($_)=/([0-9]+([.][0-9]+)+)/')"
echo " CMake   : $(cmake --version | perl -pe '($_)=/([0-9]+([.][0-9]+)+)/')"
echo ""
echo "====================================="

# Set up the trap command
trap "cleanup" EXIT

# Run CMake
build_dir=${BUILD_DIR}/build
if [ "$OSTYPE" = "linux-gnu" ]; then
  install_prefix_option="-DCMAKE_INSTALL_PREFIX=/usr/"
elif [[ $OSTYPE = darwin* ]]; then
  install_prefix_option="-DCMAKE_INSTALL_PREFIX=/usr/local/ -DCMAKE_INSTALL_RPATH=/usr/local/lib/"
fi

cmake \
  -B $build_dir \
  -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
  -DDART_VERBOSE=ON \
  -DDART_TREAT_WARNINGS_AS_ERRORS=ON \
  -DDART_CODECOV=$CODECOV \
  -DDART_IN_CI=$IN_CI \
  ${install_prefix_option}

# Check format
if [ "$CHECK_FORMAT" = "ON" ]; then
  cmake --build $build_dir --target check-format
fi

# DART: build, test, and install
cmake --build $build_dir --target all tests -j$num_threads 
ctest --output-on-failure -j$num_threads --test-dir $build_dir

if [ "$BUILD_EXAMPLES" = "ON" ]; then
  cmake --build $build_dir --target all examples -j$num_threads 
fi

if [ "$BUILD_TUTORIALS" = "ON" ]; then
  cmake --build $build_dir --target all tutorials -j$num_threads 
fi

# dartpy: build, test, and install
if [ "$BUILD_DARTPY" = "ON" ]; then
  cmake --build $build_dir --target dartpy -j$num_threads 
  cmake --build $build_dir --target pytest
fi

cmake --build $build_dir --target install

# Code coverage report generation and upload to codecov.io (only for Linux)
if [ "$CODECOV" = "ON" ]; then

  echo "Info: Code coverage is enabled."

  echo "Downloading codecov script..."
  curl -Os https://uploader.codecov.io/latest/linux/codecov
  chmod +x codecov
  ./codecov --version
  ./codecov -t $CODECOV_TOKEN
  
  echo "Generating code coverage report..."  

  # Capture coverage info
  lcov --capture --directory . --output-file coverage.info
  # Filter out system and extra files.
  # To also not include test code in coverage add them with full path to the patterns: '*/tests/*'
  lcov \
    --remove coverage.info \
      '/usr/*' \
      '*/.deps/*' \
      '*/tests/*' \
      '*/examples/*' \
      '*/tutorials/*' \
    --output-file coverage.info
  # Output coverage data for debugging (optional)
  lcov --list coverage.info

  # Uploading to CodeCov
  echo "Uploading code coverage report to codecov.io..."
  # '-f' specifies file(s) to use and disables manual coverage gathering and file search which has already been done above
  ./codecov -f coverage.info

elif [ "$TEST_INSTALLATION" = "ON" ]; then

  # DART: build an C++ example using installed DART
  echo "Info: Testing the installation..."
  cd $BUILD_DIR/examples/hello_world
  mkdir build && cd build
  cmake ..
  make -j$num_threads

fi

# dartpy: run a Python example using installed dartpy
if [ "$BUILD_DARTPY" = "ON" ]; then
  echo "Info: Running a Python example..."
  echo $PYTHONPATH
  cd $BUILD_DIR/python/examples/hello_world
  python3 main.py
fi

# Clean-up function
cleanup() {
  echo "Cleaning up before exiting..."

  # Cleanup build directories
  rm -rf $build_dir
  find . -type d -name "build" -prune -exec rm -rf {} \;

  # Python temp files
  find . -name "*.pyc" -exec rm -rf {} +
  find . -type d -name ".pytest_cache" -prune -exec rm -rf {} \;
}


