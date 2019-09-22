#!/usr/bin/env bash
set -ex

# Sanity checks for required environment variables.
if [ -z "$BUILD_DARTPY" ]; then
  echo "Info: Environment variable BUILD_DARTPY is unset. Using OFF by default."
  BUILD_DARTPY=OFF
fi

if [ -z "$BUILD_DOCS" ]; then
  echo "Info: Environment variable BUILD_DOCS is unset. Using OFF by default."
  BUILD_DOCS=OFF
fi

if [ -z "$SUDO" ]; then
  if [ -z "${DOCKERFILE}"]; then
    echo "Info: Environment variable SUDO is unset. Using sudo by default."
    SUDO=sudo
  fi
fi

if [ -z "$COMPILER" ]; then
  echo "Info: Environment variable COMPILER is unset. Using gcc by default."
  COMPILER=gcc
fi

$SUDO apt-get -qq update
$SUDO apt-get -y install lsb-release software-properties-common
$SUDO apt-add-repository -y ppa:dartsim/ppa
$SUDO apt-get -qq update

# Build tools
$SUDO apt-get install -y --no-install-recommends \
  sudo \
  build-essential \
  cmake \
  pkg-config \
  curl \
  git
if [ $COMPILER = clang ]; then
  $SUDO apt-get -qq -y install clang
fi

# Required dependencies
$SUDO apt-get install -y --no-install-recommends \
  libassimp-dev \
  libboost-filesystem-dev \
  libboost-system-dev \
  libccd-dev \
  libeigen3-dev \
  libfcl-dev

# Required dependencies for building API documentation of DART < 6.10
$SUDO apt-get install -y --no-install-recommends \
  libboost-regex-dev

# Optional dependencies
$SUDO apt-get install -y --no-install-recommends \
  freeglut3-dev \
  libxi-dev \
  libxmu-dev \
  libbullet-dev \
  liblz4-dev \
  libflann-dev \
  coinor-libipopt-dev \
  libtinyxml2-dev \
  liburdfdom-dev \
  liburdfdom-headers-dev \
  libopenscenegraph-dev
if [ $(lsb_release -sc) = "xenial" ]; then
  $SUDO apt-get -y install libnlopt-dev
  $SUDO apt-get -y install liboctomap-dev libode-dev
elif [ $(lsb_release -sc) = "bionic" ]; then
  $SUDO apt-get -y install libnlopt-dev
  $SUDO apt-get -y install liboctomap-dev libode-dev
  $SUDO apt-get -y install clang-format-6.0
elif [ $(lsb_release -sc) = "cosmic" ]; then
  $SUDO apt-get -y install libnlopt-cxx-dev
  $SUDO apt-get -y install liboctomap-dev libode-dev
elif [ $(lsb_release -sc) = "disco" ]; then
  $SUDO apt-get -y install libnlopt-cxx-dev
  $SUDO apt-get -y install liboctomap-dev libode-dev
elif [ $(lsb_release -sc) = "eoan" ]; then
  $SUDO apt-get -y install libnlopt-cxx-dev
  $SUDO apt-get -y install liboctomap-dev libode-dev
else
  echo -e "$(lsb_release -sc) is not supported."
  exit 1
fi

if [ "$BUILD_DARTPY" = "ON" ]; then
  $SUDO apt-get -y install python3-dev python3-numpy
  $SUDO apt-get -y install python3-pip -y
  $SUDO pip3 install pytest -U

  if [ $(lsb_release -sc) = "xenial" ]; then
    git clone https://github.com/pybind/pybind11 -b 'v2.3.0' --single-branch --depth 1
    cd pybind11
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_TEST=OFF
    make -j4
    $SUDO make install
    cd ../..
  elif [ $(lsb_release -sc) = "bionic" ]; then
    git clone https://github.com/pybind/pybind11 -b 'v2.3.0' --single-branch --depth 1
    cd pybind11
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_TEST=OFF
    make -j4
    $SUDO make install
    cd ../..
  elif [ $(lsb_release -sc) = "cosmic" ]; then
    $SUDO apt-get -y install pybind11-dev python3 libpython3-dev python3-pytest \
      python3-distutils
  elif [ $(lsb_release -sc) = "disco" ]; then
    $SUDO apt-get -y install pybind11-dev python3 libpython3-dev python3-pytest \
      python3-distutils
  elif [ $(lsb_release -sc) = "eoan" ]; then
    $SUDO apt-get -y install pybind11-dev python3 libpython3-dev python3-pytest \
      python3-distutils
  else
    echo -e "$(lsb_release -sc) is not supported."
    exit 1
  fi
fi

$SUDO apt-get install -y --no-install-recommends lcov

if [ $BUILD_DOCS = "ON" ]; then
  $SUDO apt-get install -y --no-install-recommends doxygen
fi
