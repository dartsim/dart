#!/usr/bin/env bash
set -ex

$SUDO apt-get -qq update
$SUDO apt-get -y install lsb-release software-properties-common
$SUDO apt-add-repository -y ppa:dartsim/ppa
$SUDO apt-get -qq update

# Build tools
$SUDO apt-get -y install \
  build-essential \
  cmake \
  curl \
  git \
  pkg-config \
  sudo

if [ $MEMCHECK = "ON" ]; then
  if [ $(lsb_release -sc) = "xenial" ]; then
    $SUDO apt-get -y install wget
    wget http://valgrind.org/downloads/valgrind-3.14.0.tar.bz2
    tar -xjf valgrind-3.14.0.tar.bz2
    cd valgrind-3.14.0
    ./configure --prefix=/usr
    make
    sudo make install
  else
    $SUDO apt-get -y install valgrind
  fi
fi

if [ $COMPILER = clang ]; then
  $SUDO apt-get -qq -y install clang
fi

# Required dependencies
$SUDO apt-get -y install \
  libassimp-dev \
  libboost-filesystem-dev \
  libboost-regex-dev \
  libboost-system-dev \
  libccd-dev \
  libeigen3-dev \
  libfcl-dev

# Optional dependencies
$SUDO apt-get -y install \
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
    git clone https://github.com/pybind/pybind11 -b 'v2.2.4' --single-branch --depth 1
    cd pybind11
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_TEST=OFF
    make -j4
    $SUDO make install
    cd ../..
  elif [ $(lsb_release -sc) = "bionic" ]; then
    git clone https://github.com/pybind/pybind11 -b 'v2.2.4' --single-branch --depth 1
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

$SUDO apt-get -y install lcov

if [ $BUILD_DOCS = "ON" ]; then
  $SUDO apt-get -qq -y install doxygen
fi
