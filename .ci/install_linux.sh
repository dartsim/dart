#!/usr/bin/env bash
set -ex

# TODO(JS): Remove this file once CODECOV and gh-page migrated to the Docker builds

# Sanity checks for required environment variables.
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

apt-get -qq update
apt-get -y install lsb-release software-properties-common
apt-add-repository -y ppa:dartsim/ppa
apt-get -qq update

# Build tools
apt-get install -y --no-install-recommends \
  sudo \
  build-essential \
  cmake \
  pkg-config \
  curl \
  git
if [ $COMPILER = clang ]; then
  apt-get -qq -y install clang
fi

# Required dependencies
apt-get install -y --no-install-recommends \
  libassimp-dev \
  libboost-filesystem-dev \
  libboost-system-dev \
  libccd-dev \
  libeigen3-dev \
  libfcl-dev

# Required dependencies for building API documentation of DART < 6.10
apt-get install -y --no-install-recommends \
  libboost-regex-dev

# Optional dependencies
apt-get install -y --no-install-recommends \
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
if [ $(lsb_release -sc) = "bionic" ]; then
  apt-get install -y --no-install-recommends \
    libnlopt-dev \
    liboctomap-dev \
    libode-dev \
    clang-format-6.0
elif [ $(lsb_release -sc) = "focal" ]; then
  apt-get install -y --no-install-recommends \
    libnlopt-cxx-dev \
    liboctomap-dev \
    libode-dev \
    clang-format-6.0
elif [ $(lsb_release -sc) = "groovy" ]; then
  apt-get install -y --no-install-recommends \
    libnlopt-cxx-dev \
    liboctomap-dev \
    libode-dev
else
  echo -e "$(lsb_release -sc) is not supported."
  exit 1
fi

# pagmo2
apt-get install -y --no-install-recommends \
  libboost-serialization-dev \
  libtbb-dev
git clone https://github.com/esa/pagmo2.git -b 'v2.15.0' --single-branch --depth 1 &&
  cd pagmo2 && mkdir build && cd build &&
  cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DPAGMO_WITH_EIGEN3=ON &&
  # NLopt and IPopt supports are disabled until https://github.com/esa/pagmo2/issues/445 is resolved
  # -DPAGMO_WITH_NLOPT=ON \
  # -DPAGMO_WITH_IPOPT=ON \
  make -j$(nproc) &&
  make install

if [ "$BUILD_DARTPY" = "ON" ]; then
  apt-get install -y --no-install-recommends \
    python3-dev \
    python3-numpy \
    python3-pip \
    python3-setuptools
  pip3 install pytest -U

  if [ $(lsb_release -sc) = "bionic" ]; then
    git clone https://github.com/pybind/pybind11 -b 'v2.3.0' --single-branch --depth 1 &&
      cd pybind11 && mkdir build && cd build &&
      cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DPYBIND11_TEST=OFF &&
      make -j$(nproc) &&
      make install
  elif [ $(lsb_release -sc) = "focal" ] || [ $(lsb_release -sc) = "groovy" ]; then
    apt-get -y install pybind11-dev python3 libpython3-dev python3-pytest \
      python3-distutils
  else
    echo -e "$(lsb_release -sc) is not supported."
    exit 1
  fi
fi

apt-get install -y --no-install-recommends lcov

if [ $BUILD_DOCS = "ON" ]; then
  apt-get install -y --no-install-recommends doxygen
fi
