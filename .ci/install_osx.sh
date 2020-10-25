#!/usr/bin/env bash
set -ex

# Temporary workaround for https://github.com/actions/virtual-environments/issues/1811
brew untap local/homebrew-openssl
brew untap local/homebrew-python2

brew update > /dev/null
brew bundle || brew bundle

# OpenSceneGraph
brew install open-scene-graph --HEAD # install master branch until 3.7.0 is released

# pagmo2: build from source until https://github.com/esa/pagmo2/issues/445 is resolved
brew install tbb
git clone https://github.com/esa/pagmo2.git -b 'v2.15.0' --single-branch --depth 1 &&
  cd pagmo2 && mkdir build && cd build &&
  cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DPAGMO_WITH_EIGEN3=ON &&
  # NLopt and IPopt supports are disabled for https://github.com/esa/pagmo2/issues/445
  # -DPAGMO_WITH_NLOPT=ON \
  # -DPAGMO_WITH_IPOPT=ON \
  make -j$(sysctl -n hw.logicalcpu) &&
  make install

# Use pip for the default Python3 version
py_version=$(python3 -c "import sys; print('{}.{}'.format(sys.version_info[0], sys.version_info[1]))")
pip$py_version install -U numpy pytest
