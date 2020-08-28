#!/usr/bin/env bash
set -ex

brew update > /dev/null
brew bundle || brew bundle
brew install open-scene-graph --HEAD  # install master branch until 3.7.0 is released

# pagmo2: build from source until https://github.com/esa/pagmo2/issues/445 is resolved
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

pip3 install -U numpy pytest
