#!/usr/bin/env bash
set -ex

# To resolve 'brew update' error
# - Build log : https://github.com/dartsim/dart/runs/1290172536?check_suite_focus=true#step:3:18
# - References:
#   - https://github.com/osrf/homebrew-simulation/blob/df708147c138d9fef869326caa15aa36918e37cd/.travis.yml#L17-L18
#   - https://github.com/Homebrew/legacy-homebrew/issues/49879#issuecomment-196294820
git -C "$(brew --repo)" reset --hard origin/master
git -C "$(brew --repo)" clean -qxdff

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
