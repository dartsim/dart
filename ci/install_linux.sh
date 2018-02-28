sudo apt-add-repository --yes ppa:libccd-debs/ppa
sudo apt-add-repository --yes ppa:fcl-debs/ppa
sudo apt-add-repository --yes ppa:dartsim/ppa
sudo apt-get -qq update

APT='
cmake
libassimp-dev
libboost-all-dev
libccd-dev
libeigen3-dev
libfcl-dev
freeglut3-dev
libxi-dev
libxmu-dev
libbullet-dev
libflann-dev
libnlopt-dev
coinor-libipopt-dev
libtinyxml2-dev
liburdfdom-dev
liburdfdom-headers-dev
libopenscenegraph-dev
clang-format-3.8
lcov
'

sudo apt-get -qq --yes --force-yes install $APT

