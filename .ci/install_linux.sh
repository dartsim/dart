#!/usr/bin/env bash
set -ex

$SUDO apt-add-repository --yes ppa:libccd-debs/ppa
$SUDO apt-add-repository --yes ppa:fcl-debs/ppa
$SUDO apt-add-repository --yes ppa:dartsim/ppa
$SUDO apt-get -qq update

APT='
cmake
libassimp-dev
libboost-filesystem-dev
libboost-regex-dev
libboost-system-dev
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

$SUDO apt-get -qq --yes --force-yes install $APT

if [ $BUILD_NAME = DOCS ]; then
  $SUDO apt-get -qq --yes --force-yes install doxygen
fi
