sudo apt-add-repository --yes ppa:libccd-debs/ppa
sudo apt-add-repository --yes ppa:fcl-debs/ppa
sudo apt-add-repository --yes ppa:dartsim/ppa
sudo apt-get -qq update

APT_CORE='
cmake
libassimp-dev
libboost-all-dev
libccd-dev
libeigen3-dev
libfcl-dev
'

APT=$APT_CORE'
freeglut3-dev
libxi-dev
libxmu-dev
libbullet-dev
libflann-dev
libnlopt-dev
coinor-libipopt-dev
libtinyxml-dev
libtinyxml2-dev
liburdfdom-dev
liburdfdom-headers-dev
libopenscenegraph-dev
'

if [ $BUILD_CORE_ONLY = OFF ]; then
  sudo apt-get -qq --yes --force-yes install $APT
else
  sudo apt-get -qq --yes --force-yes install $APT_CORE
fi

sudo apt-get -qq --yes install lcov curl

