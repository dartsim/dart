sudo add-apt-repository --yes ppa:libccd-debs/ppa
sudo add-apt-repository --yes ppa:fcl-debs/ppa
sudo add-apt-repository --yes ppa:dartsim/ppa
sudo apt-get update

APT_CORE='
cmake
freeglut3-dev
libassimp-dev
libboost-all-dev
libccd-dev
libeigen3-dev
libfcl-dev
libxi-dev
libxmu-dev 
'

APT=$APT_CORE' 
libflann-dev
libgtest-dev
libtinyxml-dev
libtinyxml2-dev
liburdfdom-dev
liburdfdom-headers-dev
'

if [ $BUILD_CORE_ONLY = OFF ]; then
  sudo apt-get --yes --force-yes install $APT
else
  sudo apt-get --yes --force-yes install $APT_CORE
fi
