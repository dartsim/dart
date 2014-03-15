before_install() {
  cd /tmp

  # Install nlopt from source since Ubuntu 12.04 does not provide debian package for nlopt
  curl -o nlopt-2.4.1.tar.gz http://ab-initio.mit.edu/nlopt/nlopt-2.4.1.tar.gz
  tar -xf nlopt-2.4.1.tar.gz
  (cd nlopt-2.4.1/; sh autogen.sh; make CPPFLAGS='-fPIC' && sudo make install)
}

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

(before_install)

