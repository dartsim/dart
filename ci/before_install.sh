before_install() {
  cd /tmp
  PROJECTS='tinyxml2 assimp flann libccd fcl'
  for REPO in $PROJECTS
  do
    git clone git://github.com/golems/$REPO.git
    (cd $REPO; cmake .; make && sudo make install)
  done
}

APT='freeglut3 freeglut3-dev libglu1-mesa-dev libboost-all-dev cmake
cmake-curses-gui libeigen3-dev libxmu-dev libxi-dev libwxgtk2.8-dev
libgtest-dev libtinyxml-dev libgtest-dev'

sudo apt-get install $APT
(before_install)
