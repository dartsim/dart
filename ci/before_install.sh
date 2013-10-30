before_install() {
  cd /tmp
  PROJECTS='tinyxml2 assimp flann libccd fcl'
  for REPO in $PROJECTS
  do
    git clone git://github.com/dartsim/$REPO.git
    (cd $REPO; cmake .; make && sudo make install)
  done
  
  # Set github and bitbucket free of host checking questions
  echo -e "Host github.com\n\tStrictHostKeyChecking no\n" >> ~/.ssh/config
  echo -e "Host bitbucket.org\n\tStrictHostKeyChecking no\n" >> ~/.ssh/config
  # Install console_bridge
  git clone git://github.com/ros/console_bridge.git
  (cd console_bridge; cmake .; make && sudo make install)
  # Install urdfdom_headers
  git clone https://github.com/ros/urdfdom_headers.git
  (cd urdfdom_headers; cmake .; make && sudo make install)
  # Install urdfdom
  git clone https://github.com/ros/urdfdom.git
  (cd urdfdom; cmake .; make && sudo make install)
}

APT='freeglut3 freeglut3-dev libglu1-mesa-dev libboost-all-dev cmake
cmake-curses-gui libeigen3-dev libxmu-dev libxi-dev libwxgtk2.8-dev
libgtest-dev libtinyxml-dev libgtest-dev'

sudo apt-get install $APT
(before_install)
