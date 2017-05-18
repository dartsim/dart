brew tap dartsim/dart
brew tap homebrew/science

brew update > /dev/null

PACKAGES='
git
cmake
assimp
fcl
bullet --with-double-precision
ode --with-libccd --with-double-precision
flann
boost
eigen
tinyxml
tinyxml2
libccd
nlopt
ipopt
ros/deps/urdfdom
ros/deps/urdfdom_headers
ros/deps/console_bridge
open-scene-graph
'

brew install $PACKAGES | grep -v '%$'
