brew tap dartsim/dart
brew tap homebrew/science

brew update > /dev/null

PACKAGES='
git
cmake
assimp
fcl03
bullet
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
ros/deps/gtest
'

brew install $PACKAGES | grep -v '%$'
