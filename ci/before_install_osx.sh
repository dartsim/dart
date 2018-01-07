brew tap dartsim/dart

brew update > /dev/null

PACKAGES='
git
cmake
assimp
fcl
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
