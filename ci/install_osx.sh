brew update > /dev/null
brew tap dartsim/dart
brew tap ros/deps
brew bundle --file="$TRAVIS_BUILD_DIR/ci/Brewfile"
