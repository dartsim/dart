brew update > /dev/null
brew bundle --file="$TRAVIS_BUILD_DIR/ci/Brewfile"
