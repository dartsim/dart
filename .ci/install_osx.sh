#!/usr/bin/env bash
set -ex

brew update > /dev/null
brew bundle || brew bundle
brew install open-scene-graph --HEAD  # install master branch until 3.7.0 is released

pip3 install -U numpy pytest
