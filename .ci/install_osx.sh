#!/usr/bin/env bash
set -ex

brew update > /dev/null
brew bundle || brew bundle

# OpenSceneGraph
# install master branch until 3.7.0 is released
brew install open-scene-graph --HEAD

# Use pip for the default Python3 version
py_version=$(python3 -c "import sys; print('{}.{}'.format(sys.version_info[0], sys.version_info[1]))")
py_version_major=`echo $py_version | cut -d. -f1`
pip$py_version_major install -U numpy pytest
