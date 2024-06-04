#!/usr/bin/env bash
set -ex

brew update >/dev/null
brew bundle || brew bundle

# OpenSceneGraph
if [ "$INSTALL_OSG_HEAD" = "OFF" ]; then
    brew install open-scene-graph
else
    # Install master branch until 3.7.0 is released (see: https://github.com/dartsim/dart/issues/1439)
    brew install open-scene-graph --HEAD
fi
