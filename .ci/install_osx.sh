#!/usr/bin/env bash
set -ex

brew update > /dev/null
brew bundle || brew bundle

pip3 install -U pytest
