#!/usr/bin/env bash
set -ex

sudo -u $(logname) brew update > /dev/null
sudo -u $(logname) brew bundle || sudo -u $(logname) brew bundle

pip3 install -U pytest
