#!/usr/bin/env bash
set -ex

brew update > /dev/null
brew bundle
brew install dartsim/dart/filament
