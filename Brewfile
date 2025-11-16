# Homebrew dependencies for building DART on macOS
#
# NOTE: If you're using pixi, you don't need this file!
# Pixi manages all dependencies automatically via pixi.toml
#
# This Brewfile is for users who want to build DART manually
# without pixi using Homebrew packages.
#
# Usage:
#   brew bundle
#
# For pixi users, instead run:
#   pixi install
#   pixi run build

tap 'osrf/simulation'

# Build dependencies
brew 'cmake'
brew 'ninja'
brew 'pkg-config'
brew 'tracy'

brew 'assimp'
brew 'bullet'
brew 'eigen'
brew 'fcl'
brew 'fmt'
brew 'ipopt'
brew 'nlopt'
brew 'octomap'
# brew 'ode'  # disabled until https://github.com/dartsim/dart/actions/runs/8477285794/job/23227980022 is resolved
# brew 'open-scene-graph'  # disabled until 3.7.0 is released
brew 'pagmo'
brew 'spdlog'
# Replace with the latest sdformat formula from the osrf/simulation tap when available.
brew 'osrf/simulation/sdformat13'
brew 'tinyxml2'
brew 'urdfdom'

# dartpy dependencies
brew 'python3'
