#!/usr/bin/env bash
set -ex

if [ "${OSTYPE//[0-9.]/}" == "linux-gnu" ]; then
  '.ci/install_linux.sh'
elif [ "${OSTYPE//[0-9.]/}" == "darwin"  ]; then
  '.ci/install_osx.sh'
fi
