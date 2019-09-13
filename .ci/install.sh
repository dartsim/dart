#!/usr/bin/env bash
set -ex

if [ -z "$OS_NAME" ]; then
  echo "Error: Environment variable OS_NAME is unset."
  exit 1
fi

if [ "$OS_NAME" = "linux" ]; then '.ci/install_linux.sh' ; fi
if [ "$OS_NAME" = "osx"   ]; then '.ci/install_osx.sh'   ; fi
