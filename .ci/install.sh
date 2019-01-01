#!/usr/bin/env bash
set -ex

if [ "$OS_NAME" = "linux" ]; then '.ci/install_linux.sh' ; fi
if [ "$OS_NAME" = "osx"   ]; then '.ci/install_osx.sh'   ; fi
