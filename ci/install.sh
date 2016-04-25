if [ "$TRAVIS_PULL_REQUEST" = "false" ] && [ "$COMPILER" = "CLANG" ]; then exit; fi

if [ "$TRAVIS_OS_NAME" = "linux" ]; then 'ci/install_linux.sh' ; fi
if [ "$TRAVIS_OS_NAME" = "osx"   ]; then 'ci/install_osx.sh'   ; fi

