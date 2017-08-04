if [ "$TRAVIS_PULL_REQUEST" = "false" ] && [ "$TRAVIS_OS_NAME" = "linux" ] && [ "$COMPILER" = "CLANG" ]; then exit; fi

mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DDART_VERBOSE=ON -DDART_TREAT_WARNINGS_AS_ERRORS=ON -DDART_CODECOV=$CODECOV ..
if [ "$TRAVIS_OS_NAME" = "linux" ]; then make -j4 tutorials examples tests; else make -j4 tests; fi
if [ "$TRAVIS_OS_NAME" = "linux" ]; then make check-format; fi
if [ $CODECOV = ON ]; then make -j4 codecov; else make -j4 test; fi
