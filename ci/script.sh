if [ "$TRAVIS_PULL_REQUEST" = "false" ] && [ "$COMPILER" = "CLANG" ]; then exit; fi

mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DDART_VERBOSE=ON -DDART_TREAT_WARNINGS_AS_ERRORS=ON -DDART_COVERALLS=$COVERALLS ..
make -j4 tutorials examples tests
if [ $COVERALLS = ON ]; then make -j4 coveralls; else make -j4 test; fi

