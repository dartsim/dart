make
if [ $BUILD_CORE_ONLY = OFF ]; then make all tests test; fi
# cd tools/
# ./code_check.sh
# ./abi_check.sh 4.1.0  # Check with DART 4.1.0
