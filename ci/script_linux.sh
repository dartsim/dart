make
if [ $COVERALLS = ON ]; then make coveralls; fi
sudo ldconfig --verbose # So the test executeables can detect libtinyxml2
make test

