#!/bin/sh

# Generate a report about the system and put it in a bug.txt file
# Users can send the bug.txt to developers for additional debugging

rm -f bug.txt
echo `uname -a` >> bug.txt
echo `gcc --version` >> bug.txt
echo `cmake --version` >> bug.txt
echo `dpkg -s libboost-dev | grep Package` >> bug.txt
echo `dpkg -s libboost-dev | grep Version` >> bug.txt
echo `dpkg -s libboost-dev | grep Status` >> bug.txt
echo `dpkg -s libflann-dev | grep Package` >> bug.txt
echo `dpkg -s libflann-dev | grep Version` >> bug.txt
echo `dpkg -s libflann-dev | grep Status` >> bug.txt

echo "Send the generated bug.txt to the developer\n"

