#!/bin/sh

PRG=$0
usage()
{
    echo "usage: ./auto.sh [<args>]" 
    echo ""
    echo "The most commonly used commands are:"
    echo "  debs <precise/quantal>  Generates debian on 32 or 64 bit machines"
    echo "  docs <doxygen>          Generates website documentation"
    echo ""
    exit 1
}

debs()
{
echo Building deb for $1
if [ "$1" = "quantal" ]; then
  echo "quantal deb uses libboost1.49"
  sed -i 's/precise/quantal/g' CMakeLists.txt 
  sed -i 's/1.46/1.49/g' CMakeLists.txt
  cmake .
  cpack -G DEB
else
  echo "precise deb uses libboost1.46"
  cpack -G DEB
fi
}

[ "$#" -lt 2 ] && usage

# parse commandline
while [ $# -gt 0 ]
do
      arg="$1"
      sys="$2"
      case "$arg" in
         debs)
         debs $sys
         ;;
         docs)
         make docs
         ;;
         --) shift; break;;  # no more options
         -*) usage ;; 
         *) break;; # not option, its some argument
       esac
       shift
done
