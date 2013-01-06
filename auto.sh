#!/bin/sh

PRG=$0
usage()
{
    echo "usage: dart [<args>]" 
    echo ""
    echo "The most commonly used commands are:"
    echo "  merge_graphics  Merges the graphics branch into master"
    echo "  merge_robotics  Merges the robotics branch into master"
    echo "  generate_debs   Generates debian on 32 or 64 bit machines"
    echo "  generate_docs   Generates website documentation"
    echo "  upload_web      Uploads the new website"
    echo ""
    exit 1
}

debs()
{
MACHINE_TYPE=`uname -m`
if [ ${MACHINE_TYPE} == 'x86_64' ]; then
  cpack -D CPACK_SYSTEM_NAME=amd64 -D CPACK_DEBIAN_PACKAGE_ARCHITECTURE=amd64 -G DEB
else
  cpack -D CPACK_SYSTEM_NAME=i386 -D CPACK_DEBIAN_PACKAGE_ARCHITECTURE=i386 -G DEB
fi
}

web()
{
scp *.deb pushkar7@golems.org:~/dart.golems.org/downloads/
cd docs
mv html dart
tar czf dart.tar.gz dart
scp dart.tar.gz pushkar7@dart.golems.org:~/dart.golems.org/
ssh pushkar7@golems.org 'cd dart.golems.org; tar -xvf dart.tar.gz'
}

[ "$#" -lt 1 ] && usage

# parse commandline
while [ $# -gt 0 ]
do
      arg="$1"
      case "$arg" in
         merge_graphics) 
         git checkout master
         git merge graphics
         echo "You are now checked out in the master branch"
         ;;
         merge_robotics) 
         git checkout master
         git merge robotics
         echo "You are now checked out in the master branch"
         ;;
         generate_debs)
         debs    
         ;;
         generate_docs)
         make docs
         ;;
         upload_web)
         web
         ;;
         --) shift; break;;  # no more options
         -*) usage ;; 
         *) break;; # not option, its some argument
       esac
       shift
done
