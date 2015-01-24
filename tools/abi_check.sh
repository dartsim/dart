#!/bin/sh

################################################################################
# Functions
################################################################################

# Print usage
usage()
{
    echo "usage: ./abi_check.sh <old_version_number> <new_version_number>" 
    echo ""
    echo "The most commonly used commands are:"
    echo "  ./abi_check.sh 4.0.0 4.1.0  ABI check of 4.0.0 and 4.1.0"
    echo "  ./abi_check.sh 4.0.0        ABI check of 4.0.0 and current"
    echo ""
    exit 1
}

print_valid_version_numbers()
{
    echo "Valid version numbers are: "
    echo "    4.0.0"
    echo "    4.1.0"
    echo "    4.1.1"
    echo "    4.2.0"
    echo "    4.2.1"
    echo "    4.3.0"
    echo "    4.3.1"
    echo "    4.3.2"
    echo "    4.3.3"
    echo "    4.3.4"
    exit 1
}

# Match given version number and corresponding branch name in the dart 
match_version_number_and_branch_name()
{
case "$VERSION_NUMBER" in
        4.0.0)
            BRANCH_NAME="tags/v4.0"
            break;;
        4.1.0)
            BRANCH_NAME="tags/v4.1.0"
            break;;
        4.1.1)
            BRANCH_NAME="tags/v4.1.1"
            break;;
        4.2.0)
            BRANCH_NAME="tags/v4.2.0"
            break;;
        4.2.1)
            BRANCH_NAME="tags/v4.2.1"
            break;;
        4.3.0)
            BRANCH_NAME="tags/v4.3.0"
            break;;
        4.3.1)
            BRANCH_NAME="tags/v4.3.1"
            break;;
        4.3.2)
            BRANCH_NAME="tags/v4.3.2"
            break;;
        4.3.3)
            BRANCH_NAME="tags/v4.3.3"
            break;;
        4.3.4)
            BRANCH_NAME="tags/v4.3.4"
            break;;
        *)
            echo "Invalid version number: ${VERSION_NUMBER}"
            print_valid_version_numbers
            break;;
    esac

    echo "Version number: ${VERSION_NUMBER}"
    echo "Branch name: ${BRANCH_NAME}"
}

# Generate version files
gen_version_files()
{
    FILE="${BASEDIR}/abi_dart_${VERSION_NUMBER}.xml"
    /bin/cat << EOM > $FILE
<version>
    ${VERSION_NUMBER}
</version>

<headers>
    ./install/dart_${VERSION_NUMBER}/include/
</headers>

<libs>
    ./install/dart_${VERSION_NUMBER}/lib/
</libs>
EOM
}

# Clone and build the target version of DART
build_target_version_dart()
{
cd $BASEDIR/source
git clone git://github.com/dartsim/dart.git dart_${VERSION_NUMBER}
cd dart_${VERSION_NUMBER}
git checkout ${BRANCH_NAME}
cmake -DBUILD_CORE_ONLY=$BUILD_CORE_ONLY -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_INSTALL_PREFIX=${BASEDIR}/install/dart_${VERSION_NUMBER} .
make -j4 install
}

# Build the current version of DART
build_current_dart()
{
if [ ! -d "$BASEDIR/source/dart_current" ]; then
    mkdir $BASEDIR/source/dart_current 
fi
cd $BASEDIR/source/dart_current
cmake -DBUILD_CORE_ONLY=$BUILD_CORE_ONLY -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_INSTALL_PREFIX=${BASEDIR}/install/dart_current ${CURRENT_DART_DIR}
make -j4 install
}

################################################################################
# Main routine 
################################################################################

# If the number of arguments is not 1 or 2, then print usage
[ "$#" -gt 2 ] && { usage; exit 1; }
[ "$#" -lt 1 ] && { usage; exit 1; }

# Set working directory
BASEDIR="$PWD"/abi_check_work
cd ../
CURRENT_DART_DIR=$PWD
echo Current DART directory: $CURRENT_DART_DIR
echo ABI check working directory: $BASEDIR

# Create directories
if [ ! -d "$BASEDIR" ]; then
    mkdir $BASEDIR;
fi
if [ ! -d "$BASEDIR/source" ]; then
    mkdir $BASEDIR/source; 
fi
if [ ! -d "$BASEDIR/install" ]; then
    mkdir $BASEDIR/install 
fi

# Set variables
PRG=$0
OLD_VER=$1
if [ "$#" -eq 2 ]; then
    NEW_VER=$2
else
    NEW_VER=current
fi

# Build and install the old version of DART
VERSION_NUMBER=$OLD_VER
match_version_number_and_branch_name
gen_version_files
build_target_version_dart

# Build and install the new version of DART
if [ "$#" -eq 2 ]; then
    VERSION_NUMBER=$NEW_VER
    match_version_number_and_branch_name
    gen_version_files
    build_target_version_dart
else
    VERSION_NUMBER=current
    gen_version_files
    build_current_dart
fi

# Install ABI checker
sudo apt-get --yes --force-yes install abi-compliance-checker

# Checkk ABI
cd $BASEDIR
abi-compliance-checker -lib DART -old abi_dart_${OLD_VER}.xml -new abi_dart_${NEW_VER}.xml


