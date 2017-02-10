#!/bin/bash

## Veryfi that this script is being sourced rather than spawned in a sub-shell
## This is required in order to setup environment correctly

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]
then
    	echo "ERROR: Script is not being sourced ..."
        echo "You need to source this script in order for it to setup correct environment"
        echo 'Eg. run the command like this "source '"$(basename $BASH_SOURCE)"'"'
        exit 1
fi

# Load required modules for building dependencies and lpzrobots
modulesToLoad='module add java-oracle gcc openmpi cmake'
eval "$modulesToLoad"

# Make modules load on logon # Comment this out if you dont want it, and remove the $modulesToLoad line from your ~/bashrc
grep -q "$modulesToLoad" "$HOME/.bashrc" || echo "$modulesToLoad" >> "$HOME/.bashrc"

# Make directory where we should download all dependecies
mkdir -p ~/lpz_dependencies/download ~/bin

dependencies=(
    # xutils-dev
    ftp://rpmfind.net/linux/opensuse/distribution/12.1/repo/oss/suse/x86_64/xorg-x11-util-devel-7.6-13.1.2.x86_64.rpm

    # libqt4-develop
    ftp://rpmfind.net/linux/opensuse/update/leap/42.1/oss/x86_64/libqt4-devel-4.8.6-13.1.x86_64.rpm

    # libqt4-qt3support
    ftp://195.220.108.108/linux/opensuse/update/leap/42.1/oss/x86_64/libqt4-qt3support-4.8.6-13.1.x86_64.rpm

    # libglu-dev and libgl1-mesa-dev
    ftp://195.220.108.108/linux/sourceforge/m/ma/magicspecs/apt/3.0/x86_64/RPMS.m/mesa-libGL-devel-10.6.3-2.20150729mgc30.x86_64.rpm

    # libreadline-dev
    ftp://rpmfind.net/linux/fedora-secondary/development/rawhide/Everything/aarch64/os/Packages/r/readline-devel-7.0-4.fc26.aarch64.rpm

    # libgsl0-dev --> WE NEED BOTH BELOW Packages
    ftp://rpmfind.net/linux/fedora/linux/releases/24/Everything/x86_64/os/Packages/g/gsl-2.1-4.fc24.x86_64.rpm
    ftp://rpmfind.net/linux/fedora-secondary/releases/25/Everything/aarch64/os/Packages/g/gsl-devel-2.1-4.fc25.aarch64.rpm

    # freeglut3-dev
    ftp://195.220.108.108/linux/fedora-secondary/releases/25/Everything/aarch64/os/Packages/f/freeglut-devel-3.0.0-3.fc24.aarch64.rpm

    # libopenscenegraph-dev
    # NOTE can not use newer version than 3.0.1 because some classes required by lpz are removed in newer versions
    http://www.openscenegraph.org/downloads/stable_releases/OpenSceneGraph-3.0.1/source/OpenSceneGraph-3.0.1.zip

    # NOTE THAT FOLLOWING ARE LISTED BY LPZ AS DEPENDENCIES BUT ARE NOT REQUIRED FOR RUNNING SIMULATIONS WITHOUT GFX ON ABACUS. THOSE WILL NOT BE INSTALLED BY THIS SCRIPT
    # gnuplot
    # gnuplot-x11
    # libncurses5-dev
)

# Download all dependencies
wget -nc -P ~/lpz_dependencies/download/ "${dependencies[@]}"

# First extract all rpm's
for rpm in ~/lpz_dependencies/download/*.rpm; do rpm2cpio $rpm | ( cd ~/lpz_dependencies/ && cpio -idv ); done

# Now install the openscenegraph library from source
cd ~/lpz_dependencies/download && unzip OpenSceneGraph-3.0.1.zip -d .
cd ~/lpz_dependencies/download/OpenSceneGraph-3.0.1 && cmake -DCMAKE_INSTALL_PREFIX="$HOME"/lpz_dependencies/usr
cd ~/lpz_dependencies/download/OpenSceneGraph-3.0.1 && make -j48
cd ~/lpz_dependencies/download/OpenSceneGraph-3.0.1 && make install -j48

# Move everything from ~/lpz_dependencies/usr/bin to $HOME/bin as this is already on the PATH
cd ~/lpz_dependencies/usr/bin && find -type f -print0 | xargs -0 -n 1 -I {} mv '{}' "$HOME"'/bin/{}'

# These lines will be added to the ~/.bashrc script if they are not already present
paths=(
    'export LD_LIBRARY_PATH=~/lpz_dependencies/usr/lib64:~/workspace/pmanoonpong-lpzrobots-fork/opende/ode/src/.libs'
    'export QMAKESPEC=/lib64/qt4/mkspecs/linux-g++'
    'export CPATH=~/lpz_dependencies/usr/include'
)
for path in "${paths[@]}" ; do
    grep -q "$path" "$HOME/.bashrc" || echo "$path" >> "$HOME/.bashrc"
    eval "$path"
done

"DONE!"