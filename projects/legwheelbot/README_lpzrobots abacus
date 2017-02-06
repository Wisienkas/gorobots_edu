##After dependencies are installed change the ode_robots-config.m4 file to point to /home/<user>/lib64 instead of using -lm

xutils-dev 
ftp://rpmfind.net/linux/opensuse/distribution/12.1/repo/oss/suse/x86_64/xorg-x11-util-devel-7.6-13.1.2.x86_64.rpm
rpm2cpio to-install.rpm | cpio -idv

libreadline-dev  
ftp://ftp.gnu.org/gnu/readline/readline-6.3.tar.gz
configure --prefix /home/almeh
make
make install

libgsl0-dev 
ftp://ftp.gnu.org/gnu/gsl/gsl-2.3.tar.gz
configure --prefix /home/almeh
make
make install


libglu-dev 
#Should be provided by the mesa below

libgl1-mesa-dev 
ftp://195.220.108.108/linux/sourceforge/m/ma/magicspecs/apt/3.0/x86_64/RPMS.m/mesa-libGL-devel-10.6.3-2.20150729mgc30.x86_64.rpm
rpm2cpio to-install.rpm | cpio -idv


freeglut3-dev  
cmake -DCMAKE_INSTALL_PREFIX=/home/almir
make 
make install

libopenscenegraph-dev 
### CANT USE THIS Missing class GUIEventHandlerVisitor http://trac.openscenegraph.org/downloads/developer_releases/OpenSceneGraph-3.4.0.zip
http://www.openscenegraph.org/downloads/stable_releases/OpenSceneGraph-3.0.1/source/OpenSceneGraph-3.0.1.zip
unzip
cmake -DCMAKE_INSTALL_PREFIX=/home/almir
make
make install

libqt4-develop /// SUSE version
ftp://rpmfind.net/linux/opensuse/update/leap/42.1/oss/x86_64/libqt4-devel-4.8.6-13.1.x86_64.rpm


libqt4-qt3support 
ftp://195.220.108.108/linux/opensuse/update/leap/42.1/oss/x86_64/libqt4-qt3support-4.8.6-13.1.x86_64.rpm

## Did not install below ##
libqt4-dev 
libqt4-opengl 
libqt4-opengl-dev 
gnuplot 
gnuplot-x11 
libncurses5-dev
