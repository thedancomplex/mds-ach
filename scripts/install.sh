sudo ls >> /dev/null
#Private repo where code is located
REPO='ln14ds20'
#INSTALL_DIR='/home/lofaro/MDS/'
sudo apt-get update
sudo apt-get install build-essential
sudo apt-get install autoconf automake libtool autoconf-archive
sudo apt-get install ia32-libs ia32-libs-dev libc6-dev-i386
sudo apt-get install autoconf automake libtool autoconf-archive help2man man2html
sudo apt-get install libtool
sudo apt-get install dkms
mkdir tmp
cd tmp
TMP_DIR="$PWD"
sudo rm -rf ach
git clone http://$REPO/mds/ach.git
cd ach
autoreconf -i
autoreconf -i
autoreconf -i
./configure
./configure
./configure
make
sudo make install
cd $TMP_DIR
cd ..
sudo rm -rf $TMP_DIR
sudo ln -s /usr/local/lib/libach* /usr/lib/
# Install ACH python bindings
sudo apt-get install python-pip
sudo pip install http://code.golems.org/src/ach/py_ach-latest.tar.gz


# For utilities
#sudo apt-get install python-pygame
