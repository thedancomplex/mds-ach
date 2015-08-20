sudo ls >> /dev/null
#Private repo where code is located
REPO='ln14ds20'
#INSTALL_DIR='/home/lofaro/MDS/'
sudo apt-get update
sudo apt-get install build-essential
sudo apt-get install autoconf automake libtool autoconf-archive
sudo apt-get install ia32-libs ia32-libs-dev libc6-dev-i386
sudo apt-get install ia32-libs 
sudo apt-get install ia32-libs-dev 
sudo apt-get install libc6-dev-i386
sudo apt-get install autoconf automake libtool autoconf-archive help2man man2html
sudo apt-get install libtool
sudo apt-get install dkms
sudo apt-get install screen

sudo echo "deb http://code.golems.org/debian squeeze golems.org" > /etc/apt/sources.list.d/ach-sources.list
sudo apt-get update

sudo apt-get install libach-dev ach-utils ach-dkms
sudo dpkg-reconfigure ach-utils

sudo ln -s /usr/local/lib/libach* /usr/lib/
# Install ACH python bindings
sudo apt-get install python-pip
sudo pip install http://code.golems.org/src/ach/py_ach-latest.tar.gz

