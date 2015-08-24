sudo ls >> /dev/null
#Private repo where code is located
REPO='ln14ds20'
#INSTALL_DIR='/home/lofaro/MDS/'
sudo apt-get update
sudo apt-get --force-yes -y install build-essential
sudo apt-get --force-yes -y install autoconf automake libtool autoconf-archive
sudo apt-get --force-yes -y install ia32-libs ia32-libs-dev libc6-dev-i386
sudo apt-get --force-yes -y install ia32-libs 
sudo apt-get --force-yes -y install ia32-libs-dev 
sudo apt-get --force-yes -y install libc6-dev-i386
sudo apt-get --force-yes -y install autoconf automake libtool autoconf-archive help2man man2html
sudo apt-get --force-yes -y install libtool
sudo apt-get --force-yes -y install dkms
sudo apt-get --force-yes -y install screen
sudo apt-get --force-yes -y install python-dev

echo 'deb http://code.golems.org/debian squeeze golems.org' | sudo tee --append /etc/apt/sources.list.d/ach-sources.list
sudo apt-get update

sudo apt-get --force-yes -y install libach-dev ach-utils ach-dkms
#sudo apt-get --force-yes -y install libach*
sudo dpkg-reconfigure ach-utils

sudo ln -s /usr/local/lib/libach* /usr/lib/
# Install ACH python bindings
sudo apt-get --force-yes -y install python-pip
sudo pip install http://code.golems.org/src/ach/py_ach-latest.tar.gz

