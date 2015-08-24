sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get --force-yes -y install libgdal-dev
sudo apt-get --force-yes -y install libsdformat2-dev
sudo apt-get --force-yes -y install libbullet2.82-dev
sudo apt-get --force-yes -y install libgazebo5
sudo apt-get --force-yes -y install gazebo5
sudo apt-get --force-yes -y install libgazebo5-dev
gazebo
