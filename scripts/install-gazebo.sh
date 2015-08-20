sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install libgdal-dev
sudo apt-get install libsdformat2-dev
sudo apt-get install libbullet2.82-dev
sudo apt-get install libgazebo5
sudo apt-get install gazebo5
sudo apt-get install libgazebo5-dev
gazebo
