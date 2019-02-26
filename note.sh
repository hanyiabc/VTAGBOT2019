#!/bin/bash

#get simulation working
#need root
sudo su
#change to home
cd ~
#clone the repo
git clone https://github.com/hanyiabc/VTAGBOT2019.git

#add environment variable
sudo echo "export GAZEBO_PLUGIN_PATH=$(pwd)"'/VTAGBOT2019/src/atvsim_description/Plugins:${GAZEBO_PLUGIN_PATH}' >> /usr/share/gazebo/setup.sh
sudo echo "export GAZEBO_MODEL_PATH=$(pwd)"'/VTAGBOT2019/src/atvsim_description/models:${GAZEBO_MODEL_PATH}' >> /usr/share/gazebo/setup.sh


#in replace all "hongxu" with your username 
#running this is assuming you put the repo in home
sed -i "s|hongxu|$USER|g" ./VTAGBOT2019/src/atvsim_gazebo/worlds/drillfieldTest.world

#install the necessar deps
sudo apt install -y ros-melodic-joy ros-melodic-ros-control ros-melodic-gazebo-ros-control ros-melodic-hector-gazebo ros-melodic-gps-common
sudo apt install -y ros-melodic-robot-state-publisher ros-melodic-media-export qt4-default ros-melodic-tf-conversions

cd VTAGBOT2019
catkin_make
source ./devel/setup.bash

echo "source ~/VTAGBOT2019/devel/setup.bash" >> ~/.bashrc
#rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map hokuyo_link 10
#rosrun rviz rviz -f hokuyo_link