cd lib_run
pip install --upgrade pip
pip install -r requirements.txt
cd scripts
. create_udev_rules
sudo apt install -y ros-$ROS_DISTRO-navigation ros-$ROS_DISTRO-amcl ros-$ROS_DISTRO-laser-filters ros-$ROS_DISTRO-hls-lfcd-lds-driver
