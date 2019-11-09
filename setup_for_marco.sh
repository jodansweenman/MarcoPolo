
export ROS_MASTER_URI=http://192.168.1.101:11311
export ROS_IP=$(ip -f inet addr show wlp3s0 | grep -Po 'inet \K[\d.]+')
