export ROS_MASTER_URI=http://141.219.120.14:11311  //this ensures that we do not use localhost, but the real IP address as master node
export ROS_IP=141.219.120.14   //this ensures that ROS knows that we cannot use hostname directly (due to DHCP firewall issues)
roscore
