cmake_minimum_required(VERSION 2.8.3)
project(charlie_code)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    razor_imu_9dof
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS nodes/interfacer.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*.launch"
)
