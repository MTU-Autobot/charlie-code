#!/bin/sh

roslaunch zed_wrapper zed.launch | tee 1.log | sed -e 's/^/[Zed wrapper] /' &

sleep 12

rosrun tf static_transform_publisher 0 0 0.838 -1.5707963267948966 0 -1.7447963267948966 camera_link zed_initial_frame 100 | tee 2.log | sed -e 's/^/[Transform] /' &

sleep 1

roslaunch rtabmap_ros rgbd_mapping.launch rtabmap_args:="--delete_db_on_start" depth_registered_topic:=/camera/depth/image_rect_color rviz:=true rtabmapviz:=false | tee 3.log | sed -e 's/^/[RTAB-MAP] /'
