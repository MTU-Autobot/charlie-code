#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::transfrmations::quaternion_from_euler(-1.5707963267948966, 0, -1.7447963267948966), tf::Vector3(0, 0.0, 0.838)),
        ros::Time::now(),"base_link", "zer_wrapper_node"));
    r.sleep();
  }
}
