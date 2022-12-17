#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>

#include "parameters.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/PinholeCamera.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv, "my_static_tf2_broadcaster");

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  ROS_INFO("Creating stamp, %s", "d1_camera");

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "world";
  static_transformStamped.child_frame_id = "d1_camera";
  static_transformStamped.transform.translation.x = 0.0f;
  static_transformStamped.transform.translation.y = 0.0f;
  static_transformStamped.transform.translation.z = 0.0f;
  tf2::Quaternion quat;
  quat.setRPY(0.0f, 0.0f, 0.0f);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();

  ROS_INFO("Sending stamp");
  static_broadcaster.sendTransform(static_transformStamped);
  // ROS_INFO("Spinning until killed publishing %s to world", frame_name);
  
  ROS_INFO("Spinnin'");
  ros::spin();
  return 0;
};