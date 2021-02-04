//
// Created by hkclr on 2021/1/15.
//

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <aruco_ros/aruco_ros_utils.h>


tf::StampedTransform STtransform_buffer;

//cb for /aruco_tracker/transform. to update the marker's transform
void brMarkerCallback(geometry_msgs::TransformStamped trans)
{
  ROS_WARN("received marker's transform");
  tf::transformStampedMsgToTF(trans, STtransform_buffer);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_broadcaster");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/aruco_tracker/transform", 5, brMarkerCallback);
  static tf::TransformBroadcaster br ;

  ROS_WARN("fake tf_br started");
  //set init tf, br will broadcasting a tf full of zero before it receive the real marker's tf
  tf::Transform trans_init;
  trans_init.setIdentity();
  STtransform_buffer.setData(trans_init);
  STtransform_buffer.frame_id_ = "camera_link";
  STtransform_buffer.child_frame_id_ = "camera_marker";
  STtransform_buffer.stamp_ = ros::Time::now();

  //check for callback and keep broadcasting tf
  while (ros::ok()){
    STtransform_buffer.stamp_ = ros::Time::now();
    br.sendTransform(STtransform_buffer);
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ROS_WARN("broadcasting marker tf");
  }
  return 0;
}