#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "gripper_tf_broadcaster");
  ros::NodeHandle n;

  tf::TransformBroadcaster gripper2_br, gripper1_br;
  tf::Transform gripper2_transform, gripper1_transform;

  ros::Rate rate(100.0);
  while(ros::ok())
  {
      gripper2_transform.setOrigin( tf::Vector3(0.0, 0.00555, 0.173) );
      gripper2_transform.setRotation( tf::Quaternion(-0.26, 0, 0, 1) );
      gripper2_br.sendTransform(tf::StampedTransform(gripper2_transform, ros::Time::now(), "gripper2_flange", "gripper2"));

      gripper1_transform.setOrigin( tf::Vector3(0.0, 0.0, 0.175) );
      gripper1_transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
      gripper1_br.sendTransform(tf::StampedTransform(gripper1_transform, ros::Time::now(), "gripper1_flange", "gripper1"));
  }
  
  rate.sleep();

  return 0;
}
