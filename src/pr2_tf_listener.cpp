#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ros/time.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "pr2_tf_listener");

  ros::NodeHandle node;

//  ros::Publisher turtle_vel = node.advertise<turtlesim::Velocity>("pr2/command_velocity", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      ros::Duration(3);
      listener.lookupTransform("gripper1", "base_link",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    

    rate.sleep();
  }
  return 0;
};
