# pr2_tf

## environment

* ROS: groovy rosbuild
* robot: PR2

## functionality

Traditional ICP is not good enough because all the source point cloud don't share the positon related to world coordinate. Our method is to use robot joint transformations to get the source point cloud according to the world coordinate and it's much more easier to use ICP.

* frame_tf_broadcaster.cpp: define the transformation of kinect on the left arm of pr2 and broadcast it to tf

* simple_point_cloud_analyzer.cpp: get the positon of source point cloud in the world coordinate(odom_combined used on pr2) with handy kinect and use ICP to register all the point cloud into a scene


