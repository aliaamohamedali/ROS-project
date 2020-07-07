#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "cstdlib"

// Runs once at beginning to set the transform between the robot body and camera
// Don;t run if the camera is fixed on the body and its position is set in the urdf tree
	

int main(int argc, char** argv){
	ros::init(argc, argv, "odom_map_tf");

	ros::NodeHandle node;
	tf::TransformBroadcaster broadcaster;	
	tf::Transform transform;
	tf::Quaternion quat;

	double x = 0;
	double y = 0;
	double th = 0;


	//since all odometry is 6DOF we'll need a quaternion created from yaw
	//geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
	//first, we'll publish the transform over tf

    ros::Rate rate(0.2);
    while (node.ok()){
		transform.setOrigin(tf::Vector3(x, y, 0.0));
		quat.setRPY(0, 0, th);
		transform.setRotation(quat);

		//send the transform
		broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera"));
		rate.sleep();
  	}

	return 0;

}





