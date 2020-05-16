#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "cstdlib"

// Runs once at beginning to set the transform between the robot body and camera
// Don;t run if the camera is fixed on the body and its position is set in the urdf tree
	

int main(int argc, char** argv){
	ros::init(argc, argv, "camera_body_tf");
	
	if(argc != 3){
		ROS_INFO("NEED THREE VALUES TO PUBLISH TRANSFORM");
		return 1;
	}

	ros::NodeHandle node;
	tf::TransformBroadcaster broadcaster;
	double x = argv[0], y = argv[1], th = argv[2];


	//since all odometry is 6DOF we'll need a quaternion created from yaw
	//geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
	//first, we'll publish the transform over tf
	tf::Transform transform;
	
	transform.setOrigin(tf::Vector3(x, y, 0.0));
	transform.setRotation(tf::createQuaternionMsgFromYaw(th));

	//send the transform
	broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera"));

	return 0;

}





