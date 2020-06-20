#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "cstdlib"

// Runs once at beginning to set the transform between the robot body and camera
// Don;t run if the camera is fixed on the body and its position is set in the urdf tree
	

int main(int argc, char** argv){
	ros::init(argc, argv, "map_odom_tf");
	
	if(argc != 4){
		ROS_INFO("NEED THREE VALUES TO PUBLISH TRANSFORM");
		return 1;
	}

	ros::NodeHandle node;
	tf::TransformBroadcaster broadcaster;	
	tf::Transform transform;
	tf::Quaternion quat;

	double x = atof(argv[1]);
	double y = atof(argv[2]);
	double th = atof(argv[3]);


	//since all odometry is 6DOF we'll need a quaternion created from yaw
	//geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
	//first, we'll publish the transform over tf

    ros::Rate rate(0.2);
    while (node.ok()){
		transform.setOrigin(tf::Vector3(x, y, 0.0));
		quat.setRPY(0, 0, th);
		transform.setRotation(quat);

		//send the transform
		broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
		rate.sleep();
  	}

	return 0;

}
