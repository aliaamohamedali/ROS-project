#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.cpp"



void cb(const nav_masgs::Odometry& msg){

	static tf:transformBroadcaster broadcaster;

	tf:Transform transform;
	transform.setOrigin(tf::Vector3(msg.twist.linear.x, msg.twist.linear.y, 0.0));
	transform.setRotation(tf::createQuaternionMsgFromYaw(msg->twist.angular.z));

	broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));


}


int main(int argc, char** argv){

	ros::init(argc, argv, "odom_body_broadcatser");
	ros::NodeHandle node;
	ros::subscriber sub = node.subscribe("odom", 100, &cb);	// nav_msgs/Odometry

	ros::spin();
	return 0;

}
