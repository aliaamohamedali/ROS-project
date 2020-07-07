#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"



tf::TransformBroadcaster* bc_ptr;
tf::Transform* tf_ptr;
tf::Quaternion* quat_ptr;

void cb(const nav_msgs::Odometry& msg){

	tf_ptr->setOrigin(tf::Vector3(msg.twist.twist.linear.x, msg.twist.twist.linear.y, 0.0));
	quat_ptr->setRPY(0, 0, msg.twist.twist.angular.z);
	tf_ptr->setRotation(*quat_ptr);

	bc_ptr->sendTransform(tf::StampedTransform(*tf_ptr, ros::Time::now(), "odom", "base_link"));


}


int main(int argc, char** argv){

	ros::init(argc, argv, "odom_body_tf");
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("odom", 100, &cb);	// nav_msgs/Odometry

	tf::TransformBroadcaster broadcaster;
	tf::Transform transform;
	tf::Quaternion quat;

	bc_ptr = &broadcaster;
	tf_ptr = &transform;
	quat_ptr = &quat;

	transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	quat.setRPY(0, 0, 0);
	transform.setRotation(quat);

	broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

	ros::spin();
	return 0;

}
