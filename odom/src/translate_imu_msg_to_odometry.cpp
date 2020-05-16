#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"


double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

double dt;
double delta_x;
double delta_y;
double delta_th;

ros::Time current_time = ros::Time::now();
ros::Time last_time = ros::Time::now();


void cb(const sensor_msgs::Imu& msg){

	dt = (current_time - last_time).toSec();

	// Newton's law
	vx = vx + msg.linear_acceleration.x * dt;
	vy = vy + msg.linear_acceleration.y * dt; 

	delta_x = (vx * cos(th) - vy * sin(th)) * dt;
	delta_y = (vx * sin(th) + vy * cos(th)) * dt;
	delta_th = msg.angular_velocity.z * dt;

	x += delta_x;
	y += delta_y;
	th += delta_th;

}


int main(int argc, char** argv) {

	ros::init(argc, argv, "odom_publisher");
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("mpu_data", 100, &cb);
	ros::Publisher pub = node.advertise<nav_msgs::Odometry>("odom", 100);

	nav_msgs::Odometry odom;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";


    current_time = ros::Time::now();
    last_time = ros::Time::now();
   
    ros::Rate r(1.0);
	
	while(node.ok()){

		ros::spinOnce();               // check for incoming messages
		current_time = ros::Time::now();

		//next, we'll publish the odometry message over ROS
		odom.header.stamp = current_time;
		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;

		//set orientation
		odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);

		//set the velocity
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		//publish the message
		pub.publish(odom);


		last_time = current_time;
		r.sleep()
	}
}
