#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "vector"





int main(int argc, char** argv) {


	ros::init(argc, argv, "laser_scan_mock");
	ros::NodeHandle node;
	ros::Publisher pub = node.advertise<sensor_msgs::LaserScan>("lidar", 100);
	sensor_msgs::LaserScan lidar;

	int num_readings = 100;
	int laser_frequency = 40;

	int count = 0;

	std::vector<float> ranges(100, 0);
	std::vector<float> intensities(100, 0);

	// Set LaserScan Definition
    lidar.header.frame_id = "lidar";
    lidar.angle_min = -1.57;											// start angle of the scan [rad]	
    lidar.angle_max = 1.57;												// end angle of the scan [rad]
    lidar.angle_increment = 3.14 / num_readings;						// angular distance between measurements [rad]
    lidar.time_increment = (1.0 / laser_frequency) / (num_readings); 
    lidar.range_min = 0.0;												// minimum range value [m]
    lidar.range_max = 100.0;											// maximum range value [m]

	
	lidar.ranges = ranges;
	lidar.intensities = intensities;


	ros::Rate r(1.0);

	while(node.ok()){
		lidar.header.stamp = ros::Time::now();
		
		for(int i=0; i<100; i++){
			lidar.ranges[i] = 1.0 * count;
			lidar.intensities[i] = 1;
		}

		pub.publish(lidar);
		count++;
		r.sleep();

	}










}
