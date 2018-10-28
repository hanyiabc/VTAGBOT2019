
#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

void laser_scan_CB(const sensor_msgs::LaserScanConstPtr &msg);
sensor_msgs::PointCloud cloud;
laser_geometry::LaserProjection projector;
//using sensor_msgs::LaserScan;
void laser_scan_CB(const sensor_msgs::LaserScanConstPtr &msg)
{
	projector.projectLaser(*msg, cloud);
	// printf("Start angle: %f\t",msg->angle_min);
	// printf("End angle: %f\t",msg->angle_max);
	// printf("Dist between scan: %f\t",msg->angle_increment);
	// printf("Time between measurement: %f\t",msg->time_increment);
	// printf("Time between scan: %f\t",msg->scan_time);
	// printf("Max range: %f\t",msg->range_max);
	// printf("Min range: %f\t",msg->range_min);
	// printf("\n");
	// for(size_t i = 0; i < msg->ranges.size(); i++)
	// {
	// 	if(msg->ranges[i] > msg->range_min && msg->ranges[i] < msg->range_max)
	// 	{
	// 		printf("Range: %f\t",msg->ranges[i]);
	// 	}
	// }
	// printf("\n");
	// printf("Start angle: %f",msg->angle_min);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "test_lidar_node");
	ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe("/scan", 500, laser_scan_CB);
	ros::Publisher pointCloudPub = n.advertise<sensor_msgs::PointCloud>("/cloud", 100);
	ros::Rate loop_rate(100);
	while (ros::ok())
	{

		pointCloudPub.publish(cloud);

		for (size_t i = 0; i < cloud.points.size(); i++)
		{
			printf("X: %f\t", cloud.points[i].x);
			printf("Y: %f\t", cloud.points[i].y);
			printf("Z: %f\n", cloud.points[i].z);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
