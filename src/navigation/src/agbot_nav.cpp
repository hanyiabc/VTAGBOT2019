#include "PPController.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
using namespace std;

//define global variables
Point currentPosition;

//callback functions

void pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
{

	//ROS_INFO("UTM_x: %f\tUTM_y: %f", msg->pose.pose.position.x, msg->pose.pose.position.y);

	currentPosition.x = msg->pose.pose.position.x;
	currentPosition.y = msg->pose.pose.position.y;
}

void heading_callback(const geometry_msgs::Point32::ConstPtr &msg)
{

	currentPosition.inputHeading = msg->z;
	//ROS_INFO("x: %f\ty: %f\tz: %f", msg->x, msg->y, msg->z);
	//cout<<currentPosition.inputHeading;
}

//execution function

int execute(int argc, char **agrv, PPController cntrl)
{
	//setup ros publishers and subscribers

	double distance2Goal = 10000000;

	//initialize ppcontroller node
	ros::init(argc, agrv, "ppcontroller");

	ros::NodeHandle nh;

	string file_name;
	nh.param<string>("/waypoint_file_name", file_name, "waypoints1.txt");

	if (!cntrl.initialize(file_name))
	{
		return EXIT_FAILURE;
	}

	auto fix = nh.subscribe("/UTM", 500, pose_callback);
	auto imu = nh.subscribe("/novatel_imu", 500, heading_callback);
	//initialize publishers
	ros::Publisher pub_steering = nh.advertise<std_msgs::Float64>("steering_cmd", 500);
	ros::Publisher pub_padel = nh.advertise<std_msgs::Float64>("speed_setpoint", 500);
	ros::Publisher pub_goal = nh.advertise<geometry_msgs::Point32>("current_goalpoint", 500);

	//initialize
	//1. Parameters (these are used for testing if the error is reducing properly)
	float error = 0;
	float threshold = 2.5;
	double velDouble;
	double deltaDouble;
	std_msgs::Float64 delta;
	std_msgs::Float64 vel;

	//2. Points
	Point goalPoint = cntrl.getwpList()[cntrl.getcurrWpIdx()]; //not to be confused with "current_goalpoint"

	//3. Commands

	geometry_msgs::Point32 command;
	geometry_msgs::Point32 stationaryCommand;
	geometry_msgs::Point32 current_goalPoint;

	stationaryCommand.x = 0;
	stationaryCommand.y = 0;
	cout << "Done Init, begin loop\n";
	cout << "Total points is: " << cntrl.getnPts() << endl;
	//stops when ros is shutdown
	ros::Rate loop_rate(100);
	int start = 0;
	int restart = 0;

	while (ros::ok())
	{
		nh.param<int>("/start_navigation", start, 0);
		if (start == 1)
		{
			//compute the new Euclidean Error
			current_goalPoint.x = goalPoint.x;
			current_goalPoint.y = goalPoint.y;

			cout << "\nCurrent Index: " << cntrl.getcurrWpIdx() << endl;
			pub_goal.publish(current_goalPoint);
			//ROS_INFO("x: %f\ty: %f\t", current_goalPoint.x, current_goalPoint.y);
			cout << "Current Goal Point: " << current_goalPoint.x << "\t" << current_goalPoint.y << "\n";
			//Vehicule is in vicinity of goal point

			cout << "New Goal is:" << goalPoint.x << "\t" << goalPoint.y << endl;

			cntrl.compute_steering_vel_cmds(currentPosition, velDouble, deltaDouble, distance2Goal);
			//Delete THIS!!
			// distance2Goal = 0.1;
			//DELETE THIS!!
			if (distance2Goal < 0.2)
			{
				//Update goal Point to next point in the waypoint list:
				cntrl.incrimentWpIdx();
				cout << "Reached Waypoint # " << cntrl.getcurrWpIdx() << endl;

				//checks to see if there are any more waypoints before updating goalpoint
				if (cntrl.getcurrWpIdx() < cntrl.getnPts())
				{
					goalPoint = cntrl.getwpList()[cntrl.getcurrWpIdx()];
				}
				else
				{
					cout << "\n --- All Waypoints have been conquered! Mission Accomplished Mr Hunt !!! --- " << endl;
					delta.data = 0;
					vel.data = 0;
					pub_steering.publish(delta);
					pub_padel.publish(vel);
					start = 0;
					nh.setParam("/start_navigation", 0);
					distance2Goal = 100000000;
				}
			}
			delta.data = deltaDouble;
			vel.data = velDouble;
			cout << "delta:\t" << delta.data << endl;
			cout << "vel:\t" << vel.data << endl;

			if (start == 1)
			{
				pub_steering.publish(delta);
				pub_padel.publish(vel);
			}
		}
		//ROS_INFO("delta: %f", delta);
		//ROS_INFO("vel: %f", vel);
		nh.param<int>("/reset_navigation", restart, 0);
		if (restart == 1)
		{
			cntrl.resetWpIdx();
			nh.setParam("/reset_navigation", 0);
			nh.param<string>("/waypoint_file_name", file_name, "/home/hanyi/VTAGBOT2019/waypoints1.txt");

			if (!cntrl.initialize(file_name))
			{
				return EXIT_FAILURE;
			}
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

int main(int argc, char **agrv)
{
	//cout << "HI!";
	AckermannVehicle mule = AckermannVehicle(2.065, 4.6, 2.2);
	PPController cntrl = PPController(0, mule.length, mule.minTurningRadius, mule.maximumVelocity);
	//filename selection

	execute(argc, agrv, cntrl);

	return 0;
}
