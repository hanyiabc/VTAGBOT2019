#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <std_msgs/String.h>

using namespace actionlib;
using namespace move_base_msgs;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

visualization_msgs::Marker markerPath;

enum ACTION_TYPE {UNINIT, START, STOP, RESET, WAITING} currAction;

void action_cb(const std_msgs::String::ConstPtr &msg)
{
	if (msg->data == "Reset")
	{
		currAction = RESET;
	}
	else if (msg->data == "Start")
	{
		currAction = START;
	}
	else if (msg->data == "Stop")
	{
		currAction = STOP;
	}
	
}

void marker_cb(const visualization_msgs::Marker::ConstPtr &msg)
{
	markerPath = *msg;
}

void doneCb(const SimpleClientGoalState & state, const MoveBaseActionConstPtr & result)
{
	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "wp_publisher");
	ros::NodeHandle n;
    auto pt_sub = n.subscribe("wp_markers", 5, marker_cb);
    auto actionSub = n.subscribe("agbot_action", 5, action_cb);

	MoveBaseClient ac("move_base", true);
	while (!ac.waitForServer(ros::Duration(5.0)))
	{
	}


	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.pose.position.x = 1.0;
	goal.target_pose.pose.orientation.w = 1.0;

	while (ros::ok())
    {
		static size_t idx = 0;
		switch (currAction)
		{
		case START:
			if(idx < markerPath.points.size())
			{
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position = markerPath.points[idx];
				ac.sendGoal(goal);
				currAction = WAITING;
			}
			
			break;
		case STOP:

			break;
		case RESET:
			break;
		case WAITING:
			if(ac.waitForResult(ros::Duration(0.1)))
			{
				idx++;
				ROS_INFO("Goal Reached, switch to next goal if available");
				currAction = START;
			}
			break;
		default:
			break;
		}
        ros::spinOnce();
    }


	//ac.waitForResult();
}

