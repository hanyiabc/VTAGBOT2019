#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

visualization_msgs::Marker markerPath;
visualization_msgs::Marker markerPt;
nav_msgs::Path wp_path;
geometry_msgs::Pose poseData;

geometry_msgs::Quaternion orientationBetween(geometry_msgs::Point pt1, geometry_msgs::Point pt2)
{
	double x_d = pt2.y - pt1.x;
	double y_d = pt2.y - pt1.y;
	double angle = atan2(x_d, y_d);
	angle += M_PI / 2;
	//angle = -angle;
	tf::Quaternion q;
	q.setRPY(0.0, 0.0, angle);
	geometry_msgs::Quaternion gq;
	gq.w = q.getW();
	gq.x = q.getX();
	gq.y = q.getY();
	gq.z = q.getZ();
	return gq;
}

void action_cb(const std_msgs::String::ConstPtr &msg)
{
	if (msg->data == "Reset")
	{
		markerPath.points.clear();
		markerPt.points.clear();
		wp_path.poses.clear();
	}
	
}

void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	poseData = msg->pose.pose;
}

void pt_cb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
	markerPath.points.push_back(msg->point);
	markerPt.points.push_back(msg->point);
	geometry_msgs::PoseStamped currPose;
	currPose.header.frame_id="map";
	currPose.pose.position = msg->point;
	currPose.pose.orientation.w = 1.0;
	wp_path.poses.push_back(currPose);
	if(wp_path.poses.size() >= 2)
	{
		auto q = orientationBetween(wp_path.poses[wp_path.poses.size() - 2].pose.position, wp_path.poses.back().pose.position);;
		wp_path.poses[wp_path.poses.size() - 1].pose.orientation = q;
	}
	// auto q = orientationBetween(poseData.position, msg->point);
	// wp_path.poses[wp_path.poses.size() - 1].pose.orientation = q;
	else if(wp_path.poses.size() == 1)
	{
		auto q = orientationBetween(poseData.position, msg->point);
		wp_path.poses[wp_path.poses.size() - 1].pose.orientation = q;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_viz_and_receiver");
	ros::NodeHandle n;
    auto pt_sub = n.subscribe("clicked_point", 5, pt_cb);
    auto actionSub = n.subscribe("agbot_action", 5, action_cb);
	auto odom_sub = n.subscribe("odom", 5, odom_cb);

	auto marker_pub = n.advertise<visualization_msgs::Marker>("wp_markers", 50);
	auto path_pub = n.advertise<nav_msgs::Path>("wp_path", 10);

	wp_path.header.frame_id="map";

	markerPath.header.frame_id = markerPt.header.frame_id = "map";
	markerPath.ns = markerPt.ns = "points_and_paths";
    markerPath.action = markerPt.action  = visualization_msgs::Marker::MODIFY;
    markerPath.pose.orientation.w = markerPt.pose.orientation.w = 1.0;
	markerPath.id = 0;
    markerPt.id = 1;

	markerPath.type = visualization_msgs::Marker::LINE_STRIP;
    markerPt.type = visualization_msgs::Marker::POINTS;

	// Points are green
    markerPt.color.g = 1.0f;
    markerPt.color.a = 1.0;
	markerPt.scale.x = 20.0;
	markerPt.scale.y = 20.0;

    // Line strip is blue
    markerPath.color.b = 1.0;
    markerPath.color.a = 1.0;
	markerPath.scale.x = 4.5;
	markerPath.scale.y = 4.5;

	ros::spinOnce();
    ros::Rate loop_rate(10);
	while (ros::ok())
    {
        marker_pub.publish(markerPath);
        marker_pub.publish(markerPt);
		path_pub.publish(wp_path);
		ros::spinOnce();
		loop_rate.sleep();
    }

	return 0;
}

