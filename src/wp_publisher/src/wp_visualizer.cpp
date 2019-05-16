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

visualization_msgs::Marker markerPath;
visualization_msgs::Marker markerPt;
nav_msgs::Path wp_path;


void action_cb(const std_msgs::String::ConstPtr &msg)
{
	if (msg->data == "Reset")
	{
		markerPath.points.clear();
		markerPt.points.clear();
		wp_path.poses.clear();
	}
	
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
	if(wp_path.poses.size() > 1)
	{
		double x_d = wp_path.poses[wp_path.poses.size() - 2].pose.position.x - msg->point.x;
		double y_d = wp_path.poses[wp_path.poses.size() - 2].pose.position.y - msg->point.y;
		double angle = atan2(x_d, y_d);
		angle -= M_PI;
		tf::Quaternion q;
		q.setRPY(0.0, 0.0, angle);
		geometry_msgs::Quaternion gq;
		gq.w = q.getW();
		gq.x = q.getX();
		gq.y = q.getY();
		gq.z = q.getZ();
		wp_path.poses[wp_path.poses.size() - 2].pose.orientation = gq;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_viz_and_receiver");
	ros::NodeHandle n;
    auto pt_sub = n.subscribe("clicked_point", 5, pt_cb);
    auto actionSub = n.subscribe("agbot_action", 5, action_cb);

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

