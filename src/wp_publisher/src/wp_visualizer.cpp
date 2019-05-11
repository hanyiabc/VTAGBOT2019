#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <std_msgs/String.h>

visualization_msgs::Marker markerPath;
visualization_msgs::Marker markerPt;


void action_cb(const std_msgs::String::ConstPtr &msg)
{
	if (msg->data == "Reset")
	{
		markerPath.points.clear();
		markerPt.points.clear();

	}
	
}

void pt_cb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
	markerPath.points.push_back(msg->point);
	markerPt.points.push_back(msg->point);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_viz_and_receiver");
	ros::NodeHandle n;
    auto pt_sub = n.subscribe("clicked_point", 5, pt_cb);
    auto actionSub = n.subscribe("agbot_action", 5, action_cb);

	auto marker_pub = n.advertise<visualization_msgs::Marker>("wp_markers", 50);

	markerPath.header.frame_id = markerPt.header.frame_id = "/map";
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
        ros::spinOnce();
        loop_rate.sleep();
    }

	return 0;
}

