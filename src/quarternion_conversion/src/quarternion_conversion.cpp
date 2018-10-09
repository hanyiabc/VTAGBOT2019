#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>

ros::Publisher pub_pose_;

void odometryCallback_(const nav_msgs::Odometry::ConstPtr msg) {
    geometry_msgs::Pose2D pose2d;
    pose2d.x = msg->pose.pose.position.x;
    pose2d.y = msg->pose.pose.position.y;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pose2d.theta = yaw;
    pub_pose_.publish(pose2d);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "conversion_node");

    ros::NodeHandle nh_;

    ros::Subscriber sub_odom_ = nh_.subscribe("odom", 1, odometryCallback_);
    pub_pose_ = nh_.advertise<geometry_msgs::Pose2D>("pose2d", 1);

    ros::spin();

    return 0;
}
