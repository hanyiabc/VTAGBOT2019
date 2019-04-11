#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

double x = 0;
double y = 0;
double xZero = 0;
double yZero = 0;
void fix_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    static bool init = false;
    if (!init)
    {
        xZero = odom->pose.pose.position.x;
        yZero = odom->pose.pose.position.y;
        init = true;
    }
    x = odom->pose.pose.position.x;
    y = odom->pose.pose.position.y;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &imu)
{
    tf::Quaternion q(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);
    static tf::TransformBroadcaster br;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3((x - xZero), (y - yZero), 0.0));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

int main(int argc, char **agrv)
{
    ros::init(argc, agrv, "map_base_tf_publisher");
    ros::NodeHandle nh;
    auto fix_sub = nh.subscribe("/odom", 500, fix_callback);
    auto imu_sub = nh.subscribe("/imu", 500, imu_callback);

    ros::spin();
}