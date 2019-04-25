#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <sensor_msgs/Imu.h>

double x = 0;
double y = 0;
double xZero = 0;
double yZero = 0;

nav_msgs::Odometry fakeOdomMes;
geometry_msgs::Point32 imuHeading;

void steering_callback(const std_msgs::Float64::ConstPtr &msg)
{
    //fakeOdomMes.twist.twist.angular.z = msg->data;
}
void vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    fakeOdomMes.twist.twist.linear.x = sqrt(msg->vector.x * msg->vector.x + msg->vector.y * msg->vector.y);

}
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
    fakeOdomMes.pose.pose.position.x = x - xZero;
    fakeOdomMes.pose.pose.position.y = y - yZero;
    fakeOdomMes.pose.pose.position.z = 0;
    fakeOdomMes.pose.covariance = geometry_msgs::PoseWithCovariance::_covariance_type();
    fakeOdomMes.twist.covariance = geometry_msgs::PoseWithCovariance::_covariance_type();
    
    tf::Quaternion q;
    q.setRPY(imuHeading.x, imuHeading.y, imuHeading.z);
    
    fakeOdomMes.pose.pose.orientation.x = q.getX();
    fakeOdomMes.pose.pose.orientation.y = q.getY();
    fakeOdomMes.pose.pose.orientation.z = q.getZ();
    fakeOdomMes.pose.pose.orientation.w = q.getW();


}

void imu_callback(const geometry_msgs::Point32::ConstPtr &pt)
{

    //tf::Quaternion q(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);
    static tf::TransformBroadcaster br;
    imuHeading = *pt;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, pt->z);
    tf::Transform transform;
    transform.setOrigin(tf::Vector3((x - xZero), (y - yZero), 0.0));
    transform.setRotation(q);
    //ROS_INFO("%s", "SentTF****************************");
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    // fakeOdomMes.twist.twist.angular.x = pt->x;
    // fakeOdomMes.twist.twist.angular.y = pt->y;
    // fakeOdomMes.twist.twist.angular.z = pt->z;
}

void imuROS_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    fakeOdomMes.twist.twist.angular.z = msg->angular_velocity.z;
}


int main(int argc, char **agrv)
{
    ros::init(argc, agrv, "map_base_tf_publisher");
    ros::NodeHandle nh;
    fakeOdomMes.child_frame_id = "/base_link";
    fakeOdomMes.header.frame_id = "/odom";
    
    auto fix_sub = nh.subscribe("UTM", 50, fix_callback);
    auto imu_sub = nh.subscribe("novatel_imu", 50, imu_callback);
    auto fake_odom = nh.advertise<nav_msgs::Odometry>("odom", 50);
    auto vel_sub = nh.subscribe("fix_velocity", 50, vel_callback);
    auto steer_feedback = nh.subscribe("/steering_pos_cmd", 50, steering_callback);
    auto imuROS_sub = nh.subscribe("imu", 50, imuROS_callback);
    ros::spinOnce();
    ros::Rate loop_rate(100);
    while (ros::ok())
    {

        fake_odom.publish(fakeOdomMes);
        ros::spinOnce();
        loop_rate.sleep();
    }
}