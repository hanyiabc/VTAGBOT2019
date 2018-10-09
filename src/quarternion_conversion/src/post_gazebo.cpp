//this file do some post conversion after gazebo to match it with reality
//it converts quarterion to row pich yall

#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>

ros::Publisher pub_rpy_;

void imuCallback_(const sensor_msgs::Imu::ConstPtr msg) {
    geometry_msgs::Vector3 rpy;

    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;
    pub_rpy_.publish(rpy);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "conversion_node");

    ros::NodeHandle nh_;

    ros::Subscriber sub_imu_ = nh_.subscribe("/imu", 1, imuCallback_);
    pub_rpy_ = nh_.advertise<geometry_msgs::Vector3>("IMU_rpy", 1);

    ros::spin();

    return 0;
}
