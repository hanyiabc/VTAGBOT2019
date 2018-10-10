//this file do some post conversion after gazebo to match it with reality
//it converts quarterion to row pich yall

#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>

ros::Publisher pub_rpy_;
ros::Publisher pub_gps_;
geometry_msgs::Vector3 rpy;


void imuCallback_(const sensor_msgs::Imu::ConstPtr msg) {

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

void gpsCallback_(const geometry_msgs::Vector3Stamped::ConstPtr msg)
{
  std_msgs::Float64 speed;
  speed.data = sqrt(pow(msg->vector.x,2) + pow(msg->vector.y,2));
  if (rpy.z * msg->vector.y < 0)
  {
    speed.data = -speed.data;
  }
  pub_gps_.publish(speed);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "conversion_node");

    ros::NodeHandle nh_;
    rpy.x = 0; rpy.y = 0; rpy.z = 0;
    ros::Subscriber sub_imu_ = nh_.subscribe("/imu", 1, imuCallback_);
    ros::Subscriber sub_speed_ = nh_.subscribe("/fix_velocity", 1, gpsCallback_);
    pub_rpy_ = nh_.advertise<geometry_msgs::Vector3>("IMU_rpy", 1);
    pub_gps_ = nh_.advertise<std_msgs::Float64>("GPS_speed", 1);

    ros::spin();

    return 0;
}
