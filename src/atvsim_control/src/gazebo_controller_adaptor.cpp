#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <math.h>
#include <sstream>

double wheelBase = 2.0;
double convert_trans_rot_vel_to_steering_angle(double v, double omega, double wheelbase)
{
  if (omega == 0 || v == 0)
    return 0;
  double radius = v / omega;
  return atan(wheelbase / radius);
}

class Adaptor
{
public:
  std_msgs::Float64 steering_cmd;
  float pedal_cmd;
  void steering_pos_cmd_Callback(const std_msgs::Float64::ConstPtr &msg);
  // void pedal_effort_cmd_Callback(const std_msgs::Float64::ConstPtr& msg);
  void pedal_cmd_Callback(const std_msgs::Float64::ConstPtr &msg);
};

void move_base_cmd_Callback(const geometry_msgs::Twist::ConstPtr &msg)
{
  static ros::NodeHandle n;
  static ros::Publisher steering_pub = n.advertise<std_msgs::Float64>("/steering_pos_cmd", 1000);
  static ros::Publisher pedal_pub = n.advertise<std_msgs::Float64>("/pedal_cmd", 1000);
  geometry_msgs::Twist newTwist(*msg);

  std_msgs::Float64 steer_mes;
  steer_mes.data = msg->angular.z;
  //steer_mes.data = convert_trans_rot_vel_to_steering_angle(msg->linear.x, msg->angular.z, wheelBase);
  std_msgs::Float64 ped_cmd;
  ped_cmd.data = newTwist.linear.x;
  pedal_pub.publish(ped_cmd);
  steering_pub.publish(std_msgs::Float64(steer_mes));
}

void Adaptor::steering_pos_cmd_Callback(const std_msgs::Float64::ConstPtr &msg)
{
  steering_cmd.data = msg->data;
  // if (msg->data > 0)
  // {
  //   steering_cmd.data = msg->data * 0.581;
  // }
  // else
  // {
  //   steering_cmd.data = msg->data * 0.7727;
  // }
}

void Adaptor::pedal_cmd_Callback(const std_msgs::Float64::ConstPtr &msg)
{
  pedal_cmd = msg->data;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "adaptor");
  ros::NodeHandle n;
  Adaptor adaptor;
  geometry_msgs::Twist twist;

  ros::Publisher steering_pub = n.advertise<std_msgs::Float64>("/steer_controller/command", 1000);
  ros::Publisher pedal_pub = n.advertise<geometry_msgs::Twist>("/rear_wheel_vel", 1000);
  ros::Subscriber steering_sub = n.subscribe("steering_pos_cmd", 1000, &Adaptor::steering_pos_cmd_Callback, &adaptor);
  ros::Subscriber pedal_sub = n.subscribe("pedal_cmd", 1000, &Adaptor::pedal_cmd_Callback, &adaptor);
  ros::Subscriber move_base_sub = n.subscribe("cmd_vel", 100, move_base_cmd_Callback);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {

    steering_pub.publish(adaptor.steering_cmd);
    twist.linear.x = adaptor.pedal_cmd;
    pedal_pub.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
