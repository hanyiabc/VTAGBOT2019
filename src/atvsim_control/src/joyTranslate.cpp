#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Joy.h"

class Translator
{
public:
  std_msgs::Float64 left_vertical;
  std_msgs::Float64 right_horizontal;
  std_msgs::Bool engine_cut;
  void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);
};

void Translator::joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  left_vertical.data = msg->axes[1];
  right_horizontal.data = msg->axes[3];
  if (msg->axes[18] == -1)
  {
    engine_cut.data = true;
  }
  else
  {
    engine_cut.data = false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "adaptor");
  ros::NodeHandle n;
  ros::Publisher steer_pub = n.advertise<std_msgs::Float64>("steering_pos_cmd", 1000);
  ros::Publisher pedal_pub = n.advertise<std_msgs::Float64>("pedal_effort_cmd", 1000);
  ros::Publisher engine_pub = n.advertise<std_msgs::Bool>("engine_cut", 1000);
  Translator translator;
  ros::Subscriber joy_sub = n.subscribe("joy", 1000, &Translator::joy_callback, &translator);
  ros::Rate loop_rate(100);
  while(ros::ok())
  {
    ROS_INFO("%f,%f", translator.left_vertical.data, translator.right_horizontal.data);
    steer_pub.publish(translator.right_horizontal);
    pedal_pub.publish(translator.left_vertical);
    engine_pub.publish(translator.engine_cut);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
