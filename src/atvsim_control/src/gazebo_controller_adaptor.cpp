#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"

#include <sstream>

class Adaptor
{
public:
  std_msgs::Float64 steering_cmd;
  float pedal_cmd;
  void steering_pos_cmd_Callback(const std_msgs::Float64::ConstPtr& msg);
  void pedal_effort_cmd_Callback(const std_msgs::Float64::ConstPtr& msg);

};

void Adaptor::steering_pos_cmd_Callback(const std_msgs::Float64::ConstPtr& msg)
{
  if (msg->data > 0)
  {
    steering_cmd.data = msg->data * 0.581;
  }
  else
  {
    steering_cmd.data = msg->data * 0.7727;
  }

}
void Adaptor::pedal_effort_cmd_Callback(const std_msgs::Float64::ConstPtr& msg)
{
  pedal_cmd = msg->data * 10000;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "adaptor");
  ros::NodeHandle n;
  Adaptor adaptor;
  std_msgs::Float64MultiArray pedal_cmd_array;
  std_msgs::MultiArrayDimension dim;
  dim.label = "";
  dim.size = 2;
  pedal_cmd_array.data.clear();
  pedal_cmd_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
  ros::Publisher steering_pub = n.advertise<std_msgs::Float64>("/steer_controller/command", 1000);
  ros::Publisher pedal_pub = n.advertise<std_msgs::Float64MultiArray>("/rear_drive_controller/command", 1000);

  ros::Subscriber steering_sub = n.subscribe("steering_pos_cmd", 1000, &Adaptor::steering_pos_cmd_Callback, &adaptor);
  ros::Subscriber pedal_sub = n.subscribe("pedal_effort_cmd", 1000, &Adaptor::pedal_effort_cmd_Callback, &adaptor);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    // ROS_INFO("Steering: %f,Pedal: %f", adaptor.steering_cmd.data, adaptor.pedal_cmd.data[0]);

    pedal_cmd_array.layout.dim.clear();
    pedal_cmd_array.layout.dim.push_back(dim);

    pedal_cmd_array.data.clear();
    pedal_cmd_array.data.push_back(adaptor.pedal_cmd);
    pedal_cmd_array.data.push_back(adaptor.pedal_cmd);
    steering_pub.publish(adaptor.steering_cmd);
    pedal_pub.publish(pedal_cmd_array);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
