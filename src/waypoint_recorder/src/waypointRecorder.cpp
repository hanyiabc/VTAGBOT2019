#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include <fstream>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
using namespace std;
using Eigen::Matrix2Xd;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;
using std::cout;

class Recorder
{
public:
  Vector2d currentLocation = Vector2d::Zero(2, 1);
  Vector2d lastLocation = Vector2d::Zero(2, 1);
  Vector2d distanceVec = Vector2d::Zero(2, 1);
  ofstream targetFile;
  ros::NodeHandle n;
  void utm_Callback(const nav_msgs::Odometry::ConstPtr &msg);
};

void Recorder::utm_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  currentLocation(0, 0) = msg->pose.pose.position.x;
  currentLocation(1, 0) = msg->pose.pose.position.y;
  ROS_INFO("UTM_x: %f\tUTM_y: %f", msg->pose.pose.position.x, msg->pose.pose.position.y);
  distanceVec = currentLocation - lastLocation;
  float distance = distanceVec.norm();
  cout << setprecision(20) << "vector = " << currentLocation << "\n";
  cout << "distance = " << distance << "\n";
  int start = 0;
  n.param<int>("/waypoint_recorder/start_recording", start, 0);
  if (start == 1)
  {
    if (!targetFile.is_open())
    {
      string file_name;
      n.param<string>("/waypoint_recorder/waypoint_file_name", file_name, "waypoints_no_name.txt");
      targetFile.open(file_name);
    }
    if (distance > 0.5)
    {
      lastLocation = currentLocation;
      targetFile << setprecision(20) << currentLocation(0, 0) << ", " << currentLocation(1, 0) << "\n";
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_recorder");

  Recorder recorder;

  ros::Subscriber utm_sub = recorder.n.subscribe("/UTM", 1000, &Recorder::utm_Callback, &recorder);
  ros::spin();
  recorder.targetFile.close();
  return 0;
}
