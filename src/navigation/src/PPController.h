#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "ros/ros.h"

using Eigen::Matrix2Xd;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;
using std::cout;
using std::ifstream;
using std::stod;
using std::string;
using std::vector;
//struct to define vehicle position on a coordinate system at a certain heading
struct Point
{
    double x;
    double y;
    double inputHeading;
    Point() : x(0), y(0), inputHeading(0) {}
    Point(double xIn, double yIn, double headingIn) : x(xIn), y(yIn), inputHeading(headingIn) {}
    Point(double xIn, double yIn) : x(xIn), y(yIn) {}
    Vector2d toVector2D()
    {
        return Vector2d(x, y);
    }
};

//class to define vehicle parameters
struct AckermannVehicle
{
    double length;
    double maximumVelocity;
    double minTurningRadius;
    AckermannVehicle(double lengthIn, double maximumVelocityIn, double minTurningRadiusIn) : length(lengthIn), maximumVelocity(maximumVelocityIn), minTurningRadius(minTurningRadiusIn) {}
};

//class to define Pure pursuit controller parameters
class PPController
{
  private:
    double leadDistance, length, turningRadius, maximumVelocity; //set the length for ppcontroller as the length of maximumSteeringAngle

    //List of waypoints: From start to end:

    vector<Point> wpList;
    //Current target waypoint index:
    size_t currWpIdx;

    //List of desired heading values:
    vector<double> tgtHeading;
    //Matrix3Xd tgtHeading;
    //List of normal vectors to segments joining the waypoints:
    Matrix2Xd segNormVecList;
    //Number of waypoints:
    size_t nPts;
    //Tuning gains:
    double k_theta;

    double k_delta;

    double k_vel;

    double minVelocity;

  public:
    PPController(double inputLeadDistance, double inputLength = 2.065, double inputMinTurningRadius = 4.6, double inputMaximumVelocity = 0.5);
    bool initialize(string filename);

    //Function to compute steering angle and forward velocity commands:
    //References are all return values
    void compute_steering_vel_cmds(Point current, double &vel, double &delta, double &distance2Goal);

    //compute the steering radius of ackerman vehicle of given parameters
    void compute_turning_radius(Point current = Point(0, 0, 0), Point goal = Point(0, 0, 0));

    //compute the steering angle of ackermann vehicle of given paramters
    double compute_steering_angle();

    //compute forward velocity relative to steering angle
    double compute_forward_velocity(); //added a variable velocity based on Bijo's suggestion

    //return the unit vector of the vector
    static Vector2d unit_vector(Vector2d vector);
    //getters
    size_t getcurrWpIdx();
    size_t getnPts();
    vector<Point> getwpList();
    void incrimentWpIdx();
    void resetWpIdx();
    ~PPController();
};

#endif
