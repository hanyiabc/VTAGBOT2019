#define _USE_MATH_DEFINES
#include <cmath>
#include "PPController.h"

PPController::PPController(double inputLeadDistance, double inputLength, double inputMinTurningRadius, double inputMaximumVelocity)
{
    leadDistance = inputLeadDistance;
    length = inputLength;
    turningRadius = inputMinTurningRadius;
    maximumVelocity = inputMaximumVelocity;
    k_theta = 0.3;
    k_delta = 1.2;
    k_vel = 0.1;
    minVelocity = 0.1;
    currWpIdx = 0;
    nPts = 0;
}

bool PPController::initialize(string filename)
{
    wpList.reserve(100);
    ifstream in(filename);
	if (in.fail())
	{
		cout << "Wrong FileName!!\n";
		return false;
	}
    string px;
    string py;
    std::getline(in, px, ',');
    std::getline(in, py, '\n');
    while(!in.fail())
    {
        wpList.push_back(Point(stod(px), stod(py) ) );
        std::getline(in, px, ',');
        std::getline(in, py, '\n');
    }
	segNormVecList = Matrix2Xd(2, wpList.size() + 1);
	segNormVecList.col(0) << Vector2d(0, 0);
    //tgtHeading = Matrix3Xd(250);
    for(size_t i = 0; i < wpList.size() - 1; i++)
    {
        tgtHeading.push_back(atan2(wpList[i+1].y - wpList[i].y, wpList[i+1].x-wpList[i].x));
		double normX = wpList[i].y - wpList[i + 1].y;
		double normY = wpList[i + 1].x - wpList[i].x;
		double nVecMag = sqrt(pow(normX, 2) + pow(normY, 2));
		segNormVecList.col(i+1) = Vector2d(normX / nVecMag, normY / nVecMag);
    }
    tgtHeading[0] = tgtHeading[1];
	segNormVecList.col(0) = segNormVecList.col(1);
	return true;
}
void PPController::compute_turning_radius(Point current, Point goal)
{
	double beta = atan2(goal.y - current.y, goal.x - current.y);
	double temp = current.inputHeading + M_PI;
	temp = fmod(temp, 2 * 3.14) - 3.14;
	double alpha = temp - beta;
	double L_a = sqrt(pow((goal.x - current.x), 2) + pow((goal.y - current.y), 2));
	turningRadius = L_a / (2 * sin(alpha));
}

void PPController::compute_steering_vel_cmds(Point current, double &vel, double &delta, double &distance2Goal)
{
    // Compute vector from current position to current waypoint:
    Vector2d vecRobot2WP = Vector2d::Zero(2,1);
    vecRobot2WP(0,0) = this->wpList[this->currWpIdx].x - current.x;
    vecRobot2WP(1,0) = this->wpList[this->currWpIdx].y - current.y;
    Vector2d vecCurHeading = Vector2d::Zero(2,1);
    // vecRobot2WP(0,0) = this->wpList[this->currWpIdx].x-current.x;
    // vecRobot2WP(1,0) = this->wpList[this->currWpIdx].y - current.y;

    vecCurHeading(0,0) = cos(this->tgtHeading[this->currWpIdx]);
    vecCurHeading(1,0) = sin(this->tgtHeading[this->currWpIdx]);
    // distance2Goal = vecRobot2WP.transpose().dot(vecCurHeading.normalized());
    distance2Goal = vecRobot2WP.dot(vecCurHeading.normalized());

    cout<<"distance2Goal:\t" << distance2Goal << "\n";

    //Compute the minimum distance from the current segment:
    //change this
    double minDist = vecRobot2WP.dot(segNormVecList.col(currWpIdx));
    double theta_gain = this->k_theta * minDist;
    if (theta_gain > M_PI /2)
    {
        theta_gain = M_PI/2;
    }
    if(theta_gain < -M_PI/2)
    {
        theta_gain = -M_PI/2;
    }
    cout << "minDist = " << minDist << "\n";
    cout << "theta_gain = " << theta_gain << "\n";
        //Compute the desired heading angle based of target heading and the min dist:
    double theta_des = this->tgtHeading[this->currWpIdx] + theta_gain;

    cout << "Theta des = " << theta_des << "\n"; //theta des use global reference
        //Compute the steering agle command:

        //change this
    double heading_err = theta_des - current.inputHeading;
    cout << "current Heading:\t" << current.inputHeading << "\n";
    cout << "headding error\t" << heading_err << "\n";
        if(heading_err > M_PI)
        {
            heading_err = heading_err - 2 * M_PI;
        }
        else if(heading_err < -M_PI)
        heading_err = heading_err + 2 * M_PI;
        delta = this->k_delta*(heading_err);
        cout << "Target heading = " << this->tgtHeading[this->currWpIdx] << "\n";

        //Compute forward velocity:
        vel = this->maximumVelocity - abs(this->k_vel * delta);


        if (vel < this->minVelocity)
            vel = this->minVelocity;
        if (delta > 1)
            delta = 1;
        if (delta < -1)
            delta = -1;
}
// compute the steering angle of ackermann vehicle of given paramters
double PPController::compute_steering_angle()
{
    // Steering angle command from Pure pursuit paper:
    // Steering angle = atan(L/R)
    return atan(length / turningRadius);

}

// compute forward velocity relative to steering angle
double PPController::compute_forward_velocity()
{
    // forwardVelocity = mule.maximumVelocity * (1 - atan(abs(steeringAngle))/(pi/2));
    //this specifies the forward velocity at a given steering angle
    double forwardVelocity = 0.4;
    return forwardVelocity;
}

static Vector2d unit_vector(Vector2d vector)
{
	return vector.normalized();
}

size_t PPController::getcurrWpIdx()
{
    return this->currWpIdx;
}
void PPController::incrimentWpIdx()
{
    currWpIdx++;
}
size_t PPController::getnPts()
{
    return this->wpList.size();
}
vector<Point> PPController::getwpList()
{
    return this->wpList;
}

PPController::~PPController()
{

}
