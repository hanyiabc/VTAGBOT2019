#pragma once

#include <opencv2/core.hpp>
#include <opencv/cv.hpp>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <math.h>
#include <limits.h>
#include <cv_bridge/cv_bridge.h>

using cv::cvtColor;
using cv::imread;
using cv::imshow;
using cv::Mat;
using cv::Scalar;
using cv::Vec2f;
using cv::Vec3b;
using cv::Vec4i;
using cv::waitKey;

using std::cout;
using std::string;
using std::vector;

#define DEBUGGING 0

void cropRowDetec(Mat &m);
void grayTransform(Mat &m);
void skeletonize(Mat &m);
vector<cv::Vec2f> houghTransform(Mat &m);
void filterSimLines(vector<Vec2f> &lines);
double solveforB(const Vec2f & twoPts);
cv::Vec2d twoPoints2Polar(const cv::Vec4i &line);
double intersection(const cv::Point2f p1, const cv::Point2f p2, const cv::Point2f p3, const cv::Point2f p4);
//double average(Mat &m);

const int HOUGH_RHO = 2;				 //Distance resolution of the accumulator in pixels
const double HOUGH_ANGLE = CV_PI / 45.0; //Angle resolution of the accumulator in radians --------- 4 degrees
const int HOUGH_THRESH_MAX = 100;			  //Accumulator threshold parameter.Only those lines are returned that get enough votes
const int HOUGH_THRESH_MIN = 10;
const int HOUGH_THRESH_INCR = 1;
const int NUMBER_OF_ROWS = 3;						 //how many crop rows to detect
const double THETA_SIM_THRESH = CV_PI / 30.0; //How similar two rows can be ----- 6 degrees
const int RHO_SIM_THRESH = 8;						 //How similar two rows can be
const double ANGLE_THRESH = CV_PI / 6.0;	//How steep angles the crop rows can be in radians -------- 30 degrees
const string FILE_NAME = "crop_row_013.JPG";

/*
int main(int argc, char const *argv[])
{
	if(argc >= 2)
	{
		cout << "Read image from commandline\n";

		for(size_t i = 1; i < argc; i++)
		{
			Mat im = imread(argv[i]);
			if (im.empty())
			{
				cout << "Wrong File Name!\n";
				cout << "File name is" << argv[i] << "\n";
				return EXIT_FAILURE;
			}
			cropRowDetec(im);
			string path = "output/" + string(argv[i]);
			cout << "Writing to file:" << path << "\n";
			cv::imwrite(path, im);
		}

		
	}
	if(argc == 1)
	{
		//cout << "Read image from constant\n";

		cout << "Use webcam";
		cv::VideoCapture cap(0); // open the default camera
		if (!cap.isOpened())  // check if we succeeded
			return EXIT_FAILURE;
		while (true)
		{
			Mat im;
			cap >> im;
			cropRowDetec(im);
			imshow("Crop Rows", im);
			if (waitKey(30) >= 0) break;
		}
		//Mat im = imread(FILE_NAME);
		//cropRowDetec(im);
	}

	cout << "Done\n";
	return EXIT_SUCCESS;
}
*/
void cropRowDetec(Mat &m)
{
	Mat original; 
	m.copyTo(original); 
	grayTransform(m);
	if (DEBUGGING)
	{
		imshow("Gray", m);
	}
	skeletonize(m);
	if (DEBUGGING)
	{
		imshow("Skele", m);
	}
		
	auto lines = houghTransform(m);

	//my code
	int j = 0;
	cv::Point2f points[lines.size()*2];
	//end my code 

	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		cv::Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		cv::line(original, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
	
		//my code
		points[j] = pt1;
		points[j+1] = pt2;
		j += 2;
		//end my code
	}

	//my code
	double xCoord[lines.size() - 1];
	int b = 0;
	for(int a = 0; a < lines.size()-1; a++)
	{
		xCoord[a] = intersection(points[b],points[b+1],points[b+2],points[b+3]);
		b += 2;
	} 

	double sum = 0;
	for(int k = 0; k < lines.size()-1;k++)
	{
		sum += xCoord[k];
	}
	double xAvg = sum/(lines.size()-1);
	cv::Point pt3, pt4;
	pt3.x =xAvg;
	pt4.x = xAvg;
	pt3.y = 0;
	pt4.y = 100;
	cv::line(original, pt3, pt4, Scalar(255, 0, 0), 3, CV_AA);
	cout << "The average x coordinate intersection is: " << xAvg << std::endl;
	//end my code

	m = original;
	if (DEBUGGING)
	{
		imshow("Lines", m);
		waitKey(0);
	}
}

void grayTransform(Mat &m)
{

	for (size_t i = 0; i < m.rows; i++)
	{
		for (size_t j = 0; j < m.cols; j++)
		{
			
			Vec3b bgr = m.at<Vec3b>(i, j);
			auto GVal = 2 * bgr[1] - bgr[0] - bgr[2];
			m.at<Vec3b>(i, j)[0] = GVal;
			m.at<Vec3b>(i, j)[1] = GVal;
			m.at<Vec3b>(i, j)[2] = GVal;

		}
	}

	Mat temp;
	cv::cvtColor(m, temp, cv::COLOR_BGR2GRAY);
	m = temp;
}

void skeletonize(Mat &m)
{
	Mat binIm;
	cv::threshold(m, binIm, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	m = binIm;

	auto size = m.rows * m.cols;
	cv::Mat skel(m.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat temp(m.size(), CV_8UC1);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

	bool done = false;
	while (!done)
	{
		Mat eroded;
		cv::erode(m, eroded, element);
		cv::dilate(eroded, temp, element);
		cv::subtract(m, temp, temp);
		cv::bitwise_or(skel, temp, skel);
		m = eroded;
		auto zeros = size - cv::countNonZero(m);
		if (zeros == size)
		{
			done = true;
		}
	}

	m = skel;
}

vector<cv::Vec2f> houghTransform(Mat &m)
{

	int hough_thresh = HOUGH_THRESH_MAX;
	bool row_found = false;

	vector<cv::Vec2f> lines;
	while(hough_thresh > HOUGH_THRESH_MIN && !row_found)
	{
		lines.clear();
		cv::HoughLines(m, lines, HOUGH_RHO, HOUGH_ANGLE, hough_thresh);
		
		filterSimLines(lines);

		hough_thresh -= HOUGH_THRESH_INCR;
		if(lines.size() >= NUMBER_OF_ROWS)
		{
			row_found = true;
		}
	}

	return lines;
}

double solveforB(const Vec2f &twoPts)
{
	if (twoPts[2] - twoPts[0] == 0)
	{
		return std::numeric_limits<double>::max();
	}
		double m = (twoPts[3] - twoPts[1]) / (twoPts[2] - twoPts[0]);
	return twoPts[3] - twoPts[2] * m;
}

void filterSimLines(vector<Vec2f> &lines)
{
	if (!lines.empty())
	{
		for (int i = lines.size() - 1; i >= 0; i--)
		{
			if (  ( (ANGLE_THRESH <= lines[i][1]) && (lines[i][1] <= CV_PI - ANGLE_THRESH) )  || 
				  /*( (-ANGLE_THRESH > lines[i][1]) && (lines[i][1] > ANGLE_THRESH - CV_PI) ) ||*/
				lines[i][1] <= 0.0001)
			{
				lines.erase(lines.begin() + i);
			}
			else
			{
				for (size_t j = 0; j < lines.size(); j++)
				{

					if (j != i)
					{
						if (abs(lines[i][1] - lines[j][1]) < THETA_SIM_THRESH)
						{
							if (abs(lines[i][0] - lines[j][0]) < RHO_SIM_THRESH)
							{
								lines.erase(lines.begin() + i);
								break;
							}
								
						}
						
						// if (abs(lines[i][1] - lines[j][1]) < THETA_SIM_THRESH)
						// {
						// 	lines.erase(lines.begin() + i);
						// 	break;
						// }
						// else if (abs(lines[i][0] - lines[j][0]) < RHO_SIM_THRESH)
						// {
						// 	lines.erase(lines.begin() + i);
						// 	break;
						// }
					}

				}

			}

		}
	}
	
}

cv::Vec2d twoPoints2Polar(const cv::Vec4i &line)
{
	// Get points from the vector
	cv::Point2f p1(line[0], line[1]);
	cv::Point2f p2(line[2], line[3]);

	// Compute 'rho' and 'theta'
	double rho = abs(p2.x * p1.y - p2.y * p1.x) / cv::norm(p2 - p1);
	double theta = -atan2((p2.x - p1.x), (p2.y - p1.y));

	// You can have a negative distance from the center
	// when the angle is negative
	if (theta < 0)
	{
		rho = -rho;
	}

	//if (theta < 0)
	//{
	//	theta = 2 * CV_PI + theta;
	//}
	return cv::Vec2d(rho, theta);
}

//start my code
double intersection(const cv::Point2f p1, const cv::Point2f p2, const cv::Point2f p3, const cv::Point2f p4)
{
	double slope1 = (p1.y-p2.y)/(p1.x-p2.x);
	double slope2 = (p3.y-p4.y)/(p3.x-p4.x);
	double xCoord;
	if(slope1 == slope2)
	{
		xCoord == ((p2.x)+(p4.x))/2;
	}
	else
	{
		xCoord = ((p3.y-p1.y) + (slope1*p1.x) - (slope2*p3.x))/(slope1-slope2);
	}

	return xCoord;
}
//end my code

/*
double average(Mat &m)
{
	int j = 0;
	auto lines = houghTransform(m);
	cv::Point2f points[lines.size()*2];
	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		cv::Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		
		points[j] = pt1;
		points[j+1] = pt2;
		j += 2;
	}
	double xCoord[lines.size()/4];
	int b = 0;
	for(int a = 0; a < lines.size()/4; a++)
	{
		xCoord[a] = intersection(points[b],points[b+1],points[b+2],points[b+3]);
		b += 4;
	} 
	double sum = 0;
	for(int k = 0; k < lines.size()/4;k++)
	{
		sum += xCoord[k];
	}
	double xAvg = sum/lines.size()/4;
	return xAvg;
}
*/
