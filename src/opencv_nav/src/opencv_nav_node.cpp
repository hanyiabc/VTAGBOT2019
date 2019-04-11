#include <ros/ros.h>
#include "crop_tracker.h"
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv/cv.hpp>
using cv::imshow;
     void imageCallback(const sensor_msgs::ImageConstPtr &image) {

                    
                    cv_bridge::CvImagePtr matImage = cv_bridge::toCvCopy(image);
                     
                     
                    cropRowDetec(matImage->image);
                   // string path = "output/" + string(argv[i]);
                   // cout << "Writing to file:" << path << "\n";
                    //imshow("Result lines", matImage->image);
                  ROS_INFO("%s", "Done\n");
     }

     int main(int argc, char **argv) {
        ros::init(argc, argv, "opencv_nav_node");
        ros::NodeHandle n("~");
        ros::Subscriber sub = n.subscribe("/front_camera/front_image_raw", 10, imageCallback);
        ros::Rate loop_rate(50);
        while (ros::ok()) {
            ros::spinOnce();

         loop_rate.sleep();
        }
     }


     

     

