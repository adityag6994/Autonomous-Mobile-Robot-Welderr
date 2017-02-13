#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
//#include <cv.h>
//#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <ros/ros.h>
// #include <sensor_msgs.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

extern ros::NodeHandle n;

using namespace cv;
using namespace std;

void perception(const sensor_msgs::ImageConstPtr& rosimage){
    //Mat image = imread("/home/aditya/Desktop/dynamics_project/test2.png",0);
    Mat image;
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(rosimage, sensor_msgs::image_encodings::BGR8);
        image = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    std::vector< cv::Point2f > corners;
    std::vector< cv::Point2f > finals;
    std::vector< cv::Point2f > temp;
    int maxCorners = 10;
    double qualityLevel = 0.01;
    double minDistance = 20.;
    cv::Mat mask;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;
    cv::goodFeaturesToTrack( image, corners, maxCorners, qualityLevel, minDistance, mask, blockSize, useHarrisDetector, k );

     //cout  << type(corners) << endl;
    cout << corners << endl;
    // cout << final << endl;
    int cornersSize;
    float xc[8],yc[8],zc[8];
    cornersSize = corners.size();
    
    geometry_msgs::PoseArray poseArray;
    poseArray.poses.clear(); //Clear last block perception result
    poseArray.header.stamp = ros::Time::now();
    poseArray.header.frame_id = "base_link";

    for(int k=2; k<cornersSize; k++){          //goes through all cv::Point2f in the vector
        float x = corners[k].y;   //second value
        float y = corners[k].x;   //first value
        xc[k-2] = x;
        yc[k-2] = y;
        zc[k-2] = 3;
        geometry_msgs::Pose pose;   
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 3;
        poseArray.poses.push_back(pose);
    }
    ros::Publisher pub = n.advertise<geometry_msgs::PoseArray>("percTopic", 100);
    pub.publish(poseArray);
    // ROS_INFO("poseArray size: %i",poseArray.poses.size()); //No. of points from perception block
}

int main(int argc, char** argv )
{
    ros::init(argc, argv, "image2points");
    printf("READY to get image\n");
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("/camera/depth/image_raw", 1, perception);
    ros::spin();

    return 0;
}