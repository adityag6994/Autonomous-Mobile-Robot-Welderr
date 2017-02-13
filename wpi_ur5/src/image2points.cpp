#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cv.h>
//#include <highgui.h/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <ros/ros.h>
// #include <sensor_msgs.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

using namespace cv;
using namespace std;

class ImageProcessor
{
public:
  ImageProcessor()
  {
    pub = n.advertise<geometry_msgs::PoseArray>("/percTopic", 100);

    image_transport::ImageTransport it(n);

    ROS_ERROR("perception constructor reached");
    sub = it.subscribe("/camera/image_raw", 1, &ImageProcessor::perception, this);
    perception();
  }

  void perception(const sensor_msgs::ImageConstPtr& rosimage){
    
    ROS_ERROR("perception callback");
    Mat image;
    cv_bridge::CvImagePtr cv_ptr;
    try{
        //cv_ptr = cv_bridge::toCvCopy(rosimage, sensor_msgs::image_encodings::BGR8);
        cv_ptr = cv_bridge::toCvCopy(rosimage, "bgr8");
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

    ROS_INFO("Processed the waypoints.");

    for(int k=2; k<cornersSize; k++){          //goes through all cv::Point2f in the vector
        float x = corners[k].y;   //second value
        float y = corners[k].x;   //first value
        xc[k-2] = x;
        yc[k-2] = y;
        zc[k-2] = 0.3;
        geometry_msgs::Pose pose;   
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0.3;
        poseArray.poses.push_back(pose);
    }

    ROS_INFO("Publishing poseArray.");
    pub.publish(poseArray);
    ROS_INFO("PoseArray published.");
    // ROS_INFO("poseArray size: %i",poseArray.poses.size()); //No. of points from perception block
    ros::spinOnce();
    sleep(20.0);
    ROS_ERROR("perception shutdown");
    ros::shutdown();
}

void perception(){
    
    ROS_ERROR("perception called");
    Mat image = imread("/home/charvak/catkin_ws/src/brownsugarMafia/wpi_ur5/src/test2.png", 0);
    ROS_ERROR("Image read");
    
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

    int cornersSize;
    float xc[8],yc[8],zc[8];
    cornersSize = corners.size();
    
    geometry_msgs::PoseArray poseArray;
    poseArray.poses.clear(); //Clear last block perception result
    poseArray.header.stamp = ros::Time::now();
    poseArray.header.frame_id = "base_link";

    ROS_INFO("Processed the waypoints.");

    for(int k=2; k<cornersSize; k++){          //goes through all cv::Point2f in the vector
        float x = corners[k].y/391;   //second value
        float y = corners[k].x/523;   //first value
        xc[k-2] = x;
        yc[k-2] = y;
        zc[k-2] = 0.3;
        geometry_msgs::Pose pose;
        // std::ostringstream ss;
        // ss << x;
        // std::string test(ss.str());
        // std::string test = boost::lexical_cast<std::string>(x);
        // ROS_ERROR(test);
        // ROS_ERROR(y);   
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0.3;
        poseArray.poses.push_back(pose);
    }

    ROS_ERROR("Publishing poseArray.");
    // sleep(40.0);
    
    ros::Rate poll_rate(100);
    while(pub.getNumSubscribers() == 0)
      poll_rate.sleep();

    pub.publish(poseArray);
    ROS_ERROR("PoseArray published.");
    // ROS_INFO("poseArray size: %i",poseArray.poses.size()); //No. of points from perception block
    ros::spinOnce();
    sleep(20.0);
    ROS_ERROR("perception waiting for shutdown");
    ros::waitForShutdown();
  }

private:
  ros::NodeHandle n; 
  ros::Publisher pub;
  image_transport::Subscriber sub;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "perception");
  ImageProcessor IPObject;

  ros::spin();

  return 0;
}