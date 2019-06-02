/* @author Aytaç Kahveci */
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/utility.hpp>

#include <vector>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <mutex>

#include <ros/ros.h>
#include <ros/transport_hints.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

class objectTracker
{
private:
    Mat frame, procFrame;

    ros::NodeHandle nh_;
    ros::Subscriber roi_sub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber it_sub_, it_sub2_;
    std::string ns_;
    VideoWriter outputVideo, outputVideo2;

    std::mutex lock_;
    bool flag = false;
public:
    objectTracker(ros::NodeHandle nh) : nh_(nh), it_(nh_)
    {
        ns_ = nh_.getNamespace();
        roi_sub_ = nh_.subscribe(ns_ + "/image_raw_roi",1,&objectTracker::roiCb,this);

        it_ = image_transport::ImageTransport(nh_);
        it_sub_ = it_.subscribe(ns_ + "/image_raw",1,&objectTracker::imageCbRaw, this);
        it_sub2_ = it_.subscribe(ns_ + "/image_processed",1,&objectTracker::imageCbProcessed, this);

    }

    ~objectTracker()
    {
    }

    void imageCbRaw(const sensor_msgs::ImageConstPtr& msg)
    {
        if(flag)
        {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        frame = cv_ptr->image;
            outputVideo << frame;
        }
    }

    void imageCbProcessed(const sensor_msgs::ImageConstPtr& msg)
    {
        if(flag)
        {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        procFrame = cv_ptr->image;

            outputVideo2 << procFrame;
        }
    }

    void roiCb(const sensor_msgs::RegionOfInterest& roiMsg)
    {
        if(lock_.try_lock())
        {
            int64_t modWith = static_cast<int64_t>(roiMsg.width % 4 );
            if(modWith > 4/2)
                modWith = modWith - 4;

            int64_t modHeight = static_cast<int64_t>(roiMsg.height % 2 );
            if(modHeight > 2/2)
                modHeight = modHeight - 2;

            int frameWidth = static_cast<int>(roiMsg.width - modWith);
            int frameHeight = static_cast<int>(roiMsg.height - modHeight);

            std::string outputPath = "/home/aytac/Desktop/polygonRecord.avi";
            //int ex = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC)); //Get codec type
            Size s = Size(frameWidth, frameHeight);
            outputVideo.open(outputPath, CV_FOURCC('D','I','V','X'), 28, s, true);
            if(!outputVideo.isOpened())
            {
                printf("Error openning the output video for writing\n");
            }

            outputPath = "/home/aytac/Desktop/polygonDrawRecord.avi";
            outputVideo2.open(outputPath, CV_FOURCC('D','I','V','X'), 28, s, true);
            if(!outputVideo2.isOpened())
            {
                printf("Error openning the output video for writing\n");
            }

            flag = true;
            lock_.unlock();
        }
        else
            ROS_ERROR("mutex lock error");
    }
};

int main(int argc, char *argv[]) {
    ros::init(argc,argv,"VideoWriter");
    ros::NodeHandle nh;
    ros::TransportHints().tcpNoDelay();

    //Image Subscriber object
    objectTracker ot(nh);

    ros::Rate loop_rate(20);
    double now;
    double previous = ros::Time::now().toSec();

    while(ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
