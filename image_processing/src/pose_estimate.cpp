/* @author Aytaç Kahveci */
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/tracking/tldDataset.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/video/tracking.hpp>

#include <vector>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <mutex>
#include <map>

#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <cv_bridge/cv_bridge.h>
#include <kuka_hw_cart_vel/kuka_class.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <dynamic_reconfigure/server.h>
#include <image_processing/hsvThreshConfig.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>

#include <angles/angles.h>

using namespace cv;
using namespace image_processing;
/*************** Image Processing Variables **************/
Mat frame, hsvImage, hsvImageThresh, pcaFrame, edges, mask, objBw;
Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3,3) );
Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8,8) );

const int FRAME_WIDTH = 1920;
const int FRAME_HEIGHT = 1200;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 40*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

int x;
int y;
double x_obj = 0;
double y_obj = 0;
double scale = 1;
int offsetX, offsetY = 0;

#define PI 3.141592654

int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;

const std::string windowName = "Source Image";
const std::string windowName2 = "HSV Image";

bool mouseIsDragging = false;
bool mouseMove = false;
bool rectangleSelected = false;
bool refMouseMove = false;
Rect rectangleROI;
Point currentPoint;
Point initialPoint;
Point initRefPoint, finalRefPoint;
bool initRefSelected = false;
bool objFound = false;

//Tracker variables
Rect2d roi;
Ptr<TrackerBoosting> tracker = TrackerBoosting::create();

Rect roi_;
Mat maskFrame, procFrame;
std::vector<Point> objLocation;
double angle, memAngle;
std::vector<Point2d> eigen_vecs(2);
std::vector<double> eigen_val(2);


//Kalman variables
KalmanFilter kf(9, 3, 0);
Mat measurement = Mat::zeros(3, 1, CV_32F);
Mat prediction, estimated;
Point2f predictPt, statePt, measPt;
Point2f posCur, posPrev;
int count = 0;
bool init, kalmanInit = true;
bool isObjectFound = false;

///hsvTrackbar is called whenever the trackbar position is changed
void hsvTrackbar(int, void*){}
void morphOps(Mat &image);
void findObj(Mat& image);
void drawObject(int x, int y,Mat &frame);
std::string intToString(int number);
void mouseCallBack(int event, int x, int y, int flags, void* userdata);
void recordHSV(Mat& frame, Mat& HSV);
void rotateImage(Mat& src, double angle);
void drawAxis(Mat&, Point, Point, Scalar, const float);
double getOrientation(const std::vector<Point> & pts, Mat& image, Point&);
/*************** /Image Processing Variables **************/

/******** ROS variables *************/
//<kuka_vars>
float A_ = 180; //unit degree
float B_ = 0; //unit degree
float C_ = 180; //unit degree
float movingRes = 0.1; //unit mm
float movingThresh = 1; //unit mm
float kukaVel = 0.5; //unit m/s
//</kuka_vars>

//<modes>
bool rightMode = false; //default mode
bool leftMode = true;
//</modes>

//<flags>
bool isObjectMoving = false;
//</flags>
/******** /ROS variables ***********/

class objectTracker
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber obj_roi_sub_, roi_sub_, command_sub_;
    ros::Publisher pubObjPosDist, pubCommand;
    image_transport::ImageTransport it_;
    image_transport::Subscriber it_sub_;
    image_transport::Publisher it_pub_;
    std::string ns_;
    VideoWriter outputVideo, outputVideo2;
    geometry_msgs::PoseStamped objPosDist_;
    std::mutex lock_;
    std::string windowName1 = "processed";
    std::string windowName2 = "pcaAnalysis";

public:
    objectTracker(ros::NodeHandle nh) : nh_(nh), it_(nh_)
    {
        ns_ = nh_.getNamespace();
        roi_sub_ = nh_.subscribe(ns_ + "/image_raw_roi",1,&objectTracker::roiCb,this);
        obj_roi_sub_ = nh_.subscribe(ns_ + "/image_raw_object_roi",1,&objectTracker::objectRoiCb,this);
        command_sub_ = nh_.subscribe(ns_ + "/goal_target", 1, &objectTracker::commandCb, this);
        it_ = image_transport::ImageTransport(nh_);
        it_sub_ = it_.subscribe(ns_ + "/image_raw",1,&objectTracker::imageCb, this);
        it_pub_ = it_.advertise(ns_ + "/image_processed",1);
        pubObjPosDist = nh_.advertise<geometry_msgs::PoseStamped>(ns_ + "/positionDist",1);
        pubCommand = nh_.advertise<geometry_msgs::PoseStamped>(ns_ + "/target_pose_command",1);

        namedWindow(windowName1, CV_WINDOW_NORMAL);
        namedWindow(windowName2, CV_WINDOW_NORMAL);
    }

    ~objectTracker()
    {
        destroyWindow(windowName1);
        destroyWindow(windowName2);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
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

        Point tempPoint_;

        frame = cv_ptr->image;
        frame.copyTo(procFrame);
        findObj(procFrame);
        if(isObjectFound)
        {
            frame.copyTo(pcaFrame);
            //Rect roi_ = roi + cv::Size_<double>(50,50);
            roi_ = roi;
            roi_.x = roi.x - 7.5;
            roi_.y = roi.y - 7.5;
            roi_.width = roi.width + 15;
            roi_.height = roi.height + 15;
            pcaFrame = Mat(pcaFrame, roi_);
            cvtColor(pcaFrame, pcaFrame, CV_BGR2HSV);
            //GaussianBlur(pcaFrame, pcaFrame, Size(5,5), 0,0);
            inRange(pcaFrame,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),pcaFrame);
            //morphOps(pcaFrame);
            std::vector<std::vector<Point> > contours_;
            std::vector<std::pair<double, int> > area;
            findContours(pcaFrame, contours_, RETR_LIST, CHAIN_APPROX_NONE);
            for(int i=0; i<contours_.size(); i++)
            {
                area.push_back(std::pair<double, int>(contourArea(contours_[i], false),i) );
                //drawContours(pcaFrame, contours_, static_cast<int>(i), Scalar(0,0,255),2,LINE_8);

            }
            if(!area.empty())
            {
                sort(area.begin(), area.end());
                int index = area.back().second;
                if(index >= 0)
                    angle = getOrientation(contours_[index], procFrame, tempPoint_) * 180.0 / M_PI;
                ROS_INFO("angle:%f", angle);
            }

            drawObject(tempPoint_.x, tempPoint_.y, procFrame);
            x_obj = static_cast<double>(-1920.0/2.0 + (offsetX + tempPoint_.x) );
            y_obj = static_cast<double>(-1200.0/2.0 + (offsetY + tempPoint_.y) );
            imshow(windowName2, pcaFrame);
        }

        //Object Position with disturbance
        if(isObjectFound)
        {
            if(kalmanInit)
            {
                kf.statePre.at<float>(0) = x_obj/scale;
                kf.statePre.at<float>(1) = y_obj/scale;
                kf.statePre.at<float>(2) = angle * M_PI / 180.0;
                setIdentity(kf.processNoiseCov, Scalar::all(1e-4));
                setIdentity(kf.measurementNoiseCov, Scalar::all(1e-2));
                setIdentity(kf.errorCovPost, Scalar::all(1e-2));
                kalmanInit = false;
            }

            prediction = kf.predict();
            predictPt = Point(prediction.at<float>(0), prediction.at<float>(1));
            posCur.x = static_cast<double>(predictPt.x * scale);
            posCur.y = static_cast<double>(predictPt.y * scale);

            measurement.at<float>(0) = x_obj/scale;
            measurement.at<float>(1) = y_obj/scale;
            measurement.at<float>(2) = angle * M_PI / 180.0;

            estimated = kf.correct(measurement);
            statePt = Point(estimated.at<float>(0), estimated.at<float>(1));
            circle(procFrame, statePt, 10, Scalar(255,50,0), 4);

            objPosDist_.pose.position.x = x_obj/scale * 0.07758621 *1e-3;
            objPosDist_.pose.position.y = y_obj/scale * 0.07758621 *1e-3;
            objPosDist_.pose.orientation = tf::createQuaternionMsgFromYaw(estimated.at<float>(2));
            objPosDist_.header.stamp = ros::Time::now();
            objPosDist_.header.frame_id = ns_;
            pubObjPosDist.publish(objPosDist_);

            angle = estimated.at<float>(2) * 180.0 / M_PI;
            Point p1 = Point((tempPoint_.x + 50*cos(angle * M_PI / 180.0)), (tempPoint_.y + 50*sin(angle * M_PI / 180.0)) );
            drawAxis(procFrame, tempPoint_, p1, Scalar(0,255,0), 1);
            objLocation.push_back(tempPoint_);

            for(int i=0; i<objLocation.size()-1; i++)
            	line(procFrame, objLocation[i], objLocation[i+1], Scalar(0,0,255), 1);
        }


        cv_ptr->image = procFrame;
        it_pub_.publish(cv_ptr->toImageMsg());
        imshow(windowName1, procFrame);
        waitKey(1);
    }

    void objectRoiCb(const sensor_msgs::RegionOfInterest& roiMsg)
    {
        roi = Rect(roiMsg.x_offset, roiMsg.y_offset, roiMsg.width, roiMsg.height);
        tracker->init(procFrame,roi);
        isObjectFound = true;
        kalmanInit = true;
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

            int64_t modOffsetX = static_cast<int64_t>(roiMsg.x_offset % 32 );
            if(modOffsetX > 32/2)
                modOffsetX = modOffsetX - 32;

            int64_t modOffsetY = static_cast<int64_t>(roiMsg.y_offset % 2 );
            if(modOffsetY > 2/2)
                modOffsetY = modOffsetY - 2;

            int frameWidth = static_cast<int>(roiMsg.width - modWith);
            int frameHeight = static_cast<int>(roiMsg.height - modHeight);
            offsetX = static_cast<int>(roiMsg.x_offset - modOffsetX);
            offsetY = static_cast<int>(roiMsg.y_offset - modOffsetY);

            ROS_INFO("**********roiCb***********");
            ROS_INFO("offsetX:%d", offsetX);
            ROS_INFO("offsetY:%d", offsetY);
            lock_.unlock();
        }
        else
            ROS_ERROR("mutex lock error");
    }

    void commandCb(const geometry_msgs::PointConstPtr &msg)
    {
        if(lock_.try_lock())
        {
            geometry_msgs::PoseStamped command;
            double x = static_cast<double>(-1920.0/2.0 + (offsetX + msg->x) ) * 0.07758621 *1e-3;
            double y = static_cast<double>(-1200.0/2.0 + (offsetY + msg->y) ) * 0.07758621 *1e-3;
            command.header.frame_id = ns_.c_str();
            command.header.stamp = ros::Time::now();
            command.pose.position.x = x;
            command.pose.position.y = y;
            command.pose.position.z = 0;
            command.pose.orientation.x = 0;
            command.pose.orientation.y = 0;
            command.pose.orientation.z = 0;
            command.pose.orientation.w = 1;
            ROS_INFO("Command X:%f, Y:%f", x, y);
            pubCommand.publish(command);
        lock_.unlock();
        }
    }
};

void dynamicReconfigureCallback(image_processing::hsvThreshConfig &config, uint32_t level)
{
        H_MAX = config.H_MIN;
        H_MIN = config.H_MAX;
        S_MAX = config.S_MAX;
        S_MIN = config.S_MIN;
        V_MAX = config.V_MAX;
        V_MIN = config.V_MIN;
}

int main(int argc, char *argv[]) {
    ros::init(argc,argv,"ObjectTracking");
    ros::NodeHandle nh;
    ros::TransportHints().tcpNoDelay();

    float dt = 1.0 / 10.0;
    kf.transitionMatrix.at<float>(0,0) = 1; //x
    kf.transitionMatrix.at<float>(1,1) = 1; //y
    kf.transitionMatrix.at<float>(2,2) = 1; //yaw

    kf.transitionMatrix.at<float>(0,3) = dt; //x_dot
    kf.transitionMatrix.at<float>(1,4) = dt; //y_dot
    kf.transitionMatrix.at<float>(2,5) = dt; //yaw_dot
    kf.transitionMatrix.at<float>(0,6) = 0.5*pow(dt, 2);
    kf.transitionMatrix.at<float>(1,7) = 0.5*pow(dt, 2);
    kf.transitionMatrix.at<float>(2,8) = 0.5*pow(dt, 2);

    kf.measurementMatrix.at<float>(0,0) = 1; //x
    kf.measurementMatrix.at<float>(1,1) = 1; //y
    kf.measurementMatrix.at<float>(2,2) = 1; //yaw

    const std::string ns = nh.getNamespace();
    dynamic_reconfigure::Server<image_processing::hsvThreshConfig> server;
    dynamic_reconfigure::Server<image_processing::hsvThreshConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    //Image Subscriber object
    objectTracker ot(nh);

    mouseIsDragging = false;
    mouseMove = false;
    rectangleSelected = false;

    ros::Rate loop_rate(10);
    double now;
    double previous = ros::Time::now().toSec();

    while(ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    destroyAllWindows();
    return 0;
}

void morphOps(Mat &image)
{
    erode(image, image, erodeElement);
    erode(image, image, erodeElement);

    dilate(image, image, dilateElement);
    dilate(image, image, dilateElement);

    erode(image, image, erodeElement);
    erode(image, image, erodeElement);
}

void findObj(Mat &image)
{
    tracker->update(image,roi);
    rectangle( image, roi, Scalar( 255, 0, 0 ), 2, 1 );
}

std::string intToString(int number){
	std::stringstream ss;
	ss << number;
	return ss.str();
}

void drawObject(int x, int y,Mat &frame){

	circle(frame,Point(x,y),20,Scalar(0,255,0),2);
    if(y-25>0)
    line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
    if(y+25<FRAME_HEIGHT)
    line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
    if(x-25>0)
    line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
    if(x+25<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

	putText(frame,intToString(x)+"|"+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);

}

double getOrientation(const std::vector<Point>& pts, Mat& img, Point& centerObj)
{
    Mat data_pts = Mat(pts.size(), 2, CV_64FC1);
    for(int i=0; i < data_pts.rows; i++)
    {
        data_pts.at<double>(i,0) = pts[i].x;
        data_pts.at<double>(i,1) = pts[i].y;
    }

    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
    Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0,0)), static_cast<int>(pca_analysis.mean.at<double>(0,1)) );


    for(int i=0; i<2; i++)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i,0), pca_analysis.eigenvectors.at<double>(i,1) );
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0,i);
    }

    Point robotCenter = cntr + Point(roi_.x, roi_.y);
    centerObj = robotCenter;

    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians

    return angle;
}

void drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale = 0.2)
{
    double angle;
    double hypotenuse;
    angle = atan2( (double) p.y - q.y, (double) p.x - q.x ); // angle in radians
    hypotenuse = sqrt( (double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));

    q.x = (int) (p.x - scale * hypotenuse * cos(angle));
    q.y = (int) (p.y - scale * hypotenuse * sin(angle));
    line(img, p, q, colour, 1, CV_AA);
    // create the arrow hooks
    p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
    line(img, p, q, colour, 1, CV_AA);
    p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
    line(img, p, q, colour, 1, CV_AA);
}
