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

#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <kuka_hw_cart_vel/kuka_class.h>
#include <dynamixel_custom_controller/dynamixel_class.h>
#include <object_position_msgs/ObjectPositionMsg.h>

using namespace cv;

/*************** Image Processing Variables **************/
Mat frame, hsvImage, hsvImageThresh, edges, mask, objBw;
Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3,3) );
Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8,8) );

/*const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;*/
const int FRAME_WIDTH = 1920;
const int FRAME_HEIGHT = 1200;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 40*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

int x;
int y;
double x_prev = 0;
double x_prev2 = 0;
double y_prev = 0;
double x_obj = 0;
double y_obj = 0;
double edgeX = 0;
double edgeY = 0;
double scale = 1;

#define PI 3.141592654

int H_MIN = 147;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
std::vector<int> H_ROI;
std::vector<int> S_ROI;
std::vector<int> V_ROI;
std::vector<Mat> channels;

const std::string windowName = "Source Image";
const std::string windowName2 = "HSV Image";
const std::string windowName3 = "Thresholded Image";
const std::string trackbarWindow = "HSV Configurations";

bool mouseIsDragging = false;
bool mouseMove = false;
bool rectangleSelected = false;
bool refMouseMove = false;
Rect rectangleROI;
Point currentPoint;
Point initialPoint;
Point initRefPoint, finalRefPoint;
bool initRefSelected = false;
bool active = false;
//Tracker variables
Rect2d roi;
Ptr<TrackerBoosting> tracker = TrackerBoosting::create();
Mat maskFrame, procFrame;
std::vector<Point> objLocation;

//Kalman variables
KalmanFilter kf(4, 2, 0);
Mat measurement = Mat::zeros(2, 1, CV_32F);
Mat prediction, estimated;
Point2f predictPt, statePt, measPt;
Point2f posCur, posPrev;
int count = 0;
bool init, kalmanInit = true;
bool isObjectFound = false;

//Polygon variables
double n = 18; //polygon #edges
double angle = 0.0; //degree
double angleInc = 360 / n; //degree
double radius = 1; //mm
double lRef = 2.0 * radius * sin((angleInc / 2.0) * PI / 180.0); //polygon edge length
double l = 0.0;
bool nextPoint = false;

//Dynamixel variables
bool epsilon = true;
bool endEpsilon = false;
double motorRes1 = 78.615;
double motorRes2 = 76.92;

///hsvTrackbar is called whenever the trackbar position is changed
void hsvTrackbar(int, void*){}
void morphOps(Mat &image);
void findObj(Mat &,int&,int&);
void findObj(Mat& image);
void drawObject(int x, int y,Mat &frame);
std::string intToString(int number);
void mouseCallBack(int event, int x, int y, int flags, void* userdata);
void recordHSV(Mat& frame, Mat& HSV);
void rotateImage(Mat& src, double angle);
void drawAxis(Mat&, Point, Point, Scalar, const float);
double getOrientation(const std::vector<Point> & pts, Mat& image);
/*************** /Image Processing Variables **************/

/******** ROS variables *************/
//<kuka_vars>
float A_ = 90; //unit degree
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
    image_transport::ImageTransport it_;
    image_transport::Subscriber it_sub_;
    std::string ns_;

public:
    objectTracker(ros::NodeHandle nh) : nh_(nh), it_(nh_)
    {
        ns_ = nh_.getNamespace();
        it_ = image_transport::ImageTransport(nh_);
        it_sub_ = it_.subscribe(ns_ + "/image_raw",1,&objectTracker::imageCb, this);
    }

    ~objectTracker()
    {
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

        frame = cv_ptr->image;
        GaussianBlur(frame,frame,Size(3,3),0,0);
        cvtColor(frame, hsvImage, CV_BGR2HSV);
        GaussianBlur(hsvImage, hsvImage, Size(5,5), 0,0);
        //recordHSV(frame, hsvImage);
        inRange(hsvImage,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),hsvImageThresh);
        //bitwise_and(hsvImageThresh, mask, hsvImageThresh);
        morphOps(hsvImageThresh);
        cvtColor(hsvImageThresh, maskFrame, COLOR_GRAY2BGR);
        bitwise_and(frame, maskFrame, procFrame);
        //objBw = hsvImageThresh(roi);
        //findObj(objBw,x,y); //object is found
        findObj(procFrame);

        for(int i=0; i<objLocation.size()-1; i++)
            line(procFrame, objLocation[i], objLocation[i+1], Scalar(0,0,255), 1);
        circle(procFrame, statePt, 10, Scalar(255,50,0), 4);
        imshow(windowName, frame);
        imshow(windowName2, procFrame);
        imshow(windowName3, hsvImageThresh);
        waitKey(1);
    }
};

int main(int argc, char *argv[]) {
    ros::init(argc,argv,"ObjectTracking");
    ros::NodeHandle nh;
    ros::TransportHints().tcpNoDelay();
    ros::Publisher pubObjPos;
    ros::Publisher pubObjPosDist;
    ros::Publisher pubKukaCart;
    ros::Publisher pubKukaVel;
    ros::Publisher pubDynamixel;

    const std::string ns = nh.getNamespace();
    pubObjPos = nh.advertise<object_position_msgs::ObjectPositionMsg>(ns + "/position",1);
    pubObjPosDist = nh.advertise<object_position_msgs::ObjectPositionMsg>(ns + "/positionDist",1);
    pubKukaCart = nh.advertise<std_msgs::Float64MultiArray>("/joint_position_controller/command",1);
    pubKukaVel = nh.advertise<std_msgs::Float64>("/TCP_velocity_command/command",1);
    pubDynamixel = nh.advertise<std_msgs::Float64MultiArray>("/dynamixel/JointPosGroup/poseffCommand",1);

    object_position_msgs::ObjectPositionMsg objPos_;
    object_position_msgs::ObjectPositionMsg objPosDist_;
    objPos_.position.data.resize(2);
    objPosDist_.position.data.resize(2);

    std_msgs::Float64MultiArray cartPos_;
    std_msgs::Float64 vel_;
    cartPos_.data.resize(6);

    std_msgs::Float64MultiArray motorPos_;
    motorPos_.data.resize(4);

    //Image Subscriber object
    objectTracker ot(nh);
    //Kuka Pos Subscriber object
    kuka_class::kuka robot(nh, 6);
    //Dynamixel Pos Subscriber object
    dynamixel_class::DynamixelClass motor(nh, 2);

    kf.transitionMatrix = (Mat_<float>(4,4) << 1,0,1,0, 0,1,0,1, 0,0,1,0, 0,0,0,1);

    namedWindow(windowName, CV_WINDOW_NORMAL);
    namedWindow(windowName2, CV_WINDOW_NORMAL);
    //Thresholded image window is also used for HSV trackbar
    namedWindow(windowName3, CV_WINDOW_NORMAL);

    createTrackbar("H_MIN", windowName3, &H_MIN, H_MAX, hsvTrackbar );
    createTrackbar("H_MAX", windowName3, &H_MAX, H_MAX, hsvTrackbar );
    createTrackbar("S_MIN", windowName3, &S_MIN, S_MAX, hsvTrackbar );
    createTrackbar("S_MAX", windowName3, &S_MAX, S_MAX, hsvTrackbar );
    createTrackbar("V_MIN", windowName3, &V_MIN, V_MAX, hsvTrackbar );
    createTrackbar("V_MAX", windowName3, &V_MAX, V_MAX, hsvTrackbar );

    setMouseCallback(windowName2, mouseCallBack, &frame);

    mouseIsDragging = false;
    mouseMove = false;
    rectangleSelected = false;

    ros::Rate loop_rate(30);
    double now;
    double previous = ros::Time::now().toSec();

    ROS_INFO("length of an edge is %f", lRef);
    while(ros::ok())
    {
        ros::spinOnce();

        if(isObjectFound)
        {
            if(init)
            {
                if(kalmanInit)
                {
                    kf.statePre.at<float>(0) = x_obj/scale;
                    kf.statePre.at<float>(1) = y_obj/scale;
                    kf.statePre.at<float>(2) = 0;
                    kf.statePre.at<float>(3) = 0;
                    setIdentity(kf.measurementMatrix);
                    setIdentity(kf.processNoiseCov, Scalar::all(1e-4));
                    setIdentity(kf.measurementNoiseCov, Scalar::all(.5));
                    setIdentity(kf.errorCovPost, Scalar::all(0.1));
                    ROS_INFO("Kalman init");
                }

                if(count < 10)
                {
                    prediction = kf.predict();
                    predictPt = Point(prediction.at<float>(0), prediction.at<float>(1));

                    measurement.at<float>(0,0) = x_obj/scale;
                    measurement.at<float>(1,0) = y_obj/scale;

                    estimated = kf.correct(measurement);
                    statePt = Point(estimated.at<float>(0,0), estimated.at<float>(1,0));

                    kalmanInit = false;
                    count++;
                }
                else
                {
                    /*Initial command will be published
                    to the robot*/
                    init = false;
                    //ROS_INFO("Kalman first exec");
                }
            }
            else
            {
                prediction = kf.predict();
                predictPt = Point(prediction.at<float>(0), prediction.at<float>(1));
                posCur.x = static_cast<double>(predictPt.x * scale);
                posCur.y = static_cast<double>(predictPt.y * scale);

                ROS_INFO("x: %f,y: %f", posCur.x, posCur.y);
                if(active)
                {
                    l = sqrt(pow(edgeX - posCur.x, 2) + pow(edgeY - posCur.y, 2)); //length of the polygon(mm)
                    //ROS_INFO("length l: %f lRef: %f", l, lRef);
                    if(l < lRef)
                    {
                        nextPoint = false;
                    }
                    else
                    {
                        nextPoint = true;
                        edgeX = posCur.x;
                        edgeY = posCur.y;
                    }

                    if(std::abs(posCur.x - x_prev) > 0.01 || std::abs(posCur.y - y_prev) > 0.01)
                    {
                        ROS_INFO("length l: %f lRef: %f", l, lRef);
                    if(!nextPoint)
                    {
                        if(epsilon)
                        {
                            motorPos_.data[0] = motor.position[0]; //mm
                            motorPos_.data[1] = motor.position[1] - 10 * motorRes2; //mm
                            motorPos_.data[2] = 5;
                            motorPos_.data[3] = 5;
                            pubDynamixel.publish(motorPos_);
                            epsilon = false;
                        }
                        if(endEpsilon)
                        {
                            motorPos_.data[0] = motor.position[0]; //mm
                            motorPos_.data[1] = motor.position[1] + 10 * motorRes2; //mm
                            motorPos_.data[2] = 5;
                            motorPos_.data[3] = 5;
                            pubDynamixel.publish(motorPos_);
                            endEpsilon = false;
                        }
                        vel_.data = 0.075;
                        pubKukaVel.publish(vel_);
                        cartPos_.data[0] = robot.position[0] - (posCur.y - y_prev) * 100.0 / 100.0; //X
                        cartPos_.data[1] = robot.position[1] + (posCur.x - x_prev) * 100.0 / 100.0; //Y
                        cartPos_.data[2] = 0; //Z
                        cartPos_.data[3] = A_ + angle;
                        cartPos_.data[4] = B_;
                        cartPos_.data[5] = C_;
                        pubKukaCart.publish(cartPos_);
                    }
                    else //rotate
                    {
                        angle += angleInc;
                        vel_.data = 0.150;
                        pubKukaVel.publish(vel_);
                        cartPos_.data[0] = robot.position[0] - (posCur.y - y_prev) * 100.0 / 100.0; //X
                        cartPos_.data[1] = robot.position[1] + (posCur.x - x_prev) * 100.0 / 100.0; //Y
                        cartPos_.data[2] = 0; //Z
                        cartPos_.data[3] = A_ + angle;
                        cartPos_.data[4] = B_;
                        cartPos_.data[5] = C_;
                        motorPos_.data[0] = motor.position[0]; //mm
                        motorPos_.data[1] = motor.position[1] + 10 * motorRes2; //mm
                        motorPos_.data[2] = 5;
                        motorPos_.data[3] = 5;
                        pubDynamixel.publish(motorPos_);
                        pubKukaCart.publish(cartPos_);
                        usleep(1500000);
                        epsilon = true;
                    }

                    x_prev = posCur.x;
                    y_prev = posCur.y;
                    ROS_INFO("condition");
                    }
                }
                measurement.at<float>(0,0) = x_obj/scale;
                measurement.at<float>(1,0) = y_obj/scale;

                estimated = kf.correct(measurement);
                statePt = Point(estimated.at<float>(0,0), estimated.at<float>(1,0));
                //ROS_INFO("Kalman filter");
                //Object Position with disturbance
                objPosDist_.position.data[0] = x_obj;
                objPosDist_.position.data[1] = y_obj;
                objPosDist_.header.stamp = ros::Time::now();
                pubObjPosDist.publish(objPosDist_);

                //ros::spinOnce();
                loop_rate.sleep();
            }
        }

    }

    destroyAllWindows();
    return 0;
}

void morphOps(Mat &image)
{
    erode(image, image, erodeElement);
    erode(image, image, erodeElement);
    erode(image, image, erodeElement);
    erode(image, image, erodeElement);

    dilate(image, image, dilateElement);
    dilate(image, image, dilateElement);

    erode(image, image, erodeElement);
    erode(image, image, erodeElement);

    dilate(image, image, dilateElement);
    dilate(image, image, dilateElement);
    erode(image, image, erodeElement);
    erode(image, image, erodeElement);
    erode(image, image, erodeElement);
    erode(image, image, erodeElement);
}

void findObj(Mat &image, int& x, int& y)
{
    std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    findContours(image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    double refArea = 0;

    if(hierarchy.size() >0)
    {
        int numObjects = hierarchy.size();
        if(numObjects < MAX_NUM_OBJECTS)
        {
            for(int i=0; i>=0; i=hierarchy[i][0] )
            {
                Moments moment = moments((cv::Mat)contours[i]);
                double area = moment.m00;

                if(area > MIN_OBJECT_AREA && area < MAX_OBJECT_AREA && area>refArea)
                {
                    double angle = getOrientation(contours[i], image);

                }
                else{
                    putText(frame, "TOO MUCH NOISE! ADJUST FILTER", Point(0,50),1,2,Scalar(0,0,255),2);
                }
            }
        }
    }
}

void findObj(Mat &image)
{
    tracker->update(image,roi);
    rectangle( image, roi, Scalar( 255, 0, 0 ), 2, 1 );
    drawObject(roi.x+roi.width/2, roi.y+roi.height/2, image);
    objLocation.push_back(Point(static_cast<int>(roi.x+roi.width/2),static_cast<int>(roi.y+roi.height/2)) );
    x_obj = static_cast<double>(roi.x + static_cast<double>(roi.width/2.0) ) * scale;
    y_obj = static_cast<double>(roi.y + static_cast<double>(roi.height/2.0) ) * scale;
}

std::string intToString(int number){
	std::stringstream ss;
	ss << number;
	return ss.str();
}

void drawObject(int x, int y,Mat &frame){

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

    //UPDATE:JUNE 18TH, 2013
    //added 'if' and 'else' statements to prevent
    //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

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

void mouseCallBack(int event, int x, int y, int flags, void* userdata)
{
    Mat* pimage = (Mat*)userdata;
    if(flags != (EVENT_FLAG_CTRLKEY+EVENT_FLAG_LBUTTON) && event == EVENT_LBUTTONDOWN && mouseIsDragging == false && rectangleSelected == false)
    {
        initialPoint = Point(x,y);
        printf("Initial point x:%d,y:%d \n",x,y);
        mouseIsDragging = true;
    }
    if(event == EVENT_MOUSEMOVE && mouseIsDragging == true && rectangleSelected == false)
    {
        currentPoint = Point(x,y);
        printf("Current point x:%d,y:%d \n",x,y);
        mouseMove = true;
    }
    if(event == EVENT_LBUTTONUP && mouseIsDragging == true)
    {
        rectangleROI = Rect(initialPoint, currentPoint);
        mouseIsDragging = false;
        mouseMove = false;
        rectangleSelected = true;
        roi = rectangleROI;
        cvtColor(hsvImageThresh, maskFrame, COLOR_GRAY2BGR);
        bitwise_and(frame, maskFrame, procFrame);
        tracker->init(procFrame,roi);

        isObjectFound = true;
        init = true;
        kalmanInit = true;
    }

    if(flags == (EVENT_FLAG_CTRLKEY+EVENT_FLAG_LBUTTON)  && event == EVENT_LBUTTONDOWN && !initRefSelected)
    {
        initRefPoint = Point(x,y);
        initRefSelected = true;
        printf("Initial point is selected x:%d,y%d\n",x,y );
    }

    if(flags == (EVENT_FLAG_CTRLKEY+EVENT_FLAG_LBUTTON)  && event == EVENT_MOUSEMOVE && initRefSelected)
    {
        finalRefPoint = Point(x,y);
        line(frame, initRefPoint, finalRefPoint, Scalar(0,0,255),5,8);
        printf("Final point is selected x:%d,y%d\n",x,y );
        refMouseMove = true;
    }

    if(flags == (EVENT_FLAG_CTRLKEY+EVENT_FLAG_LBUTTON)  && event == EVENT_LBUTTONUP && initRefSelected)
    {
        initRefSelected = false;
        refMouseMove = false;
        //scale = 50.0 / std::abs(initRefPoint.x - finalRefPoint.x);
        scale = 0.014045;
        printf("scale is %f\n", scale);
    }

    if(flags == (EVENT_FLAG_SHIFTKEY+EVENT_FLAG_LBUTTON) && event == EVENT_LBUTTONDOWN)
    {
        x_prev = x_obj;
        y_prev = y_obj;
        edgeX = x_obj;
        edgeY = y_obj;
        angle = 0;
        ROS_INFO("********x_prev is %f *********", x_prev);
        ROS_INFO("********y_prev is %f *********", x_prev);
        ROS_INFO("********edgeX is %f *********", edgeX);
        ROS_INFO("********edgeY is %f *********", edgeY);
        ROS_INFO("********angle is %f *********", angle);
        nextPoint = false;
        active = true;
    }

    if (event == EVENT_RBUTTONDOWN) {// if right button of mouse is clicked, reset H,S,V values.

			rectangleSelected = false;
			std::cout << "right button pressed" << std::endl;
		}
}

void recordHSV(Mat& frame, Mat& HSV)
{
    if(mouseMove == false && rectangleSelected == true)
    {
        for(int i=rectangleROI.x; i < rectangleROI.x + rectangleROI.width; i++)
        {
            for(int j=rectangleROI.y; j < rectangleROI.y + rectangleROI.width; j++)
            {
                H_ROI.push_back((int)HSV.at<Vec3b>(i,j)[0]);
                S_ROI.push_back((int)HSV.at<Vec3b>(i,j)[1]);
                V_ROI.push_back((int)HSV.at<Vec3b>(i,j)[2]);
            }
        }
        //H_MIN = *std::min_element(H_ROI.begin(), H_ROI.end());
        //H_MAX = *std::max_element(H_ROI.begin(), H_ROI.end());
        std::cout << "MIN 'H' VALUE: " << H_MIN << std::endl;
		std::cout << "MAX 'H' VALUE: " << H_MAX << std::endl;
        //S_MIN = *std::min_element(S_ROI.begin(), S_ROI.end());
        //S_MAX = *std::max_element(S_ROI.begin(), S_ROI.end());
        std::cout << "MIN 'S' VALUE: " << S_MIN << std::endl;
		std::cout << "MAX 'S' VALUE: " << S_MAX << std::endl;
        V_MIN = *std::min_element(V_ROI.begin(), V_ROI.end());
        V_MAX = *std::max_element(V_ROI.begin(), V_ROI.end());
        std::cout << "MIN 'V' VALUE: " << V_MIN << std::endl;
		std::cout << "MAX 'V' VALUE: " << V_MAX << std::endl;
        rectangleSelected = false;
    }
    if (mouseMove == true)
    {
        rectangle(frame, initialPoint, currentPoint, Scalar(0, 255, 0) );
    }
    if(refMouseMove)
    {
        line(frame, initRefPoint, finalRefPoint, Scalar(0,0,255),5,8);
    }
}

void rotateImage(Mat& src, double angle)
{
    Point2f center(src.cols/2, src.rows/2);
    Mat rotationMatrix = getRotationMatrix2D(center, angle, 1.0);
    warpAffine(src, src, rotationMatrix, src.size(), cv::INTER_CUBIC);
}

double getOrientation(const std::vector<Point>& pts, Mat& img)
{
    Mat data_pts = Mat(pts.size(), 2, CV_64FC1);
    for(int i=0; i < data_pts.rows; i++)
    {
        data_pts.at<double>(i,0) = pts[i].x;
        data_pts.at<double>(i,1) = pts[i].y;
    }

    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
    Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0,0)), static_cast<int>(pca_analysis.mean.at<double>(0,1)) );

    std::vector<Point2d> eigen_vecs(2);
    std::vector<double> eigen_val(2);
    for(int i=0; i<2; i++)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i,0), pca_analysis.eigenvectors.at<double>(i,1) );
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0,i);
    }

    circle(img, cntr, 3, Scalar(255,0,255), 2);
    Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]) );
    Point p2 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]) );

    drawAxis(img, cntr, p1, Scalar(0, 255, 0), 1);
    drawAxis(img, cntr, p2, Scalar(255, 255, 0), 5);
    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians

    return angle;
}

void drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale = 0.2)
{
    double angle;
    double hypotenuse;
    angle = atan2( (double) p.y - q.y, (double) p.x - q.x ); // angle in radians
    hypotenuse = sqrt( (double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
//    double degrees = angle * 180 / CV_PI; // convert radians to degrees (0-180 range)
//    cout << "Degrees: " << abs(degrees - 180) << endl; // angle in 0-360 degrees range
    // Here we lengthen the arrow by a factor of scale
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
