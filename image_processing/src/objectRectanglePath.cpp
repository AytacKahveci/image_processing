/* @author Aytaç Kahveci */
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <dynamixel_custom_controller/dynamixel_class.h>

#include <ros/ros.h>

#include <object_position_msgs/ObjectPositionMsg.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iostream>
#include <math.h>
#include <mutex>

using namespace message_filters;
using namespace object_position_msgs;

#define FRONT_CAMERA_WIDTH 1920
#define FRONT_CAMERA_HEIGHT 1200
#define BOTTOM_CAMERA_WIDTH 1920
#define BOTTOM_CAMERA_HEIGHT 1200

#define PI 3.1415

double center_bottom_x, center_bottom_y;
double center_front_x, center_front_y;
double center_x, center_y, center_z;
double rb = 0.0, rf = 0.0;
double scale_bottom = 0.107758621, scale_front = 0.1;

bool center = false;
bool init = false;
bool vectorUpdate = false;
std_msgs::Float64MultiArray cartPos, motorPos;
std_msgs::Float64 vel;
ros::Publisher pubKukaCart, pubDynamixel, pubKukaVel;
dynamixel_class::DynamixelClass* dynamixel;
double previous_time, current_time;
double threshold = 1.5;

//<kuka_vars>
float A_ = 90; //unit degree
float B_ = 0; //unit degree
float C_ = 180; //unit degree
float roll_ = 0;
float movingRes = 0.03; //unit mm
float movingThresh = 0.2; //unit mm
float kukaVel = 0.5; //unit m/s
float z_offset = 7; //unit mm
//</kuka_vars>

double x,y,z,front_y;
double commandKukaX, commandKukaY, commandKukaZ = 0.0;
double deltaX, deltaY, deltaZ = 0.0;
double refx, refy = 0.0;
bool condition = false;

//Dynamixel variables
bool epsilon = true;
bool endEpsilon = false;
double motorRes1 = 78.615;
double motorRes2 = 76.92;

std::mutex lock_;
bool flag, flag2 = false;
int frameWidth;
int frameHeight;
int offsetX = 0;
int offsetY = 0;

//<Polygon variables>
double n = 4; //polygon #edges
double angle = 0.0; //degree
double angleInc = 360 / n; //degree
double radius = 22; //mm
double lRef = radius * sin((angleInc / 2.0) * PI / 180.0); //polygon edge length
double l = 0.0;
bool nextPoint = false;
double edgeX, edgeY = 0.0;
//</Polygon vaiables>

Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az)
{
  Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rx * ry * rz;
}

void callback(const ObjectPositionMsgConstPtr& obj_xy, const sensor_msgs::JointStateConstPtr& kuka)
{
    /*ROS_INFO("X pos: %f, Y pos: %f", obj_xy->position.data[0], obj_xy->position.data[1]);
    ROS_INFO("Y pos: %f, Z pos: %f", obj_yz->position.data[0], obj_yz->position.data[1]);
    ROS_INFO("Kuka X:%f, Y:%f, Z:%f", kuka->position[0], kuka->position[1], kuka->position[2]);*/

    x = obj_xy->position.data[0]*scale_bottom; //unit mm
    y = obj_xy->position.data[1]*scale_bottom; //unit mm


    static const double initKukaX = kuka->position[0];
    static const double initKukaY = kuka->position[1];
    static const double initKukaZ = kuka->position[2];

    if(init)
    {
        //Check whether we are on the vertex or the edge
        //ROS_INFO("edgeX:%f, edgeY:%f", edgeX, edgeY);
        l = sqrt(pow(edgeX - x, 2) + pow(edgeY - y, 2));
        ROS_INFO("l: %f, lref: %f", l, lRef);
        if(l < lRef)
        {
            nextPoint = false;
        }
        else
        {
            nextPoint = true;
            //Update the edge position
            edgeX = x;
            edgeY = y;
        }

        if(std::abs(y - center_y) > movingThresh) // X axis unit mm
        {
            deltaX += (y - center_y); //total distance travelled by microrobot /unit mm
            commandKukaX = initKukaX - deltaX;
            cartPos.data[0] = commandKukaX; //unit mm
            center_y = y;
            condition = true;
        }
        else
            cartPos.data[0] = initKukaX - deltaX; //unit mm

        if(std::abs(x - center_x) > movingThresh) // Y axis unit mm
        {
            deltaY += (x - center_x); //total distance travelled by microrobot /unit mm
            commandKukaY = initKukaY + deltaY;
            cartPos.data[1] = commandKukaY; //unit mm
            center_x = x;
            condition = true;
        }
        else
            cartPos.data[1] = initKukaY + deltaY; //unit mm

        cartPos.data[2] = initKukaZ; // Z axis unit mm

        if(nextPoint) //rotate
        {
            angle += angleInc;
            vel.data = 0.150;
            pubKukaVel.publish(vel);
            cartPos.data[3] = A_ + angle;

            motorPos.data[0] = dynamixel->position[0]; //mm
            motorPos.data[1] = dynamixel->position[1] + 10 * motorRes2; //mm
            motorPos.data[2] = 5;
            motorPos.data[3] = 5;
            pubDynamixel.publish(motorPos);

            previous_time = ros::Time::now().toSec();
            condition = true;
            epsilon = true;
            nextPoint = false;
        }
        else //Give epsilon for movement
        {
            current_time = ros::Time::now().toSec();
            if((current_time - previous_time) > threshold && epsilon)
            {
                motorPos.data[0] = dynamixel->position[0]; //mm
                motorPos.data[1] = dynamixel->position[1] + 10 * motorRes2; //mm
                motorPos.data[2] = 5;
                motorPos.data[3] = 5;
                pubDynamixel.publish(motorPos);
                epsilon = false;
            }
            vel.data = 0.075;
            pubKukaVel.publish(vel);
            cartPos.data[3] = A_ + angle;
        }

        if(condition)
        {
            cartPos.data[2] = initKukaZ;
            //cartPos.data[4] = B_;
            cartPos.data[4] = - roll_;
            //cartPos.data[5] = C_ - roll_;
            cartPos.data[5] = C_;
            pubKukaCart.publish(cartPos);
            condition = false;
        }
    }

    if(center)
    {
        //center of the object in the image plane /unit mm
        center_x = x;
        center_y = y;
        center_z = z;
        edgeX = x;
        edgeY = y;
        roll_ = 0.0;
        motorPos.data[0] = dynamixel->position[0]; //mm
        motorPos.data[1] = dynamixel->position[1] + 10 * motorRes2; //mm
        motorPos.data[2] = 5;
        motorPos.data[3] = 5;
        pubDynamixel.publish(motorPos);
        ROS_INFO("*****center_x:%f, center_y:%f, center_z:%f",center_x, center_y, center_z);
        center = false;
        init = true;
        vel.data = 0.01;
        pubKukaVel.publish(vel);
    }
}

void centerCb(const std_msgs::Bool& flag)
{
    center = flag.data;
    ROS_INFO("********Center command is executed***********");
}

void roiCb(const sensor_msgs::RegionOfInterest& roiMsg)
{
    if(lock_.try_lock())
    {
        int64_t modOffsetX = static_cast<int64_t>(roiMsg.x_offset % 32 );
        if(modOffsetX > 32/2)
            modOffsetX = modOffsetX - 32;

        int64_t modOffsetY = static_cast<int64_t>(roiMsg.y_offset % 2 );
        if(modOffsetY > 2/2)
            modOffsetY = modOffsetY - 2;
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

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "kuka_command_node");

    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    pubKukaCart = nh.advertise<std_msgs::Float64MultiArray>("/joint_position_controller/command",1);
    pubKukaVel = nh.advertise<std_msgs::Float64>("/TCP_velocity_command/command",1);
    pubDynamixel = nh.advertise<std_msgs::Float64MultiArray>("/dynamixel/JointPosGroup/poseffCommand",1);
    ros::Subscriber roi_sub_ = nh.subscribe("/camera_node_bottom/image_raw_roi",1,&roiCb);
    ros::Subscriber subCenter = nh.subscribe("/center",1,&centerCb);

    dynamixel = new dynamixel_class::DynamixelClass(nh, 2);
    cartPos.data.resize(6);
    motorPos.data.resize(4);

    message_filters::Subscriber<ObjectPositionMsg> xySub(nh, "/camera_node_bottom/positionDist", 1);
    //message_filters::Subscriber<ObjectPositionMsg> yzSub(nh, "/camera_node_front/positionDist", 1);
    message_filters::Subscriber<sensor_msgs::JointState> kukaSub(nh, "/joint_states", 1);
    //typedef sync_policies::ApproximateTime<ObjectPositionMsg, ObjectPositionMsg, sensor_msgs::JointState> MySyncPolicy;
    typedef sync_policies::ApproximateTime<ObjectPositionMsg, sensor_msgs::JointState> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(14), xySub, kukaSub);
    //TimeSynchronizer<ObjectPositionMsg, ObjectPositionMsg> sync(xySub, yzSub, 3);

    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::spin();
    return 0;
}
