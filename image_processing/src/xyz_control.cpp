/* @author Aytaç Kahveci */
/*
5DOF microrobot control
*/
#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <angles/angles.h>
#include <kuka_hw_cart_vel/kuka_class.h>
#include <dynamixel_custom_controller/dynamixel_class.h>
#include <control_toolbox/pid.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <teme_msgs/PoseArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <queue>
#include <map>

#include <boost/thread.hpp>
#include <unistd.h>

#include <fstream>
#include <iostream>

double X_ = 0.0;
double Y_ = 0.0;
double Z_ = 7.2;

double A_ = 90.0;
double B_ = 0.0;
double C_ = 180.0;

//Dynamixel Variables
double motorRes1 = 78.615;
double motorRes2 = 76.92;

double yaw_goal_tolerance = 5;
double direction_goal_tolarence = 0.005;
double xy_goal_tolerance = 0.005;

bool init = true;

enum Status
{
    ROTATING_TO_GOAL = 1,
    REACHING_TO_GOAL = 2,
    REACHING_TO_ORIENTATION = 3,
    GOAL_REACHED = 4,
    YZ_PLANE = 5,
    XZ_PLANE = 6,
    HELIS = 7,
    CONSTRAINT = 8
};

std::string goal_pose_location;

namespace object_positioning
{

class ObjectPositioning
{

public:

    ObjectPositioning(ros::NodeHandle& nh) : nh_(nh)
    {
        ns_ = nh_.getNamespace();
        sub_ = nh_.subscribe("/goalPose", 1, &ObjectPositioning::orientationCB, this);
        obj_sub_bottom_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, ns_ + "camera_node_bottom/positionDist", 1);
        obj_sub_front_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, ns_ + "camera_node_front/positionDist", 1);

        obj_filter_bottom_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(*obj_sub_bottom_, listener_, end_effector_frame_, 1);
        obj_filter_front_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(*obj_sub_front_, listener_, kuka_fixed_frame_, 1);

        sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *obj_filter_bottom_, *obj_filter_front_);

        sync->registerCallback(boost::bind(&ObjectPositioning::poseCB, this, _1, _2));

        kuka_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/joint_position_controller/command",1);
        vis_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/microrobot/pose",1);
        conj_vis_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/microrobot/pose_conj",1);
        dynamixel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/dynamixel/JointPosGroup/poseffCommand",1);

        kuka_command_.data.resize(6);
        dynamixel_command_.data.resize(4);

        dynamixel_states_.reset(new dynamixel_class::DynamixelClass(nh_, 2));
        kuka_states_.reset(new kuka_class::kuka(nh_, 6));

        if(!nh_.getParam("xy_goal_tolerance", xy_goal_tolerance))
        {
            xy_goal_tolerance = 2;
            ROS_WARN("xy_goal_tolarence couldn't find in the param server. \
                       default is used: %f, ns: %s", xy_goal_tolerance, nh_.getNamespace().c_str());
        }
        if(!nh_.getParam("direction_goal_tolarence", direction_goal_tolarence))
        {
            direction_goal_tolarence = 2;
            ROS_WARN("direction_goal_tolarence couldn't find in the param server. \
                       default is used: %f, ns: %s", direction_goal_tolarence, nh_.getNamespace().c_str());
        }
        if(!nh_.getParam("yaw_goal_tolerance", yaw_goal_tolerance))
        {
            yaw_goal_tolerance = 2;
            ROS_WARN("yaw_goal_tolerance couldn't find in the param server. \
                       default is used: %f, ns: %s", yaw_goal_tolerance, nh_.getNamespace().c_str());
        }
    }

    ~ObjectPositioning()
    {}

    void poseCB(const boost::shared_ptr<const geometry_msgs::PoseStamped>& msg, const boost::shared_ptr<const geometry_msgs::PoseStamped>& msg2)
    {
        static const double initDynamixelId1 = dynamixel_states_->position[0];
        static const double initDynamixelId2 = dynamixel_states_->position[1];
        double initArray[2] = {initDynamixelId1, initDynamixelId2};
        static const double kuka_init_z = kuka_states_->position[2];

        ROS_WARN("dynamixel_state1:%f, dynamixel_state2:%f", initDynamixelId1, initDynamixelId2);

        try
        {
            listener_.transformPose(kuka_fixed_frame_, *msg, obj_fixed_frame);
            listener_.transformPose(kuka_fixed_frame_, *msg2, obj_fixed_frame2);

            tf::Matrix3x3 fixed(tf::Quaternion(obj_fixed_frame.pose.orientation.x, \
                                            obj_fixed_frame.pose.orientation.y, \
                                            obj_fixed_frame.pose.orientation.z, \
                                            obj_fixed_frame.pose.orientation.w));

            fixed.getEulerYPR(yaw_bottom_fixed_, pitch_bottom_fixed_, roll_bottom_fixed_);

            if(yaw_bottom_fixed_ < 0)
                yaw_bottom_fixed_ = fmod(2*M_PI + yaw_bottom_fixed_ + M_PI, 2*M_PI);
            conj_yaw_bottom_fixed_ = fmod(-180 + yaw_bottom_fixed_ * 180.0 / M_PI , 360.0);


            fixed = tf::Matrix3x3(tf::Quaternion(obj_fixed_frame2.pose.orientation.x, \
                                            obj_fixed_frame2.pose.orientation.y, \
                                            obj_fixed_frame2.pose.orientation.z, \
                                            obj_fixed_frame2.pose.orientation.w));

            fixed.getEulerYPR(yaw_front_fixed_, pitch_front_fixed_, roll_front_fixed_);
            pitch_front_fixed_ = -pitch_front_fixed_;

            conj_yaw_bottom_fixed_ = fmod(-180 + yaw_bottom_fixed_ * 180.0 / M_PI , 360.0);
            conj_pitch_front_fixed_ = -pitch_front_fixed_ * 180.0 / M_PI;

            ROS_INFO("[Microrobot] x:%f, y:%f, z:%f, yaw:%f, conj_yaw:%f, pitch:%f, conj_pitc:%f",
                    obj_fixed_frame.pose.position.x, obj_fixed_frame.pose.position.y, obj_fixed_frame2.pose.position.z,
                    yaw_bottom_fixed_* 180.0 / M_PI, conj_yaw_bottom_fixed_, pitch_front_fixed_* 180.0 / M_PI, conj_pitch_front_fixed_);

            microrobot_pose.pose.position.x = obj_fixed_frame.pose.position.x;
            microrobot_pose.pose.position.y = obj_fixed_frame.pose.position.y;
            microrobot_pose.pose.position.z = obj_fixed_frame2.pose.position.z;
            microrobot_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(C_ * M_PI / 180.0, pitch_front_fixed_, yaw_bottom_fixed_);
            microrobot_pose.header.stamp = ros::Time::now();
            microrobot_pose.header.frame_id = kuka_fixed_frame_;
            vis_pub_.publish(microrobot_pose);

            microrobot_pose.pose.position.x = obj_fixed_frame.pose.position.x;
            microrobot_pose.pose.position.y = obj_fixed_frame.pose.position.y;
            microrobot_pose.pose.position.z = obj_fixed_frame2.pose.position.z;
            microrobot_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(C_ * M_PI / 180.0, conj_pitch_front_fixed_ * M_PI / 180.0, conj_yaw_bottom_fixed_ * M_PI / 180.0);
            microrobot_pose.header.stamp = ros::Time::now();
            microrobot_pose.header.frame_id = kuka_fixed_frame_;
            conj_vis_pub_.publish(microrobot_pose);


            if(init_command_)
            {
                kuka_command_.data[0] = obj_fixed_frame.pose.position.x * 1e3;
                kuka_command_.data[1] = obj_fixed_frame.pose.position.y * 1e3;
                kuka_command_.data[2] = obj_fixed_frame2.pose.position.z * 1e3;

                /*msg2->pose.position.y - center_z
                double kp = 0.6;
                deltaZ = kp*scale_front*(z - center_z); //total distance travelled by microrobot /unit mm
                commandKukaZ = kuka->position[2] + deltaZ;
                cartPos.data[2] = commandKukaZ; //unit mm
                condition = true;*/


                //kuka_command_.data[2] = kuka_init_z;
                /*if((prev_kuka - obj_fixed_frame2.pose.position.z) * 1e3 > 0)
                {
                    kuka_command_.data[2] = obj_fixed_frame2.pose.position.z * 1e3;
                    prev_kuka = kuka_command_.data[2];
                }*/
                kuka_command_.data[5] = C_;

                ROS_INFO("[Goal] x:%f, y:%f, z:%f, roll:%f, pitch:%f, yaw:%f",
                    ref_.position.x, ref_.position.y, ref_.position.z,
                    ref_roll_* 180.0 / M_PI, ref_pitch_* 180.0 / M_PI, ref_yaw_* 180.0 / M_PI);

                switch(goal_status)
                {
                    case Status::ROTATING_TO_GOAL:
                        {
                            double angle_error_yaw = (atan2(ref_.position.y - obj_fixed_frame.pose.position.y,
                                                            ref_.position.x - obj_fixed_frame.pose.position.x) - yaw_bottom_fixed_)*180.0/M_PI;

                            double ref_pitch_angle_calc = atan2(ref_.position.z - obj_fixed_frame2.pose.position.z,
                                                              ref_.position.y - obj_fixed_frame.pose.position.y)*180.0/M_PI;

                            if(fabs(ref_pitch_angle_calc) > 90)
                                ref_pitch_angle_calc = fmod(180 - ref_pitch_angle_calc, 90);

                            double angle_error_pitch = ref_pitch_angle_calc - pitch_front_fixed_*180.0/M_PI;

                            double angle_error_yaw_conj = (atan2(ref_.position.y - obj_fixed_frame.pose.position.y,
                                                            ref_.position.x - obj_fixed_frame.pose.position.x))*180.0/M_PI - conj_yaw_bottom_fixed_;

                            double angle_error_pitch_conj = (ref_pitch_angle_calc) - conj_pitch_front_fixed_;

                            double angle_error_yaw_, angle_error_pitch_;

                            if(fabs(angle_error_yaw) < fabs(angle_error_yaw_conj))
                            {
                                angle_error_yaw_ = angle_error_yaw;
                                angle_error_pitch_ = angle_error_pitch;
                                yaw_bottom_fixed_ = yaw_bottom_fixed_;
                                pitch_front_fixed_ = pitch_front_fixed_;
                                direction = 1;
                            }
                            else
                            {
                                angle_error_yaw_ = angle_error_yaw_conj;
                                angle_error_pitch_ = angle_error_pitch_conj;
                                yaw_bottom_fixed_ = conj_yaw_bottom_fixed_;
                                pitch_front_fixed_ = conj_pitch_front_fixed_;
                                direction = -1;
                            }


                            kuka_command_.data[3] = yaw_bottom_fixed_ * 180.0 / M_PI + 1.0 * angle_error_yaw_ + 1.5;
                            kuka_command_.data[4] = (pitch_front_fixed_ * 180.0 / M_PI + 1.0 * angle_error_pitch_);

                            if(init)
                            {
                                /*getEpsilonCommand(dynamixel_command_, initArray, direction);
                                dynamixel_pub_.publish(dynamixel_command_);
                                usleep(1000000);
                                dynamixel_command_.data[0] = initDynamixelId1;
                                dynamixel_command_.data[1] = initDynamixelId2;
                                dynamixel_command_.data[2] = 5;
                                dynamixel_command_.data[3] = 5;
                                dynamixel_pub_.publish(dynamixel_command_);*/
                                init = false;
                            }

                            dt_ = (ros::Time::now() - prev_time_);
                            prev_time_ = ros::Time::now();

                            ROS_INFO("Rotating to goal.. AngleErrorYaw:%f, AngleYaw:%f, AngleErrorPitch:%f, AnglePitch:%f",
                                        angle_error_yaw_, atan2(ref_.position.y - obj_fixed_frame.pose.position.y,
                                                               ref_.position.x - obj_fixed_frame.pose.position.x)*180.0/M_PI,
                                        angle_error_pitch_, ref_pitch_angle_calc);
                            if(fabs(angle_error_yaw_) < direction_goal_tolarence)
                            {
                                goal_status = Status::REACHING_TO_GOAL;
                                ROS_INFO("********* Direction is reached ***********");
                            }

                            kuka_pub_.publish(kuka_command_);
                            break;
                        }
                    case Status::REACHING_TO_GOAL:
                        {
                            double position_error = hypot( hypot(ref_.position.x - obj_fixed_frame.pose.position.x,
                                                               ref_.position.y - obj_fixed_frame.pose.position.y),
                                                          ref_.position.z - obj_fixed_frame2.pose.position.z);

                            double angle_error_yaw = (atan2(ref_.position.y - obj_fixed_frame.pose.position.y,
                                                            ref_.position.x - obj_fixed_frame.pose.position.x) - yaw_bottom_fixed_)*180.0/M_PI;

                            double angle_error_pitch = (atan2(ref_.position.z - obj_fixed_frame2.pose.position.z,
                                                              ref_.position.y - obj_fixed_frame2.pose.position.y) - pitch_front_fixed_)*180.0/M_PI;

                            getEpsilonCommand(dynamixel_command_, initArray, direction);
                            dynamixel_pub_.publish(dynamixel_command_);

                            kuka_command_.data[3] = yaw_bottom_fixed_ * 180.0 / M_PI + 1.0 * angle_error_yaw + 6.5;
                            kuka_command_.data[4] = pitch_front_fixed_ * 180.0 / M_PI  + 1.0 * angle_error_pitch;
                            kuka_pub_.publish(kuka_command_);

                            ROS_INFO("Reaching to goal.. Position Err:%f, Angle Err Yaw:%f, Angle Err Pitch:%f", position_error,
                                                                                                              angle_error_yaw,
                                                                                                              angle_error_pitch);
                            if(position_error < xy_goal_tolerance)
                                goal_status = Status::REACHING_TO_ORIENTATION;
                            break;
                        }
                    case Status::REACHING_TO_ORIENTATION:
                        {
                            dynamixel_command_.data[0] = initDynamixelId1;
                            dynamixel_command_.data[1] = initDynamixelId2;
                            dynamixel_command_.data[2] = 5;
                            dynamixel_command_.data[3] = 5;
                            dynamixel_pub_.publish(dynamixel_command_);

                            double goal_yaw, goal_pitch, goal_roll;
                            tf::Matrix3x3 goal(tf::Quaternion(ref_.orientation.x,
                                                              ref_.orientation.y,
                                                              ref_.orientation.z,
                                                              ref_.orientation.w));
                            goal.getEulerYPR(goal_yaw, goal_pitch, goal_roll);
                            double orientation_error_yaw = (goal_yaw - yaw_bottom_fixed_) * 180.0 / M_PI;
                            double orientation_error_pitch = (goal_pitch - pitch_front_fixed_) * 180.0 / M_PI;

                            kuka_command_.data[3] = yaw_bottom_fixed_ * 180.0 / M_PI + 1.0 * orientation_error_yaw + 6.5;
                            kuka_command_.data[4] = pitch_front_fixed_ * 180.0 / M_PI + 1.0 * orientation_error_pitch;
                            kuka_pub_.publish(kuka_command_);

                            ROS_INFO("Reaching to orientation.. orientation_error_yaw:%f, orientation_error_pitch:%f", orientation_error_yaw, orientation_error_pitch);
                            if(fabs(orientation_error_yaw) < yaw_goal_tolerance && fabs(orientation_error_pitch) < yaw_goal_tolerance)
                                goal_status = Status::GOAL_REACHED;
                            break;
                        }
                    case Status::GOAL_REACHED:
                        {
                            ROS_INFO("Goal reached");
                            break;
                        }
                    case Status::YZ_PLANE:
                        {
                            if(!polar_queue_.empty())
                            {
                                l = sqrt(pow(microrobot_pose.pose.position.x - prev_.pose.position.x , 2) +
                                        pow(microrobot_pose.pose.position.y - prev_.pose.position.y, 2)) * 1e+3; //length of the polygon(mm)
                                if(l < lRef)
                                {
                                    nextPoint = false;
                                }
                                else
                                {
                                    ROS_INFO("Next Point");
                                    nextPoint = true;
                                    lRef = polar_queue_.front().first;
                                    next_angle = polar_queue_.front().second;
                                    polar_queue_.pop();
                                    prev_ = microrobot_pose;

                                    A_ = A_ + angle_queue_.front();
                                    angle_queue_.pop();
                                    ROS_INFO("A_:%f", A_);
                                }
                                ROS_INFO("l: %f, lRef: %f", l, lRef);

                                if(!nextPoint)
                                {
                                    if(epsilon)
                                    {
                                        if(sleep)
                                        {
                                            ROS_ERROR("Sleep");
                                            dynamixel_command_.data[0] = initDynamixelId1 - 15 * motorRes1; //mm
                                            dynamixel_command_.data[1] = initDynamixelId2; //mm
                                            dynamixel_command_.data[2] = 5;
                                            dynamixel_command_.data[3] = 5;
                                            dynamixel_pub_.publish(dynamixel_command_);
                                            epsilon = false;
                                            sleep = false;
                                        }
                                    }
                                    kuka_command_.data[3] = A_;
                                    kuka_command_.data[4] = B_ + angle;
                                }
                                else //rotate
                                {
                                    angle += next_angle;
                                    kuka_command_.data[3] = A_;
                                    kuka_command_.data[4] = B_ + angle;

                                    boost::mutex::scoped_lock lock(mutex_);
                                    /*dynamixel_command_.data[0] = initDynamixelId1; //mm
                                    dynamixel_command_.data[1] = initDynamixelId2; //mm
                                    dynamixel_command_.data[2] = 5;
                                    dynamixel_command_.data[3] = 5;
                                    dynamixel_pub_.publish(dynamixel_command_);*/
                                    boost::thread t(&ObjectPositioning::sleepThread, this);
                                    epsilon = true;
                                }
                                kuka_pub_.publish(kuka_command_);
                            }
                            else
                            {
                                ROS_ERROR("Queue is empty");
                            }
                        break;
                        }
                    case Status::XZ_PLANE:
                        {
                            if(!polar_queue_.empty())
                            {
                                l = sqrt(pow(microrobot_pose.pose.position.x - prev_.pose.position.x , 2) +
                                        pow(microrobot_pose.pose.position.y - prev_.pose.position.y, 2)) * 1e+3; //length of the polygon(mm)
                                if(l < lRef)
                                {
                                    nextPoint = false;
                                }
                                else
                                {
                                    ROS_INFO("Next Point");
                                    nextPoint = true;
                                    lRef = polar_queue_.front().first;
                                    next_angle = polar_queue_.front().second;
                                    polar_queue_.pop();
                                    prev_ = microrobot_pose;
                                }
                                ROS_INFO("l: %f, lRef: %f", l, lRef);

                                if(!nextPoint)
                                {
                                    if(epsilon)
                                    {
                                        if(sleep)
                                        {
                                            ROS_ERROR("Sleep");
                                            dynamixel_command_.data[0] = initDynamixelId1 - 15 * motorRes1; //mm
                                            dynamixel_command_.data[1] = initDynamixelId2; //mm
                                            dynamixel_command_.data[2] = 5;
                                            dynamixel_command_.data[3] = 5;
                                            dynamixel_pub_.publish(dynamixel_command_);
                                            epsilon = false;
                                            sleep = false;
                                        }
                                    }
                                    kuka_command_.data[3] = A_;
                                    kuka_command_.data[4] = B_ + angle;
                                }
                                else //rotate
                                {
                                    angle += next_angle;
                                    kuka_command_.data[3] = A_;
                                    kuka_command_.data[4] = B_ + angle;

                                    boost::mutex::scoped_lock lock(mutex_);
                                    /*dynamixel_command_.data[0] = initDynamixelId1; //mm
                                    dynamixel_command_.data[1] = initDynamixelId2; //mm
                                    dynamixel_command_.data[2] = 5;
                                    dynamixel_command_.data[3] = 5;
                                    dynamixel_pub_.publish(dynamixel_command_);*/
                                    boost::thread t(&ObjectPositioning::sleepThread, this);
                                    epsilon = true;
                                }
                                kuka_pub_.publish(kuka_command_);
                            }
                            else
                            {
                                ROS_ERROR("Queue is empty");
                            }
                            break;
                        }
                        case Status::HELIS:
                        {
                            l = sqrt(pow(microrobot_pose.pose.position.x - prev_.pose.position.x , 2) +
                                    pow(microrobot_pose.pose.position.y - prev_.pose.position.y, 2)) * 1e+3; //length of the polygon(mm)
                            if(l < lRef)
                            {
                                nextPoint = false;
                            }
                            else
                            {
                                ROS_INFO("Next Point");
                                nextPoint = true;

                                prev_ = microrobot_pose;
                            }
                            ROS_INFO("l: %f, lRef: %f", l, lRef);

                            if(!nextPoint)
                            {
                                if(epsilon)
                                {
                                    if(sleep)
                                    {
                                        ROS_ERROR("Sleep");
                                        /*dynamixel_command_.data[0] = initDynamixelId1 - 20 * motorRes1; //mm
                                        dynamixel_command_.data[1] = initDynamixelId2; //mm
                                        dynamixel_command_.data[2] = 5;
                                        dynamixel_command_.data[3] = 5;
                                        dynamixel_pub_.publish(dynamixel_command_);*/
                                        epsilon = false;
                                        sleep = false;
                                    }
                                }
                                kuka_command_.data[3] = A_ + polygonAngle;
                                kuka_command_.data[4] = B_ + pitchAngle;
                            }
                            else //rotate
                            {
                                polygonAngle += angleInc;
                                if(count >= 6)
                                    pitchAngleInc = 2;
                                pitchAngle += pitchAngleInc;
                                kuka_command_.data[3] = A_ + polygonAngle;
                                kuka_command_.data[4] = B_ + pitchAngle;

                                boost::mutex::scoped_lock lock(mutex_);
                                /*dynamixel_command_.data[0] = initDynamixelId1; //mm
                                dynamixel_command_.data[1] = initDynamixelId2; //mm
                                dynamixel_command_.data[2] = 5;
                                dynamixel_command_.data[3] = 5;
                                dynamixel_pub_.publish(dynamixel_command_);
                                boost::thread t(&ObjectPositioning::sleepThread, this);*/
                                epsilon = true;
                                count++;
                            }
                            kuka_pub_.publish(kuka_command_);
                            break;
                        }
                        case Status::CONSTRAINT:
                        {
                            if(!polar_queue_.empty())
                            {
                                l = sqrt(pow(microrobot_pose.pose.position.x - prev_.pose.position.x , 2) +
                                        pow(microrobot_pose.pose.position.y - prev_.pose.position.y, 2)) * 1e+3; //length of the polygon(mm)
                                if(l < lRef)
                                {
                                    nextPoint = false;
                                }
                                else
                                {
                                    ROS_INFO("Next Point");
                                    nextPoint = true;
                                    lRef = polar_queue_.front().first;
                                    next_angle = polar_queue_.front().second;
                                    polar_queue_.pop();
                                    prev_ = microrobot_pose;
                                }
                                ROS_INFO("l: %f, lRef: %f", l, lRef);

                                if(!nextPoint)
                                {
                                    if(epsilon)
                                    {
                                        if(sleep)
                                        {
                                            ROS_ERROR("Sleep");
                                            /*dynamixel_command_.data[0] = initDynamixelId1 - 15 * motorRes1; //mm
                                            dynamixel_command_.data[1] = initDynamixelId2; //mm
                                            dynamixel_command_.data[2] = 5;
                                            dynamixel_command_.data[3] = 5;
                                            dynamixel_pub_.publish(dynamixel_command_);*/
                                            epsilon = false;
                                            sleep = false;
                                        }
                                    }
                                    kuka_command_.data[3] = A_;
                                    kuka_command_.data[4] = B_ + angle;
                                }
                                else //rotate
                                {
                                    angle += next_angle;
                                    kuka_command_.data[3] = A_;
                                    kuka_command_.data[4] = B_ + angle;

                                    boost::mutex::scoped_lock lock(mutex_);
                                    /*dynamixel_command_.data[0] = initDynamixelId1; //mm
                                    dynamixel_command_.data[1] = initDynamixelId2; //mm
                                    dynamixel_command_.data[2] = 5;
                                    dynamixel_command_.data[3] = 5;
                                    dynamixel_pub_.publish(dynamixel_command_);*/
                                    boost::thread t(&ObjectPositioning::sleepThread, this);
                                    epsilon = true;
                                }
                                kuka_pub_.publish(kuka_command_);
                            }
                            else
                            {
                                ROS_ERROR("Queue is empty");
                            }
                            break;
                        }
                    default:
                        break;
                }
            }
        }
        catch(tf::ExtrapolationException& msg)
        {
            ROS_INFO("ExtrapolationException");
        }
    }

    void orientationCB(const teme_msgs::PoseArrayConstPtr& msg)
    {
        init_command_ = true;

        ROS_INFO("********Reference is obtained***********");
        for(int i = 0; i < msg->poses.size(); ++i)
        {
            ref_queue_.push(std::make_pair(static_cast<Status>(msg->status[i]), msg->poses[i]));
        }

        goal_status = ref_queue_.front().first;
        if(goal_status == 1 || goal_status == 2 || goal_status == 3)
        {
            ref_ = ref_queue_.front().second;
            ref_queue_.pop();
            tf::Quaternion quat;
            tf::quaternionMsgToTF(ref_.orientation, quat);
            tf::Matrix3x3 temp(quat);

            temp.getEulerYPR(ref_yaw_, ref_pitch_, ref_roll_);
        }

        if(goal_status == 5 || goal_status == 6)
        {
            prev_ = microrobot_pose;
            //polar_queue_.push(std::make_pair(2.0, 0.0));
            //polar_queue_.push(std::make_pair(5.0, -15.0));
            //polar_queue_.push(std::make_pair(5.0, 30.0));
            std::vector<std::pair<double, double> > record_goal;
            polar_queue_.push(std::make_pair(1, 0.0));
                angle_queue_.push(0);
                record_goal.push_back(polar_queue_.back());
            polar_queue_.push(std::make_pair(10, 25));
                angle_queue_.push(-30);
                record_goal.push_back(polar_queue_.back());
            polar_queue_.push(std::make_pair(0.5, -25));
                angle_queue_.push(0);
                record_goal.push_back(polar_queue_.back());
            polar_queue_.push(std::make_pair(0.5, 0));
                angle_queue_.push(0);
                record_goal.push_back(polar_queue_.back());

            std::ofstream os(goal_pose_location.c_str());
            if(!os.good())
            {
                ROS_ERROR("Error openning goal_pose.txt");
            }
            for(int i = 0; i < record_goal.size(); ++i)
            {
                os << "l: " << record_goal.at(i).first << ", theta: " << record_goal.at(i).second << "\n";
            }
            os.close();


            lRef = polar_queue_.front().first;
            next_angle = polar_queue_.front().second;
            polar_queue_.pop();

            A_ = A_ + angle_queue_.front();
            angle_queue_.pop();
        }

        if(goal_status == 7)
        {
            prev_ = microrobot_pose;
            lRef = polygonLRef;
        }
    }

    void getEpsilonCommand(std_msgs::Float64MultiArray& epsilonCommand, double initValues[2], int direction)
    {
        if(direction > 0) //positive epsilon
        {
            epsilonCommand.data[0] = initValues[0];
            epsilonCommand.data[1] = initValues[1] + 20 * motorRes2;
            epsilonCommand.data[2] = 5;
            epsilonCommand.data[3] = 5;
        }
        else //negative epsilon
        {
            epsilonCommand.data[0] = initValues[0] - 20 * motorRes1;
            epsilonCommand.data[1] = initValues[1];
            epsilonCommand.data[2] = 5;
            epsilonCommand.data[3] = 5;
        }
    }

    void sleepThread()
    {
        usleep(3500000);
        sleep = true;
    }

private:

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher kuka_pub_, vis_pub_, conj_vis_pub_, dynamixel_pub_;

    tf::TransformListener listener_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> *obj_sub_bottom_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> *obj_sub_front_;

    tf::MessageFilter<geometry_msgs::PoseStamped> *obj_filter_bottom_, *obj_filter_front_;

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy>* sync;

    geometry_msgs::PoseStamped obj_fixed_frame, obj_end_effector, obj_fixed_frame2, obj_end_effector2;
    std::string kuka_fixed_frame_ = "kuka_fixed_frame";
    std::string end_effector_frame_ = "end_effector";
    std::string camera_frame_front_ = "camera_node_front";
    std::string camera_frame_bottom_ = "camera_node_bottom";
    std::string ns_;

    double roll_bottom_fixed_, pitch_bottom_fixed_, yaw_bottom_fixed_, conj_yaw_bottom_fixed_;
    double roll_front_fixed_, pitch_front_fixed_, yaw_front_fixed_, conj_pitch_front_fixed_;

    std_msgs::Float64MultiArray kuka_command_;
    bool init_command_ = false;
    ros::Duration dt_;
    ros::Time prev_time_;
    geometry_msgs::Pose ref_;
    geometry_msgs::PoseStamped microrobot_pose, prev_;
    double ref_roll_, ref_yaw_, ref_pitch_;
    std::queue<std::pair<Status,geometry_msgs::Pose> > ref_queue_;
    Status goal_status = Status::ROTATING_TO_GOAL;

    std_msgs::Float64MultiArray dynamixel_command_;
    boost::shared_ptr<dynamixel_class::DynamixelClass> dynamixel_states_;
    boost::shared_ptr<kuka_class::kuka> kuka_states_;

    int direction = 1;

    double prev_kuka;
    double last_rotating_angle_;
    bool rotate_to_goal_ = true, reaching_to_orientation_ = true;

     //Polygon variables
    double n = 12; //polygon #edges
    double polygonAngle = 0.0; //degree
    double angleInc = 360 / n; //degree
    double pitchAngle = 0.0;
    double pitchAngleInc = -2;
    double radius = 6; //mm
    double polygonLRef = 2.0 * radius * sin((angleInc / 2.0) * M_PI / 180.0);
    int count = 0;
    //double lRef = 2.0 * radius * sin((angleInc / 2.0) * M_PI / 180.0); //polygon edge length
    double lRef = 30.0;
    double l = 0.0;
    bool nextPoint = false;
    bool epsilon = true;
    bool sleep = true;
    std::queue<std::pair<double, double> > polar_queue_;
    std::queue<double> angle_queue_;
    double angle = 0.0; //degree
    double next_angle = 0.0; //degree
    boost::mutex mutex_;
};

} // namespace


int main(int argc, char *argv[])
{
    if(argc > 1)
        goal_pose_location = std::string(argv[1]) + std::string("/goal_pose.txt");
    else
        goal_pose_location = "/home/aytac/Desktop/deneme_goal_pose.txt";
    ros::init(argc, argv, "xyz_control");
    ros::NodeHandle nh;

    object_positioning::ObjectPositioning op(nh);

    ros::Rate loop_rate(20.0);

    while(ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
