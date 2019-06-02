/* @author Aytaç Kahveci */
/*
Controlled movement in 3DOF
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

double X_ = 0.0;
double Y_ = 0.0;
double Z_ = 7.2;

double A_ = -90.0;
double B_ = 0.0;
double C_ = -180.0;

//Dynamixel Variables
double motorRes1 = 78.615;
double motorRes2 = 76.92;

double yaw_goal_tolerance = 1;
double direction_goal_tolarence = 0.005;
double xy_goal_tolerance = 0.2;

bool init = true;

enum Status
{
    ROTATING_TO_GOAL = 1,
    REACHING_TO_GOAL = 2,
    REACHING_TO_ORIENTATION = 3,
    GOAL_REACHED = 4,
    ONLY_X = 5,
    ONLY_Y = 6,
    POLAR_COORDINATE = 7,
    POLAR_COORDINATE_QUEUE = 8
};

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

        obj_filter_bottom_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(*obj_sub_bottom_, listener_, end_effector_frame_, 1);
        obj_filter_bottom_->registerCallback(boost::bind(&ObjectPositioning::poseCB, this, _1));

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
            xy_goal_tolerance = 0.2; //unit mm
            ROS_WARN("xy_goal_tolarence couldn't find in the param server. \
                       default is used: %f, ns: %s", xy_goal_tolerance, nh_.getNamespace().c_str());
        }
        if(!nh_.getParam("direction_goal_tolarence", direction_goal_tolarence))
        {
            direction_goal_tolarence = 2; //unit degree
            ROS_WARN("direction_goal_tolarence couldn't find in the param server. \
                       default is used: %f, ns: %s", direction_goal_tolarence, nh_.getNamespace().c_str());
        }
        if(!nh_.getParam("yaw_goal_tolerance", yaw_goal_tolerance))
        {
            yaw_goal_tolerance = 2; //unit degree
            ROS_WARN("yaw_goal_tolerance couldn't find in the param server. \
                       default is used: %f, ns: %s", yaw_goal_tolerance, nh_.getNamespace().c_str());
        }
    }

    ~ObjectPositioning()
    {}

    void poseCB(const boost::shared_ptr<const geometry_msgs::PoseStamped>& msg)
    {
        static const double initDynamixelId1 = dynamixel_states_->position[0];
        static const double initDynamixelId2 = dynamixel_states_->position[1];
        double initArray[2] = {initDynamixelId1, initDynamixelId2};
        static const double kuka_init_z = kuka_states_->position[2];

        ROS_WARN("dynamixel_state1:%f, dynamixel_state2:%f", initDynamixelId1, initDynamixelId2);

        try
        {
            //Get transformation matrix
            listener_.transformPose(kuka_fixed_frame_, *msg, obj_fixed_frame);

            //Tranform Quaternion to Euler angles
            tf::Matrix3x3 fixed(tf::Quaternion(obj_fixed_frame.pose.orientation.x, \
                                               obj_fixed_frame.pose.orientation.y, \
                                               obj_fixed_frame.pose.orientation.z, \
                                               obj_fixed_frame.pose.orientation.w));

            fixed.getEulerYPR(yaw_bottom_fixed_, pitch_bottom_fixed_, roll_bottom_fixed_);

            if(yaw_bottom_fixed_ < 0)
                yaw_bottom_fixed_ = fmod(2*M_PI + yaw_bottom_fixed_ + M_PI, 2*M_PI);
            conj_yaw_bottom_fixed_ = fmod(-180 + yaw_bottom_fixed_ * 180.0 / M_PI , 360.0);

            ROS_INFO("[Microrobot] x:%f, y:%f, z:%d, yaw:%f, conj_yaw:%f",
                                   obj_fixed_frame.pose.position.x, obj_fixed_frame.pose.position.y, 0,
                                   yaw_bottom_fixed_* 180.0 / M_PI, conj_yaw_bottom_fixed_);

            microrobot_pose.pose.position.x = obj_fixed_frame.pose.position.x;
            microrobot_pose.pose.position.y = obj_fixed_frame.pose.position.y;
            microrobot_pose.pose.position.z = 0;
            microrobot_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(C_ * M_PI / 180.0, B_ * M_PI / 180.0, yaw_bottom_fixed_);
            microrobot_pose.header.stamp = ros::Time::now();
            microrobot_pose.header.frame_id = kuka_fixed_frame_;
            vis_pub_.publish(microrobot_pose);

            microrobot_pose.pose.position.x = obj_fixed_frame.pose.position.x;
            microrobot_pose.pose.position.y = obj_fixed_frame.pose.position.y;
            microrobot_pose.pose.position.z = 0;
            microrobot_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(C_ * M_PI / 180.0, B_ * M_PI / 180.0, conj_yaw_bottom_fixed_ * M_PI / 180.0);
            microrobot_pose.header.stamp = ros::Time::now();
            microrobot_pose.header.frame_id = kuka_fixed_frame_;
            conj_vis_pub_.publish(microrobot_pose);

            if(init_command_)
            {
                kuka_command_.data[0] = obj_fixed_frame.pose.position.x * 1e3;
                kuka_command_.data[1] = obj_fixed_frame.pose.position.y * 1e3;
                kuka_command_.data[2] = Z_;
                kuka_command_.data[4] = B_;
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

                            double angle_error_yaw_conj = (atan2(ref_.position.y - obj_fixed_frame.pose.position.y,
                                                            ref_.position.x - obj_fixed_frame.pose.position.x))*180.0/M_PI - conj_yaw_bottom_fixed_;

                            double angle_error_yaw_;

                            if(fabs(angle_error_yaw) < fabs(angle_error_yaw_conj))
                            {
                                angle_error_yaw_ = angle_error_yaw;
                                yaw_bottom_fixed_ = yaw_bottom_fixed_;
                                direction = 1;
                            }
                            else
                            {
                                angle_error_yaw_ = angle_error_yaw_conj;
                                yaw_bottom_fixed_ = conj_yaw_bottom_fixed_;
                                direction = -1;
                            }

                            if(rotate_to_goal_)
                            {
                                kuka_command_.data[3] = atan2(ref_.position.y - obj_fixed_frame.pose.position.y,
                                                            ref_.position.x - obj_fixed_frame.pose.position.x) * 180.0 / M_PI;
                                last_rotating_angle_ = kuka_command_.data[3];
                                rotate_to_goal_ = false;
                            }
                            kuka_pub_.publish(kuka_command_);

                            ROS_INFO("Rotating to goal.. Dest:%f, AngleErrorYaw:%f, AngleErrorYawConj:%f",
                                            atan2(ref_.position.y - obj_fixed_frame.pose.position.y,
                                                                    ref_.position.x - obj_fixed_frame.pose.position.x)*180.0/M_PI,
                                            angle_error_yaw,
                                            angle_error_yaw_conj);

                            if(fabs(angle_error_yaw_) < direction_goal_tolarence) // unit degree
                            {
                                goal_status = Status::REACHING_TO_GOAL;
                                rotate_to_goal_ = true;
                                ROS_INFO("********* Direction is reached ***********");
                            }

                            break;
                        }
                    case Status::REACHING_TO_GOAL:
                        {
                            double position_error = hypot(ref_.position.x - obj_fixed_frame.pose.position.x,
                                                          ref_.position.y - obj_fixed_frame.pose.position.y)*1e3;

                            double angle_error_yaw = (atan2(ref_.position.y - obj_fixed_frame.pose.position.y,
                                                            ref_.position.x - obj_fixed_frame.pose.position.x) - yaw_bottom_fixed_)*180.0/M_PI;

                            if(direction > 0) //positive epsilon
                            {
                                if(position_error < 0.3) //unit mm
                                {
                                    dynamixel_command_.data[0] = initDynamixelId1; //mm
                                    dynamixel_command_.data[1] = initDynamixelId2; //mm
                                    dynamixel_command_.data[2] = 5;
                                    dynamixel_command_.data[3] = 5;
                                }
                                else
                                {
                                    dynamixel_command_.data[0] = initDynamixelId1; //mm
                                    dynamixel_command_.data[1] = std::ceil(initDynamixelId2 + 20 * motorRes2); //mm
                                    dynamixel_command_.data[2] = 5;
                                    dynamixel_command_.data[3] = 5;
                                }
                            }
                            else
                            {
                                if(position_error < 0.3) //unit mm
                                {
                                    dynamixel_command_.data[0] = std::ceil(initDynamixelId1 - 4*position_error * motorRes2); //mm
                                    dynamixel_command_.data[1] = initDynamixelId2; //mm
                                    dynamixel_command_.data[2] = 5;
                                    dynamixel_command_.data[3] = 5;
                                }
                                else
                                {
                                    dynamixel_command_.data[0] = std::ceil(initDynamixelId1 - 20 * motorRes2); //mm
                                    dynamixel_command_.data[1] = initDynamixelId2; //mm
                                    dynamixel_command_.data[2] = 5;
                                    dynamixel_command_.data[3] = 5;
                                }
                            }

                            kuka_command_.data[3] = last_rotating_angle_;
                            dynamixel_pub_.publish(dynamixel_command_);
                            kuka_pub_.publish(kuka_command_);

                            ROS_INFO("Reaching to goal.. Position Err:%f, Angle Err Yaw:%f", position_error,
                                                                                             angle_error_yaw);
                            if(position_error < 0.3) //unit mm
                                goal_status = Status::GOAL_REACHED;
                                //goal_status = Status::REACHING_TO_ORIENTATION;
                            break;
                        }
                    case Status::REACHING_TO_ORIENTATION:
                        {
                            dynamixel_command_.data[0] = initDynamixelId1;
                            dynamixel_command_.data[1] = initDynamixelId2;
                            dynamixel_command_.data[2] = 5;
                            dynamixel_command_.data[3] = 5;
                            dynamixel_pub_.publish(dynamixel_command_);

                            double orientation_error_yaw = (ref_yaw_ - yaw_bottom_fixed_) * 180.0 / M_PI;
                            double orientation_error_yaw_conj = (ref_yaw_ - conj_yaw_bottom_fixed_) * 180.0 / M_PI;

                            double angle_error_yaw_;

                            if(fabs(orientation_error_yaw) < fabs(orientation_error_yaw_conj))
                            {
                                angle_error_yaw_ = orientation_error_yaw;
                                yaw_bottom_fixed_ = yaw_bottom_fixed_;
                            }
                            else
                            {
                                angle_error_yaw_ = orientation_error_yaw_conj;
                                yaw_bottom_fixed_ = conj_yaw_bottom_fixed_;
                            }

                            if(reaching_to_orientation_)
                            {
                                kuka_command_.data[3] = yaw_bottom_fixed_ * 180.0 / M_PI + 1.0 * angle_error_yaw_ + 1.0;
                                reaching_to_orientation_ = true;
                            }
                            kuka_pub_.publish(kuka_command_);

                            ROS_INFO("Reaching to orientation.. orientation_error_yaw:%f", angle_error_yaw_);
                            if(fabs(angle_error_yaw_) < yaw_goal_tolerance) //unit degree
                                goal_status = Status::GOAL_REACHED;
                            break;
                        }
                    case Status::GOAL_REACHED:
                        {
                            ROS_INFO("Goal reached");
                            if(!ref_queue_.empty())
                            {
                                goal_status = ref_queue_.front().first;
                                ref_ = ref_queue_.front().second;
                                ref_queue_.pop();
                                tf::Quaternion quat;
                                tf::quaternionMsgToTF(ref_.orientation, quat);
                                tf::Matrix3x3 temp(quat);

                                temp.getEulerYPR(ref_yaw_, ref_pitch_, ref_roll_);
                            }
                            else
                            {
                                ROS_ERROR("queue is empty");
                            }
                            break;
                        }
                    case Status::ONLY_X:
                        {
                            double position_error = (ref_.position.y - obj_fixed_frame.pose.position.y)*1e3;

                            double angle_error_yaw = (atan2(ref_.position.y - obj_fixed_frame.pose.position.y,
                                                            ref_.position.x - obj_fixed_frame.pose.position.x) - yaw_bottom_fixed_)*180.0/M_PI;

                            if(direction > 0) //positive epsilon
                            {
                                if(position_error < 0.3) //unit mm
                                {
                                    dynamixel_command_.data[0] = initDynamixelId1; //mm
                                    dynamixel_command_.data[1] = std::ceil(initDynamixelId2); //mm
                                    dynamixel_command_.data[2] = 5;
                                    dynamixel_command_.data[3] = 5;
                                }
                                else
                                {
                                    dynamixel_command_.data[0] = initDynamixelId1; //mm
                                    dynamixel_command_.data[1] = std::ceil(initDynamixelId2 + 20 * motorRes2); //mm
                                    dynamixel_command_.data[2] = 5;
                                    dynamixel_command_.data[3] = 5;
                                }
                            }
                            else
                            {
                                if(position_error < 0.3) //unit mm
                                {
                                    dynamixel_command_.data[0] = std::ceil(initDynamixelId1 - 4*position_error * motorRes2); //mm
                                    dynamixel_command_.data[1] = initDynamixelId2; //mm
                                    dynamixel_command_.data[2] = 5;
                                    dynamixel_command_.data[3] = 5;
                                }
                                else
                                {
                                    dynamixel_command_.data[0] = std::ceil(initDynamixelId1 - 20 * motorRes2); //mm
                                    dynamixel_command_.data[1] = initDynamixelId2; //mm
                                    dynamixel_command_.data[2] = 5;
                                    dynamixel_command_.data[3] = 5;
                                }
                            }

                            kuka_command_.data[3] = A_;
                            dynamixel_pub_.publish(dynamixel_command_);
                            kuka_pub_.publish(kuka_command_);

                            ROS_INFO("ONLY_X Movement. Position Err:%f, Angle Err Yaw:%f", position_error,
                                                                                             angle_error_yaw);
                            if(position_error < 0.3) //unit mm
                                goal_status = Status::GOAL_REACHED;
                            break;
                        }
                    case Status::ONLY_Y:
                        {
                            double position_error = (ref_.position.x - obj_fixed_frame.pose.position.x)*1e3;

                            double angle_error_yaw = (atan2(ref_.position.y - obj_fixed_frame.pose.position.y,
                                                            ref_.position.x - obj_fixed_frame.pose.position.x) - yaw_bottom_fixed_)*180.0/M_PI;

                            if(direction > 0) //positive epsilon
                            {
                                if(position_error < 0.3) //unit mm
                                {
                                    dynamixel_command_.data[0] = initDynamixelId1; //mm
                                    dynamixel_command_.data[1] = std::ceil(initDynamixelId2); //mm
                                    dynamixel_command_.data[2] = 5;
                                    dynamixel_command_.data[3] = 5;
                                }
                                else
                                {
                                    dynamixel_command_.data[0] = initDynamixelId1; //mm
                                    dynamixel_command_.data[1] = std::ceil(initDynamixelId2 + 20 * motorRes2); //mm
                                    dynamixel_command_.data[2] = 5;
                                    dynamixel_command_.data[3] = 5;
                                }
                            }
                            else
                            {
                                if(position_error < 0.3) //unit mm
                                {
                                    dynamixel_command_.data[0] = std::ceil(initDynamixelId1 - 4*position_error * motorRes2); //mm
                                    dynamixel_command_.data[1] = initDynamixelId2; //mm
                                    dynamixel_command_.data[2] = 5;
                                    dynamixel_command_.data[3] = 5;
                                }
                                else
                                {
                                    dynamixel_command_.data[0] = std::ceil(initDynamixelId1 - 20 * motorRes2); //mm
                                    dynamixel_command_.data[1] = initDynamixelId2; //mm
                                    dynamixel_command_.data[2] = 5;
                                    dynamixel_command_.data[3] = 5;
                                }
                            }

                            kuka_command_.data[3] = 0;
                            dynamixel_pub_.publish(dynamixel_command_);
                            kuka_pub_.publish(kuka_command_);

                            ROS_INFO("ONLY_Y Movement. Position Err:%f, Angle Err Yaw:%f", position_error,
                                                                                             angle_error_yaw);
                            if(position_error < 0.3) //unit mm
                                goal_status = Status::GOAL_REACHED;
                            break;
                        }
                    case Status::POLAR_COORDINATE:
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
                                        dynamixel_command_.data[0] = initDynamixelId1 - 15 * motorRes1; //mm
                                        dynamixel_command_.data[1] = initDynamixelId2; //mm
                                        dynamixel_command_.data[2] = 5;
                                        dynamixel_command_.data[3] = 5;
                                        dynamixel_pub_.publish(dynamixel_command_);
                                        epsilon = false;
                                        sleep = false;
                                    }
                                }
                                kuka_command_.data[3] = A_ - angle;
                            }
                            else //rotate
                            {
                                kuka_command_.data[3] = A_ - angle;

                                dynamixel_command_.data[0] = initDynamixelId1; //mm
                                dynamixel_command_.data[1] = initDynamixelId2; //mm
                                dynamixel_command_.data[2] = 5;
                                dynamixel_command_.data[3] = 5;
                                dynamixel_pub_.publish(dynamixel_command_);
                                boost::thread t(&ObjectPositioning::sleepThread, this);
                                epsilon = true;
                            }
                            kuka_pub_.publish(kuka_command_);

                        }
                    case Status::POLAR_COORDINATE_QUEUE:
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
                                kuka_command_.data[3] = A_ - angle;
                            }
                            else //rotate
                            {
                                angle -= next_angle;
                                kuka_command_.data[3] = A_ - angle;

                                boost::mutex::scoped_lock lock(mutex_);
                                dynamixel_command_.data[0] = initDynamixelId1; //mm
                                dynamixel_command_.data[1] = initDynamixelId2; //mm
                                dynamixel_command_.data[2] = 5;
                                dynamixel_command_.data[3] = 5;
                                dynamixel_pub_.publish(dynamixel_command_);
                                boost::thread t(&ObjectPositioning::sleepThread, this);
                                epsilon = true;
                            }
                            kuka_pub_.publish(kuka_command_);
                        }
                        else
                        {
                            ROS_ERROR("Queue is empty");
                        }

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
        ref_ = ref_queue_.front().second;
        ref_queue_.pop();
        tf::Quaternion quat;
        tf::quaternionMsgToTF(ref_.orientation, quat);
        tf::Matrix3x3 temp(quat);

        temp.getEulerYPR(ref_yaw_, ref_pitch_, ref_roll_);

        prev_ = microrobot_pose;
        polar_queue_.push(std::make_pair(2.0, 0.0));
        polar_queue_.push(std::make_pair(10.0, 30.0));
        polar_queue_.push(std::make_pair(13.0, -22.619864948040426));
        polar_queue_.push(std::make_pair(13.0, 0.0));



        lRef = polar_queue_.front().first;
        next_angle = polar_queue_.front().second;
        polar_queue_.pop();
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
    tf::MessageFilter<geometry_msgs::PoseStamped> *obj_filter_bottom_;

    geometry_msgs::PoseStamped obj_fixed_frame, obj_end_effector;
    std::string kuka_fixed_frame_ = "kuka_fixed_frame";
    std::string end_effector_frame_ = "end_effector";
    std::string camera_frame_front_ = "camera_node_front";
    std::string camera_frame_bottom_ = "camera_node_bottom";
    std::string ns_;

    double roll_bottom_fixed_, pitch_bottom_fixed_, yaw_bottom_fixed_, conj_yaw_bottom_fixed_;

    std_msgs::Float64MultiArray kuka_command_;
    bool init_command_ = false;

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
    double n = 8; //polygon #edges
    double angle = 0.0; //degree
    double angleInc = 360 / n; //degree
    double radius = 6; //mm
    //double lRef = 2.0 * radius * sin((angleInc / 2.0) * M_PI / 180.0); //polygon edge length
    double lRef = 30.0;
    double l = 0.0;
    bool nextPoint = false;
    bool epsilon = true;
    bool sleep = true;
    std::queue<std::pair<double, double> > polar_queue_;
    double next_angle = 0.0;
    boost::mutex mutex_;

};

} // namespace


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "polar_coordinate");
    ros::NodeHandle nh;

    object_positioning::ObjectPositioning op(nh);

    ros::Rate loop_rate(5);

    while(ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
