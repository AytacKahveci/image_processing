/* @author Aytaç Kahveci */
/*
Controlled movement in 3D
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
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <queue>
#include <map>

double X_ = 0.0;
double Y_ = 0.0;
double Z_ = 0.0;

double A_ = 90.0;
double B_ = 0.0;
double C_ = 180.0;

//Dynamixel Variables
double motorRes1 = 78.615;
double motorRes2 = 76.92;

double pitch_goal_tolerance = 5;
double direction_goal_tolarence = 0.005;
double yz_goal_tolerance = 0.005;

bool init = true;

enum Status
{
    ROTATING_TO_GOAL = 1,
    REACHING_TO_GOAL = 2,
    REACHING_TO_ORIENTATION = 3,
    GOAL_REACHED = 4
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
        obj_sub_front_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, ns_ + "camera_node_front/positionDist", 1);

        obj_filter_front_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(*obj_sub_front_, listener_, end_effector_frame_, 1);
        obj_filter_front_->registerCallback(boost::bind(&ObjectPositioning::poseCB, this, _1));

        kuka_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/joint_position_controller/command",1);
        vis_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/microrobot/pose",1);
        conj_vis_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/microrobot/pose_conj",1);
        dynamixel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/dynamixel/JointPosGroup/poseffCommand",1);

        kuka_command_.data.resize(6);
        dynamixel_command_.data.resize(4);

        dynamixel_states_.reset(new dynamixel_class::DynamixelClass(nh_, 2));
        kuka_states_.reset(new kuka_class::kuka(nh_, 6));

        if(!nh_.getParam("yz_goal_tolerance", yz_goal_tolerance))
        {
            yz_goal_tolerance = 2; //unit meter
            ROS_WARN("xy_goal_tolarence couldn't find in the param server. \
                       default is used: %f, ns: %s", yz_goal_tolerance, nh_.getNamespace().c_str());
        }
        if(!nh_.getParam("direction_goal_tolarence", direction_goal_tolarence))
        {
            direction_goal_tolarence = 2; //unit degree
            ROS_WARN("direction_goal_tolarence couldn't find in the param server. \
                       default is used: %f, ns: %s", direction_goal_tolarence, nh_.getNamespace().c_str());
        }
        if(!nh_.getParam("pitch_goal_tolerance", pitch_goal_tolerance))
        {
            pitch_goal_tolerance = 2; //unit degree
            ROS_WARN("pitch_goal_tolerance couldn't find in the param server. \
                       default is used: %f, ns: %s", pitch_goal_tolerance, nh_.getNamespace().c_str());
        }
    }

    ~ObjectPositioning()
    {}

    void poseCB(const boost::shared_ptr<const geometry_msgs::PoseStamped>& msg)
    {
        static const double initDynamixelId1 = dynamixel_states_->position[0];
        static const double initDynamixelId2 = dynamixel_states_->position[1];
        double initArray[2] = {initDynamixelId1, initDynamixelId2};
        static const double kuka_init_x = kuka_states_->position[0];

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

            fixed.getEulerYPR(yaw_front_fixed_, pitch_front_fixed_, roll_front_fixed_);

            pitch_front_fixed_ = -pitch_front_fixed_;
            conj_pitch_front_fixed_ = -pitch_front_fixed_ * 180.0 / M_PI;

            ROS_INFO("[Microrobot] x:%f, y:%f, z:%f, A:%f, pitch:%f, conj_pitch:%f, C:%f",
                                   0.0, obj_fixed_frame.pose.position.y, obj_fixed_frame.pose.position.z,
                                   A_, pitch_front_fixed_* 180.0 / M_PI, conj_pitch_front_fixed_, C_);

            microrobot_pose.pose.position.x = 0;
            microrobot_pose.pose.position.y = obj_fixed_frame.pose.position.y;
            microrobot_pose.pose.position.z = obj_fixed_frame.pose.position.z;
            microrobot_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(C_ * M_PI / 180.0, pitch_front_fixed_, A_ * M_PI / 180.0);
            microrobot_pose.header.stamp = ros::Time::now();
            microrobot_pose.header.frame_id = kuka_fixed_frame_;
            vis_pub_.publish(microrobot_pose);

            microrobot_pose.pose.position.x = 0;
            microrobot_pose.pose.position.y = obj_fixed_frame.pose.position.y;
            microrobot_pose.pose.position.z = obj_fixed_frame.pose.position.z;
            microrobot_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(C_ * M_PI / 180.0, conj_pitch_front_fixed_ * M_PI / 180.0, A_ * M_PI / 180.0);
            microrobot_pose.header.stamp = ros::Time::now();
            microrobot_pose.header.frame_id = kuka_fixed_frame_;
            conj_vis_pub_.publish(microrobot_pose);

            if(init_command_)
            {
                kuka_command_.data[0] = kuka_init_x;
                kuka_command_.data[1] = obj_fixed_frame.pose.position.y * 1e3;
                kuka_command_.data[2] = obj_fixed_frame.pose.position.z * 1e3;
                kuka_command_.data[3] = A_;
                kuka_command_.data[5] = C_;

                ROS_INFO("[Goal] x:%f, y:%f, z:%f, roll:%f, pitch:%f, yaw:%f",
                                 ref_.position.x, ref_.position.y, ref_.position.z,
                                 ref_roll_* 180.0 / M_PI, ref_pitch_* 180.0 / M_PI, ref_yaw_* 180.0 / M_PI);

                switch(goal_status)
                {
                    case Status::ROTATING_TO_GOAL:
                        {
                            double ref_pitch_angle_calc = atan2(ref_.position.z - obj_fixed_frame.pose.position.z,
                                                                ref_.position.y - obj_fixed_frame.pose.position.y)*180.0/M_PI;

                            if(fabs(ref_pitch_angle_calc) > 90)
                                ref_pitch_angle_calc = fmod(180 - ref_pitch_angle_calc, 90);

                            double angle_error_pitch = ref_pitch_angle_calc - pitch_front_fixed_*180.0/M_PI;

                            double angle_error_pitch_conj = (ref_pitch_angle_calc) - conj_pitch_front_fixed_;

                            double angle_error_pitch_;

                            if(fabs(angle_error_pitch) < fabs(angle_error_pitch_conj))
                            {
                                angle_error_pitch_ = angle_error_pitch;
                                pitch_front_fixed_ = pitch_front_fixed_;
                                direction = 1;
                            }
                            else
                            {
                                angle_error_pitch_ = angle_error_pitch_conj;
                                pitch_front_fixed_ = conj_pitch_front_fixed_;
                                direction = -1;
                            }
                            kuka_command_.data[4] = -(pitch_front_fixed_ * 180.0 / M_PI + 1.0 * angle_error_pitch_) - 6.5;


                            ROS_INFO("Rotating to goal.. AngleErrorPitch:%f, AnglePitch:%f",
                                            angle_error_pitch_, ref_pitch_angle_calc);

                            if(fabs(angle_error_pitch_) < direction_goal_tolarence) // unit degree
                            {
                                goal_status = Status::REACHING_TO_GOAL;
                                ROS_INFO("********* Direction is reached ***********");
                            }

                            kuka_pub_.publish(kuka_command_);
                            break;
                        }
                    case Status::REACHING_TO_GOAL:
                        {
                            double position_error = hypot(ref_.position.y - obj_fixed_frame.pose.position.y,
                                                          ref_.position.z - obj_fixed_frame.pose.position.z)*1e3;

                            double ref_pitch_angle_calc = atan2(ref_.position.z - obj_fixed_frame.pose.position.z,
                                                                ref_.position.y - obj_fixed_frame.pose.position.y)*180.0/M_PI;

                            double angle_error_pitch = ref_pitch_angle_calc - pitch_front_fixed_*180.0/M_PI;

                            if(position_error < 6) //unit mm
                            {
                                dynamixel_command_.data[0] = initDynamixelId1; //mm
                                dynamixel_command_.data[1] = initDynamixelId2 + position_error * motorRes2; //mm
                                dynamixel_command_.data[2] = 5;
                                dynamixel_command_.data[3] = 5;
                            }
                            else
                            {
                                dynamixel_command_.data[0] = initDynamixelId1; //mm
                                dynamixel_command_.data[1] = initDynamixelId2 + 20 * motorRes2; //mm
                                dynamixel_command_.data[2] = 5;
                                dynamixel_command_.data[3] = 5;
                            }

                            kuka_command_.data[4] = kuka_states_->position[4];
                            dynamixel_pub_.publish(dynamixel_command_);
                            kuka_pub_.publish(kuka_command_);

                            ROS_INFO("Reaching to goal.. Position Err:%f, Angle Err Pitch:%f", position_error,
                                                                                             angle_error_pitch);
                            if(position_error < yz_goal_tolerance) //unit mm
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

                            double ref_pitch_angle_calc = atan2(ref_.position.z - obj_fixed_frame.pose.position.z,
                                                                ref_.position.y - obj_fixed_frame.pose.position.y)*180.0/M_PI;

                            double angle_error_pitch = ref_pitch_angle_calc - pitch_front_fixed_*180.0/M_PI;
                            double angle_error_pitch_conj = (ref_pitch_angle_calc) - conj_pitch_front_fixed_;

                            double angle_error_pitch_;

                            if(fabs(angle_error_pitch) < fabs(angle_error_pitch_conj))
                            {
                                angle_error_pitch_ = angle_error_pitch;
                                pitch_front_fixed_ = pitch_front_fixed_;
                            }
                            else
                            {
                                angle_error_pitch_ = angle_error_pitch_conj;
                                pitch_front_fixed_ = conj_pitch_front_fixed_;
                            }

                            kuka_command_.data[4] = -(pitch_front_fixed_ * 180.0 / M_PI + 1.0 * angle_error_pitch_) - 6.5;

                            kuka_pub_.publish(kuka_command_);

                            ROS_INFO("Reaching to orientation.. orientation_error_pitch:%f", angle_error_pitch_);
                            if(fabs(angle_error_pitch_) < pitch_goal_tolerance) //unit degree
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

    void orientationCB(const geometry_msgs::PoseArrayConstPtr& msg)
    {
        init_command_ = true;

        ROS_INFO("********Reference is obtained***********");
        for(int i = 0; i < msg->poses.size(); ++i)
        {
            ref_queue_.push(std::make_pair(Status::REACHING_TO_GOAL, msg->poses[i]));
        }

        goal_status = ref_queue_.front().first;
        ref_ = ref_queue_.front().second;
        ref_queue_.pop();
        tf::Quaternion quat;
        tf::quaternionMsgToTF(ref_.orientation, quat);
        tf::Matrix3x3 temp(quat);

        temp.getEulerYPR(ref_yaw_, ref_pitch_, ref_roll_);
    }

private:

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher kuka_pub_, vis_pub_, conj_vis_pub_, dynamixel_pub_;

    tf::TransformListener listener_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> *obj_sub_front_;
    tf::MessageFilter<geometry_msgs::PoseStamped> *obj_filter_front_;

    geometry_msgs::PoseStamped obj_fixed_frame, obj_end_effector;
    std::string kuka_fixed_frame_ = "kuka_fixed_frame";
    std::string end_effector_frame_ = "end_effector";
    std::string camera_frame_front_ = "camera_node_front";
    std::string camera_frame_bottom_ = "camera_node_bottom";
    std::string ns_;

    double roll_front_fixed_, pitch_front_fixed_, yaw_front_fixed_, conj_pitch_front_fixed_;

    std_msgs::Float64MultiArray kuka_command_;
    bool init_command_ = false;

    geometry_msgs::Pose ref_;
    geometry_msgs::PoseStamped microrobot_pose;
    double ref_roll_, ref_yaw_, ref_pitch_;
    std::queue<std::pair<Status,geometry_msgs::Pose> > ref_queue_;
    Status goal_status = Status::ROTATING_TO_GOAL;

    std_msgs::Float64MultiArray dynamixel_command_;
    boost::shared_ptr<dynamixel_class::DynamixelClass> dynamixel_states_;
    boost::shared_ptr<kuka_class::kuka> kuka_states_;

    int direction = 1;

    double prev_kuka;
};

} // namespace


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "yz_control");
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
