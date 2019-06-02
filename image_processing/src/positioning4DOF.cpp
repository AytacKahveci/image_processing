/* @author Aytaç Kahveci */
/*
4DOF Controlled movement(x, y, z, yaw)
In the beginning, end effector pose is alligned with microrobot pose by this script
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
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

double X_ = 0.0;
double Y_ = 0.0;
double Z_ = 0.0;

double A_ = 90.0;
double B_ = 0.0;
double C_ = 180.0;

//Dynamixel Variables
double motorRes1 = 78.615;
double motorRes2 = 76.92;

double yaw_goal_tolerance = 5;
double direction_goal_tolarence = 0.005;
double xy_goal_tolerance = 0.005;

namespace object_positioning
{

class ObjectPositioning
{

public:

    ObjectPositioning(ros::NodeHandle& nh) : nh_(nh)
    {
        ns_ = nh_.getNamespace();
        obj_sub_bottom_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, ns_ + "camera_node_bottom/positionDist", 1);
        obj_sub_front_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, ns_ + "camera_node_front/positionDist", 1);

        obj_filter_bottom_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(*obj_sub_bottom_, listener_, end_effector_frame_, 1);
        obj_filter_front_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(*obj_sub_front_, listener_, kuka_fixed_frame_, 1);

        sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *obj_filter_bottom_, *obj_filter_front_);

        sync->registerCallback(boost::bind(&ObjectPositioning::poseCB, this, _1, _2));

        pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/joint_position_controller/command",1);
        vis_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/microrobot/pose",1);

        kuka_command_.data.resize(6);
    }

    ~ObjectPositioning()
    {}

    void poseCB(const boost::shared_ptr<const geometry_msgs::PoseStamped>& msg, const boost::shared_ptr<const geometry_msgs::PoseStamped>& msg2)
    {
        try
        {
            listener_.transformPose(kuka_fixed_frame_, *msg, obj_fixed_frame);
            listener_.transformPose(kuka_fixed_frame_, *msg2, obj_fixed_frame2);

            tf::Matrix3x3 fixed(tf::Quaternion(obj_fixed_frame.pose.orientation.x, \
                                            obj_fixed_frame.pose.orientation.y, \
                                            obj_fixed_frame.pose.orientation.z, \
                                            obj_fixed_frame.pose.orientation.w));

            fixed.getEulerYPR(yaw_bottom_fixed_, pitch_bottom_fixed_, roll_bottom_fixed_);

            ROS_INFO("[fixed frame -> bottom] X:%f, Y:%f, Z:%f, A:%f, B:%f, C:%f", obj_fixed_frame.pose.position.x, \
                                                                        obj_fixed_frame.pose.position.y, \
                                                                        obj_fixed_frame.pose.position.z, \
                                                                        yaw_bottom_fixed_ * 180.0/M_PI, \
                                                                        pitch_bottom_fixed_ * 180.0/M_PI, \
                                                                        roll_bottom_fixed_ * 180.0/M_PI);

            fixed = tf::Matrix3x3(tf::Quaternion(obj_fixed_frame2.pose.orientation.x, \
                                            obj_fixed_frame2.pose.orientation.y, \
                                            obj_fixed_frame2.pose.orientation.z, \
                                            obj_fixed_frame2.pose.orientation.w));

            fixed.getEulerYPR(yaw_front_fixed_, pitch_front_fixed_, roll_front_fixed_);
            ROS_INFO("[fixed frame -> front] X:%f, Y:%f, Z:%f, A:%f, B:%f, C:%f", obj_fixed_frame2.pose.position.x, \
                                                                        obj_fixed_frame2.pose.position.y, \
                                                                        obj_fixed_frame2.pose.position.z, \
                                                                        yaw_front_fixed_ * 180.0/M_PI, \
                                                                        pitch_front_fixed_ * 180.0/M_PI, \
                                                                        roll_front_fixed_ * 180.0/M_PI);

            microrobot_pose.pose.position.x = obj_fixed_frame.pose.position.x;
            microrobot_pose.pose.position.y = obj_fixed_frame.pose.position.y;
            microrobot_pose.pose.position.z = obj_fixed_frame2.pose.position.z;
            microrobot_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(C_ * M_PI / 180.0, B_ * M_PI / 180.0, yaw_bottom_fixed_);
            microrobot_pose.header.stamp = ros::Time::now();
            microrobot_pose.header.frame_id = kuka_fixed_frame_;
            vis_pub_.publish(microrobot_pose);
            ROS_INFO("[Microrobot] x:%f, y:%f, z:%f, A:%f, B:%f, C:%f",
                        obj_fixed_frame.pose.position.x, obj_fixed_frame.pose.position.y, obj_fixed_frame2.pose.position.z,
                        yaw_bottom_fixed_, B_ * M_PI / 180.0, C_ * M_PI / 180.0);


            kuka_command_.data[0] = obj_fixed_frame.pose.position.x * 1e3;
            kuka_command_.data[1] = obj_fixed_frame.pose.position.y * 1e3;
	    kuka_command_.data[2] = obj_fixed_frame2.pose.position.z * 1e3;

	    if(fabs(yaw_bottom_fixed_ - prev_yaw_bottom_fixed_)*180.0/M_PI > 0)
            {
                kuka_command_.data[3] = yaw_bottom_fixed_ * 180.0 / M_PI;
                prev_yaw_bottom_fixed_ = yaw_bottom_fixed_;
            }
            else
                kuka_command_.data[3] = prev_yaw_bottom_fixed_ * 180.0 / M_PI;


            kuka_command_.data[4] = B_;
            kuka_command_.data[5] = C_;

            pub_.publish(kuka_command_);
        }
        catch(tf::ExtrapolationException& msg)
        {
            ROS_INFO("ExtrapolationException");
        }
    }

private:

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_,vis_pub_;

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

    double roll_bottom_fixed_, pitch_bottom_fixed_, yaw_bottom_fixed_, prev_yaw_bottom_fixed_ = 0.0;
    double roll_front_fixed_, pitch_front_fixed_, yaw_front_fixed_, prev_pitch_front_fixed_ = 0.0, z_prev = 0.0;

    std_msgs::Float64MultiArray kuka_command_;
    bool init_command_ = false;
    geometry_msgs::PoseStamped microrobot_pose;

};

} // namespace


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "positioning_4dof");
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
