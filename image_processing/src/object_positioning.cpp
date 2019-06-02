/* @author Aytaç Kahveci */
/*
/*
It is required to allign the Kuka tool center point to the microrobot pose.
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

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>

double X_ = 0.0;
double Y_ = 0.0;
double Z_ = -13.0;


double A_ = 90.0;
double B_ = 0.0;
double C_ = 180.0;

namespace object_positioning
{

class ObjectPositioning
{

public:

    ObjectPositioning(ros::NodeHandle& nh) : nh_(nh)
    {
        ns_ = nh_.getNamespace();
        obj_sub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, ns_ + "camera_node_bottom/positionDist", 1);
        obj_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(*obj_sub_, listener_, end_effector_frame_, 1);
        obj_filter_->registerCallback(boost::bind(&ObjectPositioning::poseCB, this, _1));

        pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/joint_position_controller/command",1);

        kuka_command_.data.resize(6);
    }

    ~ObjectPositioning()
    {}

    void poseCB(const boost::shared_ptr<const geometry_msgs::PoseStamped>& msg)
    {
        try
        {
            listener_.waitForTransform(kuka_fixed_frame_, camera_frame_, ros::Time(0), ros::Duration(0.1));
            listener_.waitForTransform(end_effector_frame_, camera_frame_, ros::Time(0), ros::Duration(0.1));

            listener_.transformPose(kuka_fixed_frame_, *msg, obj_fixed_frame);
            listener_.transformPose(end_effector_frame_, *msg, obj_end_effector);
            tf::Matrix3x3 fixed(tf::Quaternion(obj_fixed_frame.pose.orientation.x, \
                                            obj_fixed_frame.pose.orientation.y, \
                                            obj_fixed_frame.pose.orientation.z, \
                                            obj_fixed_frame.pose.orientation.w));
            tf::Matrix3x3 end(tf::Quaternion(obj_end_effector.pose.orientation.x, \
                                            obj_end_effector.pose.orientation.y, \
                                            obj_end_effector.pose.orientation.z, \
                                            obj_end_effector.pose.orientation.w));
            fixed.getEulerYPR(yaw_fixed_, pitch_fixed_, roll_fixed_);
            end.getEulerYPR(yaw_end_, pitch_end_, roll_end_);
            ROS_INFO("[fixed frame] X:%f, Y:%f, Z:%f, A:%f, B:%f, C:%f", obj_fixed_frame.pose.position.x, \
                                                                        obj_fixed_frame.pose.position.y, \
                                                                        obj_fixed_frame.pose.position.z, \
                                                                        yaw_fixed_*180.0/M_PI, \
                                                                        pitch_fixed_*180.0/M_PI, \
                                                                        roll_fixed_*180.0/M_PI \
                                                                        );
            ROS_INFO("[end effector frame] X:%f, Y:%f, Z:%f, A:%f, B:%f, C:%f", obj_end_effector.pose.position.x, \
                                                                                obj_end_effector.pose.position.y, \
                                                                                obj_end_effector.pose.position.z, \
                                                                                yaw_end_*180.0/M_PI, \
                                                                                pitch_end_*180.0/M_PI, \
                                                                                roll_end_*180.0/M_PI \
                                                                                );

            kuka_command_.data[0] = obj_fixed_frame.pose.position.x * 1e3;
            kuka_command_.data[1] = obj_fixed_frame.pose.position.y * 1e3;
            kuka_command_.data[2] = Z_;
            kuka_command_.data[3] = A_;
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
    ros::Publisher pub_;

    tf::TransformListener listener_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> *obj_sub_;
    tf::MessageFilter<geometry_msgs::PoseStamped> *obj_filter_;

    geometry_msgs::PoseStamped obj_fixed_frame, obj_end_effector;
    std::string kuka_fixed_frame_ = "kuka_fixed_frame";
    std::string end_effector_frame_ = "end_effector";
    std::string camera_frame_ = "camera_node_front";
    std::string ns_;

    double roll_fixed_, pitch_fixed_, yaw_fixed_;
    double roll_end_, pitch_end_, yaw_end_;

    std_msgs::Float64MultiArray kuka_command_;
};

} // namespace


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "object_positioning");
    ros::NodeHandle nh;

    object_positioning::ObjectPositioning op(nh);

    ros::spin();

    return 0;
}
