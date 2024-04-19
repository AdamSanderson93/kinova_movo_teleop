#ifndef BIO_IK_GROUP
#define BIO_IK_GROUP

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/time.h>

#include <boost/thread/mutex.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <bio_ik/bio_ik.h>

class BIO_IK_Group
{
public:
    BIO_IK_Group(ros::NodeHandle &ph, tf2_ros::Buffer &tf_buffer, std::string group_name, std::vector<std::string> joint_names, std::string base_link, double hz, std::vector<double> joint_max_speed);

    void add_goals(std::vector<std::unique_ptr<bio_ik::Goal>>& goal_list);

    void publish_trajectory();

    void create_trajectory(std::vector<double> position);

    bool ready();

    void commandCallback(const geometry_msgs::PoseStampedConstPtr& cmd);

    std::vector<double> interpolate_motion(std::vector<double> start_position, std::vector<double> end_position);

    std::string group_name_;

protected:
    
    boost::mutex group_mutex_;

    ros::Publisher pub_;
    ros::Subscriber sub_;

    bool initialized_=false;
    
    double position_weight_, orientation_weight_, pose_weight_, freq_;

    tf2::Transform goal_pose_;

    trajectory_msgs::JointTrajectory traj_;

    std::string goal_link_, base_link_;

    std::vector<std::string> joint_names_;
    std::vector<double> max_speeds_;


    tf2_ros::Buffer& tf_buffer_;
    
};

#endif