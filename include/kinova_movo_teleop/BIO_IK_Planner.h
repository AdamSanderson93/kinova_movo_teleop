#ifndef BIO_IK_PLANNER
#define BIO_IK_PLANNER

#include <string>
#include <vector>

#include <kinova_movo_teleop/BIO_IK_Group.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class BIO_IK_Planner
{
public:
    BIO_IK_Planner(ros::NodeHandle &ph, double rate);

    bool Collision_Callback(moveit::core::RobotState *state,
                            const moveit::core::JointModelGroup *group,
                            const double *values);
    
    bool ready();

    bool plan();
    
    void publish();

protected:
    double center_weight_, min_disp_weight_, hz_, ik_timeout_;

    std::string main_group_, root_link_;
    std::vector<std::string> groups_;

    std::vector<BIO_IK_Group*> bio_groups_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    std::vector<double> previous_joints_;
    robot_state::RobotStatePtr current_state_;

    moveit::core::GroupStateValidityCallbackFn callback_;
};


#endif