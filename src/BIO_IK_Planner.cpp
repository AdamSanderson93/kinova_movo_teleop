#include <kinova_movo_teleop/BIO_IK_Planner.h>

BIO_IK_Planner::BIO_IK_Planner(ros::NodeHandle &ph, double rate):
tf_listener_(tf_buffer_),
hz_(rate),
current_state_(nullptr)
{
    ph.param("main_group", main_group_, std::string("movo_full"));
    ph.param("root_link", root_link_, std::string("base_link"));

    // Load parameters
    ph.getParam("groups", groups_);
    ph.param("center_joints_weight", center_weight_, 0.01);
    ph.param("min_displacement_weight", min_disp_weight_, 0.2);

    ph.param("ik_timeout", ik_timeout_, 0.007);
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    if (!planning_scene_monitor_->getPlanningScene())
    {
      ROS_ERROR_STREAM("PlanningSceneMonitor setup failed.");
      exit(-1);
    }

    // Start the planning scene monitor
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
        true);
    planning_scene_monitor_->startStateMonitor();

    if (!planning_scene_monitor_->getStateMonitor()->waitForCompleteState(main_group_, 10.0))
    {
      ROS_ERROR_STREAM("Timeout waiting for state");
      exit(-1);
    }

    callback_ = boost::bind(&BIO_IK_Planner::Collision_Callback, this, _1, _2, _3);

    // Wait for initialization
    current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
    current_state_->copyJointGroupPositions(main_group_, previous_joints_);

    for (const std::string& group_name : groups_)
    {
        const moveit::core::JointModelGroup *jmg = current_state_->getJointModelGroup(group_name);
        const std::vector<const moveit::core::JointModel::Bounds*>& bounds = jmg->getActiveJointModelsBounds();
        const std::vector<unsigned int>& bij = jmg->getKinematicsSolverJointBijection();
        std::vector<std::string> joint_names = jmg->getActiveJointModelNames();

        std::vector<double> max_speeds;

        for (std::size_t j = 0; j < joint_names.size(); ++j)
        {
            const std::vector<moveit::core::VariableBounds>* var_bounds = bounds[bij[j]];
            if (var_bounds->size() != 1)
            {
                max_speeds.push_back(0.0);
            }
            else
            {
                max_speeds.push_back((*var_bounds)[0].max_velocity_);
            }
        }

        bio_groups_.push_back(new BIO_IK_Group(ph, tf_buffer_, group_name, joint_names, root_link_, hz_, max_speeds));
    }
}

bool BIO_IK_Planner::ready()
{
    for (auto& group : bio_groups_)
    {
        if (!group->ready())
        {
            return false;
        }
    }
    return true;
}

bool BIO_IK_Planner::plan()
{
    current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();

    bio_ik::BioIKKinematicsQueryOptions query;
    query.return_approximate_solution = true;
    
    query.lock_redundant_joints = true;
    
    query.goals.emplace_back(new bio_ik::CenterJointsGoal(center_weight_, true));
    query.goals.emplace_back(new bio_ik::MinimalDisplacementGoal(min_disp_weight_));

    for (auto& group : bio_groups_)
    {
        group->add_goals(query.goals);
    }

    robot_state::RobotState solution_state(*current_state_);
    robot_state::RobotState interp_state(*current_state_);

    bool found_ik = solution_state.setFromIK(current_state_->getJointModelGroup(main_group_),
                                            EigenSTL::vector_Isometry3d(),
                                            ros::V_string(),
                                            ik_timeout_,
                                            callback_,
                                            query);
    if (!found_ik)
    {
        ROS_ERROR_STREAM_THROTTLE(1.0, "No IK Found");
        return false;
    }

    planning_scene_monitor::LockedPlanningSceneRO scene_ro(planning_scene_monitor_);
    if (scene_ro->isStateColliding(solution_state, main_group_, true))
    {
        ROS_ERROR_STREAM_THROTTLE(1.0, "Planned End State Colliding");
        return false;
    }

    for (auto& group : bio_groups_)
    {
        std::vector<double> joint_state, start_state;
        solution_state.copyJointGroupPositions(group->group_name_, joint_state);
        current_state_->copyJointGroupPositions(group->group_name_, start_state);
        joint_state = group->interpolate_motion(start_state, joint_state);
        interp_state.setJointGroupPositions(group->group_name_, joint_state);
    }

    if (scene_ro->isStateColliding(interp_state, main_group_))
    {
    ROS_WARN_STREAM_THROTTLE(1, "Motion is in collision");
    return false;
    }

    for (auto& group : bio_groups_)
    {
        std::vector<double> joints;
        interp_state.copyJointGroupPositions(group->group_name_, joints);
        group->create_trajectory(joints);
    }
    *current_state_ = interp_state;
    current_state_->copyJointGroupPositions(main_group_, previous_joints_);
    return true;
}
    
void BIO_IK_Planner::publish()
{
    for (auto& group : bio_groups_)
    {
        group->publish_trajectory();
    }
}


bool BIO_IK_Planner::Collision_Callback(moveit::core::RobotState *state,
                            const moveit::core::JointModelGroup *group,
                            const double *values)
{
    planning_scene_monitor::LockedPlanningSceneRO scene_ro(planning_scene_monitor_);
    state->setJointGroupPositions(group, values);
    state->update();
    return !scene_ro->isStateColliding(*state, group->getName());
}

