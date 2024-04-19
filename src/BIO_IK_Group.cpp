#include <kinova_movo_teleop/BIO_IK_Group.h>

    BIO_IK_Group::BIO_IK_Group(ros::NodeHandle &ph, tf2_ros::Buffer &tf_buffer, std::string group_name, std::vector<std::string> joint_names, std::string base_link, double hz, std::vector<double> joint_max_speed)
    : tf_buffer_(tf_buffer),
    group_name_(group_name),
    base_link_(base_link),
    max_speeds_(joint_max_speed),
    joint_names_(joint_names),
    freq_(hz)
    {
        std::string sub_topic, pub_topic;

        ros::NodeHandle grouph(ph, group_name);
        grouph.param<double>("position_weight", position_weight_, 0.0);
        grouph.param<double>("orient_weight", orientation_weight_, 0.0);
        grouph.param<double>("pose_weight", pose_weight_, 0.0);

        grouph.param<std::string>("goal_link", goal_link_, "base_link");
        grouph.param<std::string>("sub_topic", sub_topic, "cmd");
        grouph.param<std::string>("pub_topic", pub_topic, "traj");
        
        
        sub_ = ph.subscribe<geometry_msgs::PoseStamped>(sub_topic, 1, boost::bind(&BIO_IK_Group::commandCallback, this, _1));

        pub_ = ph.advertise<trajectory_msgs::JointTrajectory>(pub_topic, 1, true);
        
        traj_.joint_names = joint_names_;
        trajectory_msgs::JointTrajectoryPoint temp_point;
        temp_point.time_from_start = ros::Rate(freq_).cycleTime();
        traj_.points.push_back(temp_point);
    }

    void BIO_IK_Group::add_goals(std::vector<std::unique_ptr<bio_ik::Goal>>& goal_list)
    {
        group_mutex_.lock();

        if (pose_weight_)
            goal_list.emplace_back(new bio_ik::PoseGoal(goal_link_, goal_pose_.getOrigin(), goal_pose_.getRotation(), pose_weight_));
        if (position_weight_)
            goal_list.emplace_back(new bio_ik::PositionGoal(goal_link_, goal_pose_.getOrigin(), position_weight_));
        if (orientation_weight_)
            goal_list.emplace_back(new bio_ik::OrientationGoal(goal_link_, goal_pose_.getRotation(), orientation_weight_));
        
        group_mutex_.unlock();
    }

    void BIO_IK_Group::create_trajectory(std::vector<double> position)
    {
        traj_.header.frame_id = base_link_;
        traj_.points[0].positions = position;
    }

    void BIO_IK_Group::publish_trajectory()
    {
        pub_.publish(traj_);
    }
    
    bool BIO_IK_Group::ready()
    {
        return initialized_;
    }

    void BIO_IK_Group::commandCallback(const geometry_msgs::PoseStampedConstPtr& cmd)
    {
        group_mutex_.lock();
        
        try
        {
            geometry_msgs::PoseStamped temp;
            temp.pose = cmd->pose;
            temp.header = cmd->header;
            tf2::convert(tf_buffer_.transform(temp, base_link_).pose, goal_pose_);
            initialized_ = true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_STREAM("Could NOT transform: " << ex.what());
        }

        group_mutex_.unlock();
    }

    std::vector<double> BIO_IK_Group::interpolate_motion(std::vector<double> start, std::vector<double> end)
    {
        double scaling = 1;
        for (int i = 0; i < end.size(); ++i)
        {
            if (max_speeds_[i] <= 0)
                continue;

            double temp_scale = std::abs(start[i] - end[i]) * freq_ / max_speeds_[i];

            if (temp_scale > scaling && !isinf(temp_scale))
            {
                scaling = temp_scale;
            }
        }

        std::vector<double> interp(end);

        if (scaling > 1.0)
        {
            for (int j = 0; j < interp.size(); ++j)
            {
                interp[j] = start[j] + (end[j] - start[j])/scaling;
            }
        }

        return interp;
    }
