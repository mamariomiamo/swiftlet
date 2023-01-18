#include "../include/px4ctrl/px4fsm.h"
#include "../include/px4ctrl/helper.h"

namespace PX4FSM
{
    PX4FSM::PX4FSM(ros::NodeHandle &nh) : nh_(nh)
    {
        nh_.param<std::string>("drone_id", uav_id_, "drone0");
        nh_.param<double>("takeoff_height", takeoff_height_, 1.0);

        takeoff_flag_ = false;

        uav_task_state_ = UavTaskState::kIdle;

        // subscriber
        uav_state_sub_ = nh_.subscribe<mavros_msgs::State>("/" + uav_id_ + "/mavros/state", 10, &PX4FSM::UavStateCallback, this);

        uav_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/" + uav_id_ + "/mavros/local_position/pose", 10,
                                                                  &PX4FSM::UavPoseCallback, this);

        user_cmd_sub_ = nh_.subscribe<std_msgs::Byte>("/user_cmd", 1, &PX4FSM::UserCommandCallback, this);

        // publihser
        uav_pos_sp_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/" + uav_id_ + "/mavros/setpoint_position/local", 10);
        uav_traj_sp_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/" + uav_id_ + "/mavros/setpoint_raw/local", 10);

        // Mavros PX4 service client
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/" + uav_id_ + "/mavros/set_mode");
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/" + uav_id_ + "/mavros/cmd/arming");
        takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/" + uav_id_ + "/mavros/cmd/takeoff");
        land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/" + uav_id_ + "/mavros/cmd/land");

        fsm_timer_ = nh_.createTimer(ros::Duration(0.05), &PX4FSM::FSMTimerCallback, this, false, false);
    }

    void PX4FSM::UavStateCallback(const mavros_msgs::State::ConstPtr &uav_state)
    {
        uav_state_ = *uav_state;
    }

    void PX4FSM::UavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &uav_pose)
    {
        uav_pose_ = *uav_pose;
    }

    void PX4FSM::UserCommandCallback(const std_msgs::Byte::ConstPtr &user_cmd)
    {
        int cmd = user_cmd->data;
        std::cout << "user command received: " << cmd << std::endl;
        switch (cmd)
        {
        case UserCommand::kTakeOff:
        {
            ROS_INFO("TAKEOFF command received!");
            ChangeTaskState(UavTaskState::kAutoTakeOff);

            takeoff_position_ = uav_pose_.pose.position;
            takeoff_position_.z += takeoff_height_;

            takeoff_yaw_ = px4fsm_helper::GetYawFromQuat(uav_pose_.pose.orientation);

            if (SetOffboardAndArm())
            {
                fsm_timer_.start();
                takeoff_flag_ = true;
            }
            break;
        }
        case UserCommand::kMission:
        {
            uav_task_state_ = UavTaskState::kAutoHover;
            break;
        }
        case UserCommand::kLand:
        {
            uav_task_state_ = UavTaskState::kAutoLand;
            break;
        }
        default:
            break;
        }
    }

    void PX4FSM::FSMTimerCallback(const ros::TimerEvent &)
    {
        switch (uav_task_state_)
        {
        case UavTaskState::kIdle:
        {
            break;
        }

        case UavTaskState::kAutoTakeOff:
        {
            pos_sp_.header.stamp = ros::Time::now();
            pos_sp_.position = takeoff_position_;
            pos_sp_.yaw = takeoff_yaw_;
            SendPositionTarget(pos_sp_, CommandType::kPositionOnly);

            if (TakeoffComplete(uav_pose_.pose.position, takeoff_position_))
            {
                ChangeTaskState(UavTaskState::kAutoHover);
                hover_position_ = takeoff_position_;
                hover_yaw_ = takeoff_yaw_;
            }

            break;
        }

        case UavTaskState::kAutoHover:
        {
            pos_sp_.header.stamp = ros::Time::now();
            pos_sp_.position = hover_position_;
            pos_sp_.yaw = hover_yaw_;
            SendPositionTarget(pos_sp_, CommandType::kPositionOnly);
            break;
        }

        case UavTaskState::kCmdCtrl:
        {
            break;
        }

        default:
            break;
        }
    }

    bool PX4FSM::SetOffboardAndArm()
    {
        ros::Rate rate(20.0);
        ros::Time last_request = ros::Time::now();
        pos_sp_.position = uav_pose_.pose.position;
        pos_sp_.yaw = px4fsm_helper::GetYawFromQuat(uav_pose_.pose.orientation);

        // send a few setpoints before starting
        for (int i = 10; ros::ok() && i > 0; --i)
        {
            SendPositionTarget(pos_sp_, CommandType::kPositionOnly);
            ros::spinOnce();
            rate.sleep();
        }

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        bool is_mode_ready = false;
        last_request = ros::Time::now();
        arm_cmd_.request.value = true;

        while (!is_mode_ready)
        {
            if (uav_state_.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                ROS_INFO("Try set offboard");
                if (set_mode_client_.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }
            else
            {
                if (!uav_state_.armed && (ros::Time::now() - last_request > ros::Duration(1.0)))
                {
                    ROS_INFO("Try Arming");
                    if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success)
                    {
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
                // is_mode_ready = (uav_state_.mode == "OFFBOARD");
            }
            SendPositionTarget(pos_sp_, CommandType::kPositionOnly);
            is_mode_ready = (uav_state_.mode == "OFFBOARD" && uav_state_.armed);
            ros::spinOnce();
            rate.sleep();
        }

        if (is_mode_ready)
        {
            ROS_INFO("Offboard mode activated!");
        }

        return is_mode_ready;
    }

    void PX4FSM::SendPositionTarget(mavros_msgs::PositionTarget &pos_sp, const CommandType &cmd_type)
    {
        int type_mask;
        switch (cmd_type)
        {
        case CommandType::kPositionOnly:
        {
            // ignore vx, vy, vz, ax, ay, az, yaw_rate i.e. 8+16+32+64+128+256+2048=2552
            type_mask = 2552;
            break;
        }

        case CommandType::kPVA:
        {
            // use p,v,a and ignore yaw_rate
            type_mask = 2048;
            break;
        }

        default:
        {
            std::cout << KRED << "Invalid CommandType\n";
            break;
        }
        }

        pos_sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        pos_sp.header.stamp = ros::Time::now();
        pos_sp.type_mask = type_mask;
        uav_traj_sp_pub_.publish(pos_sp);
    }

    void PX4FSM::ChangeTaskState(UavTaskState new_state)
    {
        UavTaskState prev_uav_task_state = uav_task_state_;
        uav_task_state_ = new_state;

        std::string prev_state = TaskStateString(prev_uav_task_state);
        std::string current_state = TaskStateString(uav_task_state_);

        std::cout << KGRN << "[PX4CTRL] UAV Task State is changed from " << prev_state << " to " << current_state << KNRM << std::endl;
    }

    bool PX4FSM::TakeoffComplete(const geometry_msgs::Point &current_pos, const geometry_msgs::Point &desired_pos)
    {
        Eigen::Vector3d current_position = px4fsm_helper::Point2Vect(current_pos);
        Eigen::Vector3d desired_position = px4fsm_helper::Point2Vect(desired_pos);

        if ((current_position - desired_position).norm() < 0.1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    std::string PX4FSM::TaskStateString(const UavTaskState &task_state)
    {
        std::string ret;
        switch (task_state)
        {
        case UavTaskState::kIdle:
        {
            ret = "Idle";
            break;
        }

        case UavTaskState::kAutoTakeOff:
        {
            ret = "AutoTakeOff";
            break;
        }

        case UavTaskState::kAutoHover:
        {
            ret = "AutoHover";
            break;
        }

        case UavTaskState::kCmdCtrl:
        {
            ret = "CmdCtrl";
            break;
        }

        case UavTaskState::kAutoLand:
        {
            ret = "AutoLand";
            break;
        }

        default:
        {
            ret = "Unknown";
            break;
        }
        }

        return ret;
    }

}; // namespace PX4FSM