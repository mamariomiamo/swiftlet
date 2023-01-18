#pragma once
#include <ros/ros.h>
// #include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/HomePosition.h>
#include <std_msgs/Byte.h>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <tf/transform_listener.h>
#include "quadrotor_msgs/TrajectoryPoint.h"
#include <memory>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

namespace PX4FSM
{
    enum UserCommand
    {
        kTakeOff,
        kMission,
        kLand,
    };

    class PX4FSM
    {
    public:
        // https://google.github.io/styleguide/cppguide.html#Enumerator_Names
        enum UavTaskState
        {
            kIdle = 0,
            kManualCtrl,
            kCmdCtrl,
            kAutoTakeOff,
            kAutoHover,
            kAutoLand
        };

        enum CommandType
        {
            kPositionOnly,
            kPVA,
        };

        PX4FSM(ros::NodeHandle &nh);
        ~PX4FSM() = default;

    private:
        ros::NodeHandle nh_;
        ros::Subscriber uav_state_sub_;
        ros::Subscriber uav_pose_sub_;
        ros::Subscriber user_cmd_sub_;

        ros::Publisher uav_pos_sp_pub_;  // used for sending position setpoints to px4
        ros::Publisher uav_traj_sp_pub_; // used for sening traj (i.e. p,v,a setpoints) to px4

        ros::ServiceClient set_mode_client_;
        ros::ServiceClient arming_client_;
        ros::ServiceClient takeoff_client_;
        ros::ServiceClient land_client_;
        mavros_msgs::CommandBool arm_cmd_;

        ros::Timer fsm_timer_;

        std::string uav_id_;
        double takeoff_height_;
        bool auto_start_;
        bool takeoff_flag_;

        UavTaskState uav_task_state_;
        mavros_msgs::State uav_state_; // uav mavros state
        geometry_msgs::PoseStamped uav_pose_;
        geometry_msgs::Point hover_position_;
        geometry_msgs::Point takeoff_position_;
        mavros_msgs::PositionTarget pos_sp_;
        double takeoff_yaw_, hover_yaw_;

        private:
        void UavStateCallback(const mavros_msgs::State::ConstPtr &uav_state);
        void UavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &uav_pose);
        void UserCommandCallback(const std_msgs::Byte::ConstPtr &user_cmd);
        void FSMTimerCallback(const ros::TimerEvent &);

        bool SetOffboardAndArm();
        void SendPositionTarget(mavros_msgs::PositionTarget &pos_sp, const CommandType &cmd_type);
        void ChangeTaskState(UavTaskState new_state);
        bool TakeoffComplete(const geometry_msgs::Point& current_pos, const geometry_msgs::Point& desired_pos);
        std::string TaskStateString(const UavTaskState &task_state);
    };

}; // namespace PX4FSM