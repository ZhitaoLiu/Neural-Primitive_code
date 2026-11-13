#ifndef _REPLAN_FSM_H_
#define _REPLAN_FSM_H_

#include <traj_utils/planning_visualization.h>
#include "plan_manage/plan_manager.h"
#include "plan_manage/PrimitiveTraj.h"

#include "Eigen/Eigen"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include "ros/ros.h"

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

namespace npe2e_planner
{
    class npe2eReplanFSM
    {
    private:
        static constexpr bool kEnableIntermediateGuide = true;
        static constexpr double kGuideAngleCosine = -0.5;
        static constexpr double kMinGuideSpeed = 0.2;

        enum FSM_EXEC_STATE
        {
            INIT,
            WAIT_TARGET,
            GEN_NEW_TRAJ,
            REPLAN_TRAJ,
            EXEC_TRAJ,
            EMERGENCY_STOP,
            APPROACH_GOAL
        };
        enum TARGET_TYPE
        {
            MANUAL_TARGET = 1,
            PRESET_TARGET = 2
        };

        PlanningVisualization::Ptr visualization_;
        npe2ePlannerManager::Ptr planner_manager_;
        int goal_id_ = 0, waypoint_num_ = 1;
        std::vector<Eigen::Vector3d> path_global_;

        int target_type_;
        double no_replan_thresh_, replan_thresh_;

        bool trigger_, have_target_, have_odom_;
        FSM_EXEC_STATE exec_state_;

        Eigen::Vector3d odom_pos_, odom_vel_;
        Eigen::Quaterniond odom_q_;
        Eigen::Vector3d imu_acc_;

        Eigen::Vector3d start_pt_, start_vel_, start_acc_;
        Eigen::Vector3d end_pt_;
        Eigen::Vector3d guide_pt_;
        bool guide_created_ = false;
        bool guide_active_ = false;
        Eigen::Matrix3d RWV;

        std::vector<double> waypoints_x_, waypoints_y_, waypoints_z_;
        size_t num_waypoints_;

        double target_z_;
        bool if_yaw_aligned_;
        ros::Time yaw_aligned_stable_start_time_;
        bool turnHead_flag_;

        bool flag_escape_emergency_;
        int collision_replan_count_ = 0;
        bool from_collision_emergency_ = false;

        ros::Timer exec_timer_;
        ros::Subscriber waypoint_sub_, odom_sub_, imu_sub_;

        ros::Publisher PrimitiveTraj_pub_, stop_pub_, imu_acc_pub_, odom_speed_pub_;
        ros::Publisher yaw_aligned_pub_;

        bool callTrajReplan();
        void resetIntermediateGuide();
        bool updateIntermediateGuide(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity);
        void callEmergencyStop(const Eigen::Vector3d &stop_pos);
        bool isTrajCollisionFree();

        void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call);
        void printFSMExecState();
        void planbyGivenWps();

        void publishTrajectory();

        void execFSMCallback(const ros::TimerEvent &);
        void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
        void waypointCallback(const nav_msgs::PathConstPtr &msg);
        void imuCallback(const sensor_msgs::ImuConstPtr &msg);

        void yawAlignmentPub();

    public:
        void init(ros::NodeHandle &nh);
    };

}

#endif
