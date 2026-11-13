#ifndef PLAN_VIS_H
#define PLAN_VIS_H

#include "data_set_gen/primitive_library.hpp"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>

namespace dataSet_gen
{
    class PlanVisualizer
    {
    public:
        PlanVisualizer(ros::NodeHandle &nh);

        void visualDronePose(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation);
        void visualPlannedTraj(PrimitiveTrajectory *bestPrim_, const Eigen::Vector3d &start_pos, const Eigen::Matrix3d &RWV);
        void visualTargetPoint(const Eigen::Vector3d &target_pos);
        void visualLib(const PrimitiveLibrary &primLib, const Eigen::Vector3d &start_pos, const Eigen::Matrix3d &RWV);
        void visualSensedCloud(const sensor_msgs::PointCloud2 &output_cloud);

    private:
        void pubTimerCallback(const ros::TimerEvent &);

        void publishDrone();
        void publishPlannedTraj();
        void publishTargetPoint();
        void publishLib();
        void publishSensedCloud();

        ros::NodeHandle nh_;
        ros::Publisher drone_pub_;
        ros::Publisher plannedTraj_pub_;
        ros::Publisher target_point_pub_;
        ros::Publisher lib_pub_;
        ros::Publisher pub_cloud_;
        ros::Timer pub_timer_;

        bool has_drone_pose_;
        bool has_planner_traj_;
        bool has_target_point_;
        bool has_lib_;
        bool has_cloud_;

        Eigen::Vector3d position_drone_;
        Eigen::Quaterniond orientation_drone_;

        std::vector<Eigen::Vector3d> pts_plannedTraj_;

        Eigen::Vector3d target_pos_;

        std::vector<std::vector<Eigen::Vector3d>> pts_lib_;

        sensor_msgs::PointCloud2 output_cloud_;
    };
}

#endif