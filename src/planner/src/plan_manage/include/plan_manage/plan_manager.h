#ifndef _PLAN_MANAGER_H_
#define _PLAN_MANAGER_H_

#include <traj_utils/planning_visualization.h>
#include "plan_manage/plan_container.hpp"
#include <plan_env/grid_map.h>

#include <torch/script.h>
#include <torch/torch.h>

#include <string>

#include "ros/package.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/random_sample.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

namespace npe2e_planner
{

    class npe2ePlannerManager
    {
    private:
        PlanningVisualization::Ptr visualization_;

        ros::Publisher modelInf_latency_pub_;
        ros::Publisher planner_cloud_pub_;
        ros::Subscriber dep_odom_sub_, dep_cloud_sub_;
        pcl::RandomSample<pcl::PointXYZ> rs_;
        int sampleSize_;
        double pcd_range_limit_;
        double pcd_vertical_deg_;
        double pcd_horizontal_deg_;

        Eigen::Vector3d odom_pos_, odom_vel_;
        Eigen::Quaterniond odom_q_;

        torch::jit::script::Module model_;
        torch::Device device_;
        bool model_loaded_ = false;

        double fixed_traj_duration_;

        void initNNModels(const std::string &model_name);

    public:
        npe2ePlannerManager() : device_(torch::kCUDA) {}

        LocalTrajData local_data_;
        GridMap::Ptr grid_map_;

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> depthCloudStack_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr plannerCloud_;

        int depthCloudStackNum_, depthCloudCount_;
        bool has_odom_, has_cloud_;

        void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis);
        void odomCallback(const nav_msgs::OdometryConstPtr &odom);
        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img);
        bool trajReplan(const Eigen::Vector3d &start_pt,
                        const Eigen::Vector3d &start_vel,
                        const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &local_target_pt,
                        const Eigen::Matrix3d &RWV);
        Eigen::Matrix3d computeRWV(const Eigen::Vector3d &vel, const Eigen::Quaterniond &odom_q);

        typedef std::unique_ptr<npe2ePlannerManager> Ptr;
    };

}

#endif
