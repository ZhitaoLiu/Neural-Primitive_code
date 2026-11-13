#ifndef QP_PRIMITIVE_PLANNER_HPP
#define QP_PRIMITIVE_PLANNER_HPP

#include "data_set_gen/PrimitiveTraj.h"
#include "data_set_gen/primitive_library.hpp"
#include "data_set_gen/map_gen.h"
#include "data_set_gen/plan_vis.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/random_sample.h>
#include <memory>
#include <vector>
#include <unordered_set>

namespace dataSet_gen
{

    struct LocalData
    {
        ros::Time start_time_;
        Eigen::Vector3d start_pos_;
        PrimitiveTrajectory *bestPrim_;
        LocalData() : bestPrim_(nullptr) {}
    };

    class QPPrimitivePlanner
    {
    public:
        QPPrimitivePlanner(ros::NodeHandle &nh, ros::NodeHandle &nh_priv, PrimitiveLibrary &primLib);

        bool initPlanModules(const Eigen::Vector3d &pos_worldFrame,
                             const Eigen::Vector3d &vel_worldFrame,
                             const Eigen::Quaterniond &orientation,
                             const sensor_msgs::PointCloud2 &cloud_msg,
                             const Eigen::Vector3d &target_pos);

        void cloudInput(const sensor_msgs::PointCloud2 &cloud_msg);

        bool trajReplan(const Eigen::Vector3d &start_pt,
                        const Eigen::Vector3d &target_pt,
                        const Eigen::Matrix3d &RWV);

        const Eigen::Matrix3d &getRWV() const;

        const LocalData &getLocalData() const;

        void publishTrajectory(ros::Publisher &traj_pub);

        pcl::PointCloud<pcl::PointXYZ>::Ptr processed_sensedCloud_velFrame_;
        bool has_odom_ = false;

    private:
        LocalData local_data_;

        bool selectBestPrimitive(const Eigen::Vector3d &start_Pos,
                                 const Eigen::Vector3d &targetPos,
                                 const Eigen::Matrix3d &RWV);

        bool labelObsCollisionPrims(const Eigen::Vector3d &start_pt,
                                    const Eigen::Matrix3d &RWV,
                                    const std::vector<PrimitiveTrajectory *> &candidates);

        Eigen::Matrix3d computeRWV(const Eigen::Vector3d &vel_worldFrame,
                                   const Eigen::Quaterniond &odom_q,
                                   double xV_offset = 0.0);

        PrimitiveLibrary &primLib_;

        ros::NodeHandle nh_, nh_priv_;

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> depthCloudStack_ = {};
        int depthCloudCount_ = 0;
        int depthCloudStackNum_ = 1;
        bool has_cloud_ = false;

        int voxelNumX_, voxelNumY_, voxelNumZ_, voxelNumAll_;
        double voxelX_, voxelY_, voxelZ_;

        pcl::RandomSample<pcl::PointXYZ> rs_;

        std::unordered_set<int> clearPrimList_;

        double boxX_, boxY_, boxZ_;
        double voxelSize_;
        int sampleSize_;
        double x_size_, y_size_, z_size_;

        Eigen::Matrix3d RWV_;

        Eigen::Vector3d pos_worldFrame_;
        Eigen::Vector3d vel_worldFrame_;

        PlanVisualizer vis_;
    };

}

#endif