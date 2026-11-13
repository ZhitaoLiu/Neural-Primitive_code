#ifndef MAP_GEN_H
#define MAP_GEN_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <random>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/package.h>
#include <filesystem>

namespace dataSet_gen
{

    class randomMapGenerator
    {
    public:
        randomMapGenerator(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);

        void generateRandomSimulatedMap(int obs_num_);

        pcl::PointCloud<pcl::PointXYZ> cloud_map_;
        sensor_msgs::PointCloud2 globalMap_pcd;

    private:
        void mapPubTimerCallback(const ros::TimerEvent &event);

        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;
        ros::Publisher all_map_pub_;
        ros::Timer map_pub_timer_;
        std::random_device rd_;
        std::default_random_engine eng_{rd_()};
        std::uniform_real_distribution<double> rand_x_;
        std::uniform_real_distribution<double> rand_y_;
        std::uniform_real_distribution<double> rand_w_;
        std::uniform_real_distribution<double> rand_h_;
        std::uniform_real_distribution<double> rand_inf_;
        std::uniform_real_distribution<double> rand_radius_;
        std::uniform_real_distribution<double> rand_radius2_;
        std::uniform_real_distribution<double> rand_theta_;
        std::uniform_real_distribution<double> rand_z_;
        bool map_ok_{false};
        double x_size_, y_size_, z_size_;
        double x_l_, x_h_, y_l_, y_h_;
        double w_l_, w_h_, h_l_, h_h_;
        double z_limit_, resolution_;
        double min_dist_;
        double publish_rate_;
        int obs_num_;
        int circle_num_;
        double radius_l_, radius_h_, z_l_, z_h_, theta_;
        pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
    };

}

#endif