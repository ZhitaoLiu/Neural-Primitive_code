#ifndef POINT_CLOUD_SENSING_H
#define POINT_CLOUD_SENSING_H

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <vector>

namespace dataSet_gen
{
    class PointCloudSensor
    {
    public:
        PointCloudSensor(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);

        void recevGlobalMap(const pcl::PointCloud<pcl::PointXYZ> &cloud_map_recev);

        void sensingPointCloud(const Eigen::Vector3d &curr_pos,
                               const Eigen::Vector3d &curr_vel,
                               const Eigen::Quaterniond &curr_orien);

        sensor_msgs::PointCloud2 output_cloud;

    private:
        void
        init();

        Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i &index);
        Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt);

        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;

        pcl::PointCloud<pcl::PointXYZ> _cloud_all_map;
        pcl::PointCloud<pcl::PointXYZ> _local_map;
        pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
        pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;
        std::vector<int> _pointIdxRadiusSearch;
        std::vector<float> _pointRadiusSquaredDistance;

        sensor_msgs::PointCloud2 _last_output_cloud;

        double _sensing_horizon;
        double _pcd_range_limit, _pcd_vertical_deg, _pcd_horizontal_deg;
        double _resolution, _inv_resolution;
        double _x_size, _y_size, _z_size;
        double _gl_xl, _gl_yl, _gl_zl;
        double _publish_rate;
        int _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;
    };

}
#endif
