#include "data_set_gen/point_cloud_sensing.h"

namespace dataSet_gen
{
    PointCloudSensor::PointCloudSensor(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
        : nh_(nh), nh_priv_(nh_priv)
    {
        init();
    }

    void PointCloudSensor::init()
    {
        nh_priv_.param("sensing_horizon", _sensing_horizon, 6.0);
        nh_priv_.param("manager/pcd_range_limit", _pcd_range_limit, 6.0);
        nh_priv_.param("manager/pcd_vertical_deg", _pcd_vertical_deg, 60.0);
        nh_priv_.param("manager/pcd_horizontal_deg", _pcd_horizontal_deg, 90.0);
        nh_priv_.param("map/resolution", _resolution, 0.1);
        nh_priv_.param("map/x_size", _x_size, 50.0);
        nh_priv_.param("map/y_size", _y_size, 20.0);
        nh_priv_.param("map/z_size", _z_size, 10.0);
        nh_priv_.param("publish_rate", _publish_rate, 10.0);

        _inv_resolution = 1.0 / _resolution;
        _gl_xl = -_x_size / 2.0;
        _gl_yl = -_y_size / 2.0;
        _gl_zl = 0.0;
        _GLX_SIZE = static_cast<int>(_x_size * _inv_resolution);
        _GLY_SIZE = static_cast<int>(_y_size * _inv_resolution);
        _GLZ_SIZE = static_cast<int>(_z_size * _inv_resolution);
    }

    void PointCloudSensor::recevGlobalMap(const pcl::PointCloud<pcl::PointXYZ> &cloud_map_recev)
    {

        _voxel_sampler.setLeafSize(_resolution, _resolution, _resolution);
        _voxel_sampler.setInputCloud(cloud_map_recev.makeShared());
        _voxel_sampler.filter(_cloud_all_map);

        _kdtreeLocalMap.setInputCloud(_cloud_all_map.makeShared());
    }

    Eigen::Vector3d PointCloudSensor::gridIndex2coord(const Eigen::Vector3i &index)
    {
        Eigen::Vector3d pt;
        pt(0) = (static_cast<double>(index(0)) + 0.5) * _resolution + _gl_xl;
        pt(1) = (static_cast<double>(index(1)) + 0.5) * _resolution + _gl_yl;
        pt(2) = (static_cast<double>(index(2)) + 0.5) * _resolution + _gl_zl;
        return pt;
    }

    Eigen::Vector3i PointCloudSensor::coord2gridIndex(const Eigen::Vector3d &pt)
    {
        Eigen::Vector3i idx;
        idx(0) = std::min(std::max(static_cast<int>((pt(0) - _gl_xl) * _inv_resolution), 0), _GLX_SIZE - 1);
        idx(1) = std::min(std::max(static_cast<int>((pt(1) - _gl_yl) * _inv_resolution), 0), _GLY_SIZE - 1);
        idx(2) = std::min(std::max(static_cast<int>((pt(2) - _gl_zl) * _inv_resolution), 0), _GLZ_SIZE - 1);
        return idx;
    }

    void PointCloudSensor::sensingPointCloud(const Eigen::Vector3d &curr_pos,
                                             const Eigen::Vector3d &curr_vel,
                                             const Eigen::Quaterniond &curr_orien)
    {

        _local_map.clear();
        _pointIdxRadiusSearch.clear();
        _pointRadiusSquaredDistance.clear();

        Eigen::Matrix3d rot = curr_orien.toRotationMatrix();
        Eigen::Vector3d yaw_vec = rot.col(0);
        Eigen::Vector3d horizontal_yaw_vec(yaw_vec.x(), yaw_vec.y(), 0.0);

        Eigen::Vector3d fov_forward(curr_vel.x(), curr_vel.y(), 0.0);
        if (fov_forward.norm() < 0.2)
            fov_forward = horizontal_yaw_vec;
        fov_forward.normalize();

        const double vertical_tan = std::tan(_pcd_vertical_deg * M_PI / 360.0);
        const double horizontal_cos = std::cos(_pcd_horizontal_deg * M_PI / 360.0);

        pcl::PointXYZ search_point(curr_pos(0), curr_pos(1), curr_pos(2));
        if (_kdtreeLocalMap.radiusSearch(search_point, _sensing_horizon,
                                         _pointIdxRadiusSearch,
                                         _pointRadiusSquaredDistance) > 0)
        {
            for (size_t i = 0; i < _pointIdxRadiusSearch.size(); ++i)
            {
                pcl::PointXYZ pt = _cloud_all_map.points[_pointIdxRadiusSearch[i]];

                if ((fabs(pt.z - curr_pos(2)) / _sensing_horizon) > tan(M_PI / 6.0))
                    continue;

                Eigen::Vector3d pt_vec(pt.x - curr_pos(0), pt.y - curr_pos(1), pt.z - curr_pos(2));
                if (pt_vec.normalized().dot(yaw_vec) < 0.5 * sqrt(2))
                    continue;

                const double distance = pt_vec.norm();
                if (distance > _pcd_range_limit)
                    continue;
                if (distance < 1e-6)
                    continue;
                if (std::abs(pt_vec.z()) / distance > vertical_tan)
                    continue;
                if (pt_vec.normalized().dot(fov_forward) < horizontal_cos)
                    continue;

                _local_map.points.push_back(pt);
            }
        }

        _local_map.width = _local_map.points.size();
        _local_map.height = 1;
        _local_map.is_dense = true;

        pcl::toROSMsg(_local_map, output_cloud);
        output_cloud.header.frame_id = "world";
        output_cloud.header.stamp = ros::Time::now();
    }

}
