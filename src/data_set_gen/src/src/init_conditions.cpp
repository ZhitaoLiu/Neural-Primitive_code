#include "data_set_gen/init_conditions.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>

namespace dataSet_gen
{

    ConditionInitializer::ConditionInitializer(const randomMapGenerator &map_generator, ros::NodeHandle &nh_priv)
        : map_generator_(map_generator)
    {
        nh_priv.param("map/x_size", x_size_, 30.0);
        nh_priv.param("map/y_size", y_size_, 20.0);
        nh_priv.param("map/z_size", z_size_, 10.0);
        nh_priv.param("map/resolution", resolution_, 0.1);

        nh_priv.param("init/vel_magnitude_max", vel_magnitude_max_, 10.0);
        nh_priv.param("init/acc_max", acc_max_, 12.0);

        x_l_ = -x_size_ / 2.0;
        x_h_ = x_size_ / 2.0;
        y_l_ = -y_size_ / 2.0;
        y_h_ = y_size_ / 2.0;
        z_l_ = 0.0;
        z_h_ = z_size_;

        rand_x_ = std::uniform_real_distribution<double>(x_l_, x_h_);
        rand_y_ = std::uniform_real_distribution<double>(y_l_, y_h_);
        rand_z_ = std::uniform_real_distribution<double>(z_l_, z_h_);
        rand_yaw_ = std::uniform_real_distribution<double>(0.0, 2.0 * M_PI);
        rand_vel_mag_ = std::uniform_real_distribution<double>(0.0, vel_magnitude_max_);
        rand_acc_ = std::uniform_real_distribution<double>(-acc_max_, acc_max_);
    }

    InitState ConditionInitializer::randomInitStateGen(Eigen::Vector3d iniVel_body, Eigen::Vector3d iniAcc_velFrame)
    {
        InitState state;
        bool valid_position = false;
        bool valid_target = false;
        int max_attempts = 1000;

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(map_generator_.cloud_map_.makeShared());

        while (!valid_position && max_attempts-- > 0)
        {
            state.iniPos_worldFrame.x() = rand_x_(eng_);
            state.iniPos_worldFrame.y() = rand_y_(eng_);
            state.iniPos_worldFrame.z() = rand_z_(eng_);

            pcl::PointXYZ search_point;
            search_point.x = static_cast<float>(state.iniPos_worldFrame.x());
            search_point.y = static_cast<float>(state.iniPos_worldFrame.y());
            search_point.z = static_cast<float>(state.iniPos_worldFrame.z());

            std::vector<int> point_idx;
            std::vector<float> point_distances;
            kdtree.nearestKSearch(search_point, 1, point_idx, point_distances);

            if (!point_distances.empty() && std::sqrt(point_distances[0]) > 1.0f)
                valid_position = true;
        }
        if (!valid_position)
        {
            ROS_WARN("Failed to find valid initial position after max attempts, set to origin.");
            state.iniPos_worldFrame = Eigen::Vector3d(0.0, 0.0, 2.0);
        }

        double yaw = rand_yaw_(eng_);
        state.iniQuaterniond = Eigen::Quaterniond(
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

        state.iniVel_worldFrame = state.iniQuaterniond * iniVel_body;
        state.iniAcc_velFrame = iniAcc_velFrame;

        Eigen::Vector3d forward_vec = state.iniQuaterniond * Eigen::Vector3d::UnitX();

        max_attempts = 1000;
        while (!valid_target && max_attempts-- > 0)
        {
            state.target_pos_worldFrame.x() = rand_x_(eng_);
            state.target_pos_worldFrame.y() = rand_y_(eng_);
            state.target_pos_worldFrame.z() = rand_z_(eng_);

            Eigen::Vector3d dir_world = state.target_pos_worldFrame - state.iniPos_worldFrame;

            Eigen::Vector3d dir_velFrame = state.iniQuaterniond.inverse() * dir_world;
            double dir_norm = dir_velFrame.norm();
            if (dir_norm > 1e-6)
            {
                state.dir_target_from_drone = dir_velFrame / dir_norm;
            }
            else
            {
                state.dir_target_from_drone = forward_vec;
            }

            pcl::PointXYZ target_point;
            target_point.x = static_cast<float>(state.target_pos_worldFrame.x());
            target_point.y = static_cast<float>(state.target_pos_worldFrame.y());
            target_point.z = static_cast<float>(state.target_pos_worldFrame.z());

            std::vector<int> target_idx;
            std::vector<float> target_distances;
            kdtree.nearestKSearch(target_point, 1, target_idx, target_distances);

            if (!target_distances.empty() && std::sqrt(target_distances[0]) > 1.0f)
                valid_target = true;
        }
        if (!valid_target)
        {
            ROS_WARN("Failed to find valid target position in front after max attempts, set to be in front of the drone.");
            state.target_pos_worldFrame =
                state.iniPos_worldFrame + forward_vec * 5.0 + Eigen::Vector3d(0, 0, 2.0);
        }

        return state;
    }

}