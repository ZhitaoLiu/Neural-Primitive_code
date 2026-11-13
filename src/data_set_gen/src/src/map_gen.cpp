#include "data_set_gen/map_gen.h"

namespace fs = std::filesystem;

namespace dataSet_gen
{

    randomMapGenerator::randomMapGenerator(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
        : nh_(nh), nh_priv_(nh_priv)
    {

        nh_priv_.param("map/x_size", x_size_, 50.0);
        nh_priv_.param("map/y_size", y_size_, 20.0);
        nh_priv_.param("map/z_size", z_size_, 10.0);
        nh_priv_.param("map/resolution", resolution_, 0.1);
        nh_priv_.param("publish_rate", publish_rate_, 10.0);

        nh_priv_.param("ObstacleShape/lower_rad", w_l_, 0.3);
        nh_priv_.param("ObstacleShape/upper_rad", w_h_, 0.8);
        nh_priv_.param("ObstacleShape/lower_hei", h_l_, 2.0);
        nh_priv_.param("ObstacleShape/upper_hei", h_h_, 10.0);

        nh_priv_.param("ObstacleShape/radius_l", radius_l_, 0.3);
        nh_priv_.param("ObstacleShape/radius_h", radius_h_, 0.7);
        nh_priv_.param("ObstacleShape/z_l", z_l_, 1.0);
        nh_priv_.param("ObstacleShape/z_h", z_h_, 5.0);
        nh_priv_.param("ObstacleShape/theta", theta_, 7.0);

        nh_priv_.param("min_distance", min_dist_, 1.0);

        x_l_ = -x_size_ / 2.0;
        x_h_ = x_size_ / 2.0;
        y_l_ = -y_size_ / 2.0;
        y_h_ = y_size_ / 2.0;

        z_limit_ = z_size_;

        all_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);
        map_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &randomMapGenerator::mapPubTimerCallback, this);
    }

    void randomMapGenerator::generateRandomSimulatedMap(int obs_num_)
    {

        obs_num_ = std::min(obs_num_, static_cast<int>(x_size_ * 10));
        int circle_num_ = std::ceil(0.25 * obs_num_);

        cloud_map_.clear();
        pcl::PointXYZ pt_random;
        std::vector<Eigen::Vector2d> obs_position;

        rand_x_ = std::uniform_real_distribution<double>(x_l_, x_h_);
        rand_y_ = std::uniform_real_distribution<double>(y_l_, y_h_);
        rand_w_ = std::uniform_real_distribution<double>(w_l_, w_h_);
        rand_h_ = std::uniform_real_distribution<double>(h_l_, h_h_);
        rand_inf_ = std::uniform_real_distribution<double>(0.5, 1.5);
        rand_radius_ = std::uniform_real_distribution<double>(radius_l_, radius_h_);
        rand_radius2_ = std::uniform_real_distribution<double>(radius_l_, 1.2);
        rand_theta_ = std::uniform_real_distribution<double>(-theta_, theta_);
        rand_z_ = std::uniform_real_distribution<double>(z_l_, z_h_);

        for (int i = 0; i < obs_num_ && ros::ok(); ++i)
        {
            double x = rand_x_(eng_);
            double y = rand_y_(eng_);
            double w = rand_w_(eng_);
            double inf = rand_inf_(eng_);

            bool flag_continue = false;
            for (const auto &p : obs_position)
            {
                if ((Eigen::Vector2d(x, y) - p).norm() < min_dist_)
                {
                    --i;
                    flag_continue = true;
                    break;
                }
            }
            if (flag_continue)
                continue;

            obs_position.emplace_back(x, y);

            x = std::floor(x / resolution_) * resolution_ + resolution_ / 2.0;
            y = std::floor(y / resolution_) * resolution_ + resolution_ / 2.0;

            int wid_num = std::ceil((w * inf) / resolution_);
            double radius = (w * inf) / 2;

            for (int r = -wid_num / 2; r < wid_num / 2; ++r)
            {
                for (int s = -wid_num / 2; s < wid_num / 2; ++s)
                {
                    double h = rand_h_(eng_);
                    int hei_num = std::ceil(h / resolution_);
                    for (int t = 0; t < hei_num; ++t)
                    {
                        double temp_x = x + (r + 0.5) * resolution_ + 1e-2;
                        double temp_y = y + (s + 0.5) * resolution_ + 1e-2;
                        double temp_z = (t + 0.5) * resolution_ + 1e-2;
                        if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() <= radius)
                        {
                            pt_random.x = temp_x;
                            pt_random.y = temp_y;
                            pt_random.z = temp_z;
                            cloud_map_.points.push_back(pt_random);
                        }
                    }
                }
            }
        }

        for (int i = 0; i < circle_num_; ++i)
        {
            double x = rand_x_(eng_);
            double y = rand_y_(eng_);
            double z = rand_z_(eng_);

            x = std::floor(x / resolution_) * resolution_ + resolution_ / 2.0;
            y = std::floor(y / resolution_) * resolution_ + resolution_ / 2.0;
            z = std::floor(z / resolution_) * resolution_ + resolution_ / 2.0;

            double theta = rand_theta_(eng_);
            Eigen::Matrix3d rotate;
            rotate << std::cos(theta), -std::sin(theta), 0.0,
                std::sin(theta), std::cos(theta), 0.0,
                0.0, 0.0, 1.0;

            double radius1 = rand_radius_(eng_);
            double radius2 = rand_radius2_(eng_);

            Eigen::Vector3d cpt;
            for (double angle = 0.0; angle < 6.282; angle += resolution_ / 2)
            {
                cpt(0) = 0.0;
                cpt(1) = radius1 * std::cos(angle);
                cpt(2) = radius2 * std::sin(angle);

                Eigen::Vector3d cpt_if = rotate * cpt + Eigen::Vector3d(x, y, z);
                pt_random.x = cpt_if(0);
                pt_random.y = cpt_if(1);
                pt_random.z = cpt_if(2);
                cloud_map_.push_back(pt_random);
            }
        }

        cloud_map_.width = cloud_map_.points.size();
        cloud_map_.height = 1;
        cloud_map_.is_dense = true;

        pcl::toROSMsg(cloud_map_, globalMap_pcd);
        globalMap_pcd.header.frame_id = "world";
        globalMap_pcd.header.stamp = ros::Time::now();

        map_ok_ = true;

        std::cout << "\033[32m [map generation]: Simulated random map generated! \033[0m" << std::endl;
    }

    void randomMapGenerator::mapPubTimerCallback(const ros::TimerEvent &event)
    {
        if (!map_ok_)
            return;

        globalMap_pcd.header.stamp = ros::Time::now();
        all_map_pub_.publish(globalMap_pcd);
    }

}