#ifndef MAP_GEN_H
#define MAP_GEN_H

#include <ros/ros.h>
#include <random>
#include <sensor_msgs/PointCloud2.h>

class randomMapGenerator
{
public:
    randomMapGenerator(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);

    void generateRandomSimulatedMap(int obs_num);

private:
    void mapPubTimerCallback(const ros::TimerEvent &);

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
    double init_x, init_y;
    double x_size_, y_size_;
    double x_l_, x_h_, y_l_, y_h_;
    double w_l_, w_h_, h_l_, h_h_;
    double resolution_;
    double min_dist_;
    double publish_rate_;
    double radius_l_, radius_h_, z_l_, z_h_, theta_;
    sensor_msgs::PointCloud2 globalMap_pcd;
};

#endif