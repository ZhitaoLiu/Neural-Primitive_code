#ifndef INIT_CONDITIONS_H
#define INIT_CONDITIONS_H

#include "data_set_gen/map_gen.h"
#include <Eigen/Dense>
#include <random>

namespace dataSet_gen
{

    struct InitState
    {
        Eigen::Vector3d iniPos_worldFrame;
        Eigen::Quaterniond iniQuaterniond;
        Eigen::Vector3d iniVel_worldFrame;
        Eigen::Vector3d iniAcc_velFrame;
        Eigen::Vector3d target_pos_worldFrame;
        Eigen::Vector3d dir_target_from_drone;
    };

    class ConditionInitializer
    {
    public:
        ConditionInitializer(const randomMapGenerator &map_generator, ros::NodeHandle &nh_priv);

        InitState randomInitStateGen(Eigen::Vector3d iniVel_body, Eigen::Vector3d iniAcc_velFrame);

    private:
        const randomMapGenerator &map_generator_;
        std::default_random_engine eng_{std::random_device{}()};
        std::uniform_real_distribution<double> rand_x_;
        std::uniform_real_distribution<double> rand_y_;
        std::uniform_real_distribution<double> rand_z_;
        std::uniform_real_distribution<double> rand_yaw_;
        std::uniform_real_distribution<double> rand_vel_mag_;
        std::uniform_real_distribution<double> rand_acc_;
        double x_size_, y_size_, z_size_, resolution_;
        double x_l_, x_h_, y_l_, y_h_, z_l_, z_h_;
        double vel_magnitude_max_, acc_max_;
    };

}

#endif