#ifndef DATA_SET_GENERATOR_HPP
#define DATA_SET_GENERATOR_HPP

#include "data_set_gen/PrimitiveTraj.h"
#include "data_set_gen/map_gen.h"
#include "data_set_gen/init_conditions.h"
#include "data_set_gen/primitive_library.hpp"
#include "data_set_gen/point_cloud_sensing.h"
#include "data_set_gen/plan_vis.h"
#include "data_set_gen/qp_primitive_planner.hpp"
#include "data_set_gen/data_set_storage.hpp"

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <ros/ros.h>
#include <random>
#include <thread>
#include <vector>
#include <atomic>
#include <mutex>

namespace dataSet_gen
{

    class DataSetGenerator
    {
    public:
        DataSetGenerator(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);
        void run(ros::NodeHandle &nh);

    private:
        void processPlannerParallel(int sim_map_idx);
        void processPlannerOrdinary(int sim_map_idx);
        double calculate_yaw(double t, const PrimitiveTrajectory &prim, const Eigen::Matrix3d &RWV, const Eigen::Vector3d &start_pos);

        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;
        randomMapGenerator map_generator_;
        DataSetStorage storage_;
        std::random_device rd_;
        std::default_random_engine eng_{rd_()};
        std::uniform_real_distribution<double> rand_obs_num_;
        std::uniform_real_distribution<double> rand_vel_mag_;
        std::uniform_real_distribution<double> rand_acc_;

        ros::Publisher PrimitiveTraj_pub_, imu_acc_pub_, odom_speed_pub_;
        ros::Subscriber odom_sub_, imu_sub_, trigger_replan_sub_;

        Eigen::Vector3d curr_pos_worldFrame_, curr_vel_worldFrame_, curr_acc_velFrame_;
        Eigen::Quaterniond curr_orien_;

        Eigen::Vector3d odom_pos_, odom_vel_;
        Eigen::Quaterniond odom_q_;
        Eigen::Vector3d imu_acc_;

        bool have_odom_ = false;
        bool replan_success_;
        bool flag_arrive_;

        PrimitiveLibrary primLib_;
        PointCloudSensor PcdSensor_;
        QPPrimitivePlanner planner_;
        ConditionInitializer init_conditions_;
        InitState init_state_;
        PlanVisualizer PlanVis_;

        int max_map_iterations_simMap_;
        int max_scenarios_per_simMap_;
        int max_initPose_iterations_;
        int max_valid_data_;
        int max_threads_;
        double obs_num_min_;
        double obs_num_max_;
        double vel_max_;
        double acc_max_;
        bool enable_parallel_;
        double replan_thresh_, no_replan_thresh_;

        std::normal_distribution<double> gaussian_noise_pos_;
        std::normal_distribution<double> gaussian_noise_vel_;
        std::normal_distribution<double> gaussian_noise_acc_;
        std::normal_distribution<double> gaussian_noise_yaw_;
        double noise_std_pos_;
        double noise_std_vel_;
        double noise_std_acc_;
        double noise_std_yaw_;

        std::vector<DataRecord> temp_records_;
        Eigen::Vector3d dir_target_from_drone_;

        std::atomic<int> total_attempt_data_{0};
        std::atomic<int> valid_data_{0};
        std::atomic<int> total_scenario_num_{0};
        std::atomic<bool> done_{false};
    };

}

#endif
