#include "data_set_gen/data_set_generator.hpp"

namespace dataSet_gen
{

    DataSetGenerator::DataSetGenerator(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
        : nh_(nh),
          nh_priv_(nh_priv),
          map_generator_(nh, nh_priv),
          storage_(nh, nh_priv, 10000),
          PcdSensor_(nh, nh_priv),
          planner_(nh, nh_priv, primLib_),
          init_conditions_(map_generator_, nh_priv),
          PlanVis_(nh)
    {
        nh_priv_.getParam("max_map_iterations_simMap", max_map_iterations_simMap_);
        nh_priv_.getParam("max_scenarios_per_simMap", max_scenarios_per_simMap_);
        nh_priv_.getParam("obs_num_min", obs_num_min_);
        nh_priv_.getParam("obs_num_max", obs_num_max_);
        nh_priv_.getParam("vel_max", vel_max_);
        nh_priv_.getParam("acc_max", acc_max_);
        nh_priv_.getParam("max_valid_data", max_valid_data_);
        const unsigned int detected_hardware_threads = std::thread::hardware_concurrency();
        const int hardware_threads = detected_hardware_threads > 0
                                         ? static_cast<int>(detected_hardware_threads)
                                         : 1;
        nh_priv_.param("max_threads", max_threads_, hardware_threads);
        const int configured_max_threads = max_threads_;
        if (max_threads_ < 1)
            max_threads_ = 1;
        else if (max_threads_ > hardware_threads)
            max_threads_ = hardware_threads;
        nh_priv_.getParam("enable_parallel", enable_parallel_);
        int effective_enabled_threads = enable_parallel_ ? max_threads_ : 1;
        if (enable_parallel_ && max_scenarios_per_simMap_ > 0 &&
            effective_enabled_threads > max_scenarios_per_simMap_)
            effective_enabled_threads = max_scenarios_per_simMap_;
        nh_priv_.getParam("thresh_no_replan", no_replan_thresh_);
        nh_priv_.getParam("replan_thresh", replan_thresh_);

        nh_priv_.getParam("noise_std_pos", noise_std_pos_);
        nh_priv_.getParam("noise_std_vel", noise_std_vel_);
        nh_priv_.getParam("noise_std_acc", noise_std_acc_);
        nh_priv_.getParam("noise_std_yaw", noise_std_yaw_);

        gaussian_noise_pos_ = std::normal_distribution<double>(0.0, noise_std_pos_);
        gaussian_noise_vel_ = std::normal_distribution<double>(0.0, noise_std_vel_);
        gaussian_noise_acc_ = std::normal_distribution<double>(0.0, noise_std_acc_);
        gaussian_noise_yaw_ = std::normal_distribution<double>(0.0, noise_std_yaw_);

        rand_obs_num_ = std::uniform_real_distribution<double>(obs_num_min_, obs_num_max_);
        rand_vel_mag_ = std::uniform_real_distribution<double>(0.0, vel_max_);
        rand_acc_ = std::uniform_real_distribution<double>(-acc_max_, acc_max_);

        PrimitiveTraj_pub_ = nh.advertise<data_set_gen::PrimitiveTraj>("/planning/primitiveTraj", 10);

        imu_acc_pub_ = nh.advertise<std_msgs::Float64>("/data_set_gen/imu_acc_pub", 10);
        odom_speed_pub_ = nh.advertise<std_msgs::Float64>("/data_set_gen/odom_speed_pub", 10);

        temp_records_.reserve(1000);

        std::cout << "\033[34m=============== System info for parallel computation ===============\033[0m" << std::endl;
        std::cout << "\033[32m [Dataset generator]: System Info:\033[0m" << std::endl;
        std::cout << " Configured Maximum Threads: " << configured_max_threads << std::endl;
        std::cout << " Hardware Supported Maximum Threads: " << hardware_threads << std::endl;
        std::cout << " Effectively Enabled Threads: " << effective_enabled_threads << std::endl;

        ros::Duration(2.0).sleep();
    }

    void DataSetGenerator::run(ros::NodeHandle &nh)
    {
        for (int i = 0; i < max_map_iterations_simMap_ && ros::ok() && !done_; ++i)
        {
            std::cout << "\033[34m=============== Plan on SimMap " << i << " ===============\033[0m" << std::endl;
            int obs_num = static_cast<int>(std::round(rand_obs_num_(eng_)));
            map_generator_.generateRandomSimulatedMap(obs_num);
            enable_parallel_ ? processPlannerParallel(i) : processPlannerOrdinary(i);
        }
    }

    void DataSetGenerator::processPlannerParallel(int sim_map_idx)
    {
        std::vector<std::thread> threads;
        std::mutex cout_mutex;
        std::mutex storage_mutex;
        std::atomic<bool> done_local{false};

        auto worker = [&, sim_map_idx](int scenario_idx)
        {
            total_scenario_num_++;

            std::random_device rd;
            std::mt19937 eng(rd());

            PrimitiveLibrary primLib;
            PointCloudSensor PcdSensor(nh_, nh_priv_);
            QPPrimitivePlanner planner(nh_, nh_priv_, primLib);
            ConditionInitializer init_conditions(map_generator_, nh_priv_);
            PlanVisualizer PlanVis(nh_);
            std::vector<DataRecord> temp_records;
            temp_records.reserve(1000);

            {
                std::lock_guard<std::mutex> lock(cout_mutex);
                std::cout << "\033[34m---------------------- Episode " << scenario_idx << " in SimMap " << sim_map_idx << " started ----------------------\033[0m" << std::endl;
            }

            PcdSensor.recevGlobalMap(map_generator_.cloud_map_);

            double iniVel_magnitude = rand_vel_mag_(eng);
            Eigen::Vector3d iniVel_velFrame(iniVel_magnitude, 0.0, 0.0);
            Eigen::Vector3d iniAcc_velFrame = {rand_acc_(eng), rand_acc_(eng), rand_acc_(eng)};

            InitState init_state = init_conditions.randomInitStateGen(iniVel_velFrame, iniAcc_velFrame);

            Eigen::Vector3d curr_pos_worldFrame = init_state.iniPos_worldFrame;
            Eigen::Vector3d curr_vel_worldFrame = init_state.iniVel_worldFrame;
            Eigen::Vector3d curr_acc_velFrame = init_state.iniAcc_velFrame;
            Eigen::Quaterniond curr_orien = init_state.iniQuaterniond;
            Eigen::Vector3d dir_target_from_drone = init_state.dir_target_from_drone;

            bool have_odom = true;
            planner.has_odom_ = true;

            bool replan_success = true;
            bool flag_arrive = false;
            while (ros::ok() && replan_success && !flag_arrive && !done_local && !done_)
            {

                total_attempt_data_++;

                double curr_vel_norm = curr_vel_worldFrame.norm();
                std::vector<double> curr_acc_velFrame_vec = {curr_acc_velFrame(0), curr_acc_velFrame(1), curr_acc_velFrame(2)};
                primLib.generateLibrary(nh_priv_, curr_vel_norm, curr_acc_velFrame_vec);

                PcdSensor.sensingPointCloud(curr_pos_worldFrame, curr_vel_worldFrame, curr_orien);

                replan_success = planner.initPlanModules(
                    curr_pos_worldFrame,
                    curr_vel_worldFrame,
                    curr_orien,
                    PcdSensor.output_cloud,
                    init_state.target_pos_worldFrame);

                if (curr_pos_worldFrame(2) <= 0)
                {
                    replan_success = false;
                }

                if (replan_success)
                {

                    DataRecord record;
                    record.record_id = valid_data_;
                    record.pos_worldFrame = curr_pos_worldFrame;
                    record.vel_magnitude = curr_vel_norm;
                    record.acc_velFrame = curr_acc_velFrame;
                    record.target_pos_worldFrame = init_state.target_pos_worldFrame;
                    record.dir_target_from_drone = dir_target_from_drone;
                    record.processed_sensedCloud_velFrame = planner.processed_sensedCloud_velFrame_;
                    record.quaterniond = curr_orien;
                    record.duration = planner.getLocalData().bestPrim_->getDuration();
                    record.coeffs = planner.getLocalData().bestPrim_->getCoeffs();

                    temp_records.push_back(record);

                    auto info = planner.getLocalData();
                    curr_pos_worldFrame = info.start_pos_ + planner.getRWV() * info.bestPrim_->getPos(replan_thresh_);
                    curr_vel_worldFrame = planner.getRWV() * info.bestPrim_->getVel(replan_thresh_);
                    curr_acc_velFrame = info.bestPrim_->getAcc(replan_thresh_);
                    double yaw = calculate_yaw(replan_thresh_, *(info.bestPrim_), planner.getRWV(), info.start_pos_);

                    Eigen::Vector3d dir_worldFrame = init_state.target_pos_worldFrame - curr_pos_worldFrame;
                    Eigen::Vector3d dir_velFrame = planner.getRWV().inverse() * dir_worldFrame;
                    double dir_norm = dir_velFrame.norm();
                    if (dir_norm > 1e-6)
                    {
                        dir_target_from_drone = dir_velFrame / dir_norm;
                    }
                    else
                    {
                        dir_target_from_drone = curr_orien * Eigen::Vector3d::UnitX();
                    }

                    curr_pos_worldFrame(0) += gaussian_noise_pos_(eng);
                    curr_pos_worldFrame(1) += gaussian_noise_pos_(eng);
                    curr_pos_worldFrame(2) += gaussian_noise_pos_(eng);

                    curr_vel_worldFrame(0) += gaussian_noise_vel_(eng);
                    curr_vel_worldFrame(1) += gaussian_noise_vel_(eng);
                    curr_vel_worldFrame(2) += gaussian_noise_vel_(eng);

                    curr_acc_velFrame(0) += gaussian_noise_acc_(eng);
                    curr_acc_velFrame(1) += gaussian_noise_acc_(eng);
                    curr_acc_velFrame(2) += gaussian_noise_acc_(eng);

                    yaw += gaussian_noise_yaw_(eng);

                    curr_orien = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

                    double dist_relative = (curr_pos_worldFrame - init_state.target_pos_worldFrame).norm();
                    if (dist_relative <= no_replan_thresh_)
                    {
                        flag_arrive = true;
                        {
                            std::lock_guard<std::mutex> lock(cout_mutex);
                            std::cout << "-------------------------------" << std::endl;
                            std::cout << "\033[32m Episode " << scenario_idx << " in SimMap " << sim_map_idx << " reached target.\033[0m" << std::endl;
                        }
                    }
                }
                else
                {

                    temp_records.clear();
                    return;
                }
            }

            if (flag_arrive)
            {
                std::lock_guard<std::mutex> lock(storage_mutex);
                for (auto &record : temp_records)
                {
                    record.record_id = valid_data_++;
                    storage_.addRecord(record);
                }
                {
                    std::lock_guard<std::mutex> lock(cout_mutex);
                    std::cout << "\033[32m [Save Dataset]: Episode " << scenario_idx << " in SimMap " << sim_map_idx << " saved data successfully.\033[0m" << std::endl;
                    std::cout << "\033[32m [Save Dataset]: Valid data number: " << valid_data_ << " \033[0m" << std::endl;
                }
            }

            if (valid_data_ >= max_valid_data_)
            {
                done_ = true;
                done_local = true;
            }
        };

        int active_threads = 0;
        for (int j = 0; j < max_scenarios_per_simMap_ && ros::ok() && !done_; ++j)
        {
            threads.emplace_back(worker, j);
            active_threads++;

            while (active_threads >= max_threads_ && !threads.empty())
            {
                threads.front().join();
                threads.erase(threads.begin());
                active_threads--;
            }
        }

        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
    }

    void DataSetGenerator::processPlannerOrdinary(int sim_map_idx)
    {
        for (int j = 0; j < max_scenarios_per_simMap_ && ros::ok() && !done_; ++j)
        {

            total_scenario_num_++;

            temp_records_.clear();
            std::cout << "\033[34m---------------------- Episode " << j << " in SimMap " << sim_map_idx << " started ----------------------\033[0m" << std::endl;

            PcdSensor_.recevGlobalMap(map_generator_.cloud_map_);

            double iniVel_magnitude = rand_vel_mag_(eng_);
            Eigen::Vector3d iniVel_velFrame(iniVel_magnitude, 0.0, 0.0);
            Eigen::Vector3d iniAcc_velFrame = {rand_acc_(eng_), rand_acc_(eng_), rand_acc_(eng_)};

            init_state_ = init_conditions_.randomInitStateGen(iniVel_velFrame, iniAcc_velFrame);

            curr_pos_worldFrame_ = init_state_.iniPos_worldFrame;
            curr_vel_worldFrame_ = init_state_.iniVel_worldFrame;
            curr_acc_velFrame_ = init_state_.iniAcc_velFrame;
            curr_orien_ = init_state_.iniQuaterniond;
            dir_target_from_drone_ = init_state_.dir_target_from_drone;

            have_odom_ = true;
            planner_.has_odom_ = true;

            replan_success_ = true;
            flag_arrive_ = false;
            while (ros::ok() && replan_success_ && !flag_arrive_)
            {

                total_attempt_data_++;

                double curr_vel_norm = curr_vel_worldFrame_.norm();
                std::vector<double> curr_acc_velFrame = {curr_acc_velFrame_(0), curr_acc_velFrame_(1), curr_acc_velFrame_(2)};
                primLib_.generateLibrary(nh_priv_, curr_vel_norm, curr_acc_velFrame);

                PcdSensor_.sensingPointCloud(curr_pos_worldFrame_, curr_vel_worldFrame_, curr_orien_);

                replan_success_ = planner_.initPlanModules(
                    curr_pos_worldFrame_,
                    curr_vel_worldFrame_,
                    curr_orien_,
                    PcdSensor_.output_cloud,
                    init_state_.target_pos_worldFrame);

                if (curr_pos_worldFrame_(2) <= 0)
                {
                    replan_success_ = false;
                }

                if (replan_success_)
                {

                    DataRecord record;
                    record.record_id = valid_data_;
                    record.pos_worldFrame = curr_pos_worldFrame_;
                    record.vel_magnitude = curr_vel_norm;
                    record.acc_velFrame = curr_acc_velFrame_;
                    record.target_pos_worldFrame = init_state_.target_pos_worldFrame;
                    record.dir_target_from_drone = dir_target_from_drone_;
                    record.processed_sensedCloud_velFrame = planner_.processed_sensedCloud_velFrame_;
                    record.quaterniond = curr_orien_;
                    record.duration = planner_.getLocalData().bestPrim_->getDuration();
                    record.coeffs = planner_.getLocalData().bestPrim_->getCoeffs();

                    temp_records_.push_back(record);

                    {
                        PlanVis_.visualSensedCloud(PcdSensor_.output_cloud);
                        PlanVis_.visualTargetPoint(init_state_.target_pos_worldFrame);
                        PlanVis_.visualLib(primLib_, curr_pos_worldFrame_, planner_.getRWV());
                        PlanVis_.visualDronePose(curr_pos_worldFrame_, curr_orien_);
                        PlanVis_.visualPlannedTraj(planner_.getLocalData().bestPrim_, curr_pos_worldFrame_, planner_.getRWV());
                        ros::spinOnce();
                    }

                    auto info = planner_.getLocalData();
                    curr_pos_worldFrame_ = info.start_pos_ + planner_.getRWV() * info.bestPrim_->getPos(replan_thresh_);
                    curr_vel_worldFrame_ = planner_.getRWV() * info.bestPrim_->getVel(replan_thresh_);
                    curr_acc_velFrame_ = info.bestPrim_->getAcc(replan_thresh_);
                    double yaw = calculate_yaw(replan_thresh_, *(info.bestPrim_), planner_.getRWV(), info.start_pos_);

                    Eigen::Vector3d dir_worldFrame = init_state_.target_pos_worldFrame - curr_pos_worldFrame_;
                    Eigen::Vector3d dir_velFrame = planner_.getRWV().inverse() * dir_worldFrame;
                    double dir_norm = dir_velFrame.norm();
                    if (dir_norm > 1e-6)
                    {
                        dir_target_from_drone_ = dir_velFrame / dir_norm;
                    }
                    else
                    {
                        dir_target_from_drone_ = curr_orien_ * Eigen::Vector3d::UnitX();
                    }

                    curr_pos_worldFrame_(0) += gaussian_noise_pos_(eng_);
                    curr_pos_worldFrame_(1) += gaussian_noise_pos_(eng_);
                    curr_pos_worldFrame_(2) += gaussian_noise_pos_(eng_);

                    curr_vel_worldFrame_(0) += gaussian_noise_vel_(eng_);
                    curr_vel_worldFrame_(1) += gaussian_noise_vel_(eng_);
                    curr_vel_worldFrame_(2) += gaussian_noise_vel_(eng_);

                    curr_acc_velFrame_(0) += gaussian_noise_acc_(eng_);
                    curr_acc_velFrame_(1) += gaussian_noise_acc_(eng_);
                    curr_acc_velFrame_(2) += gaussian_noise_acc_(eng_);

                    yaw += gaussian_noise_yaw_(eng_);

                    curr_orien_ = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
                }
                else
                {
                    continue;
                }

                double dist_relative = (curr_pos_worldFrame_ - init_state_.target_pos_worldFrame).norm();

                if (dist_relative <= no_replan_thresh_)
                {
                    flag_arrive_ = true;
                    std::cout << "-------------------------------" << std::endl;
                    std::cout << "\033[32m Episode " << j << " in SimMap " << sim_map_idx << " reached target.\033[0m" << std::endl;
                }
            }

            if (flag_arrive_)
            {
                for (auto &record : temp_records_)
                {
                    record.record_id = valid_data_++;
                    storage_.addRecord(record);
                }
                std::cout << "\033[32m [Save Dataset]: Episode " << j << " in SimMap " << sim_map_idx << " saved data successfully.\033[0m" << std::endl;
                std::cout << "\033[32m [Save Dataset]: Valid data number: " << valid_data_ << " \033[0m" << std::endl;
            }

            if (valid_data_ >= max_valid_data_)
            {
                done_ = true;
                break;
            }
        }
    }

    double DataSetGenerator::calculate_yaw(double t, const PrimitiveTrajectory &prim, const Eigen::Matrix3d &RWV, const Eigen::Vector3d &start_pos)
    {
        constexpr double PI = 3.1415926;

        Eigen::Vector3d vel = prim.getVel(t);

        Eigen::Vector3d vel_world = RWV * vel;

        double yaw = vel_world.norm() > 0.001 ? atan2(vel_world(1), vel_world(0)) : 0.0;

        if (yaw > PI)
            yaw -= 2 * PI;
        if (yaw < -PI)
            yaw += 2 * PI;

        return yaw;
    }
}
