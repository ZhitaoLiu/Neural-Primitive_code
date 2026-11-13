#include "plan_manage/plan_manager.h"
#include <torch/cuda.h>
#include <cstdlib>

namespace npe2e_planner
{

    void npe2ePlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
    {
        if (!torch::cuda::is_available())
        {
            ROS_FATAL("CUDA is required, but no usable CUDA device was found.");
            std::exit(EXIT_FAILURE);
        }
        nh.param("manager/depthCloudStackNum", depthCloudStackNum_, 1);
        nh.param("manager/sampleSize", sampleSize_, 666);
        nh.param("manager/pcd_range_limit", pcd_range_limit_, 6.0);
        nh.param("manager/pcd_vertical_deg", pcd_vertical_deg_, 60.0);
        nh.param("manager/pcd_horizontal_deg", pcd_horizontal_deg_, 90.0);
        nh.param("manager/fixed_traj_duration", fixed_traj_duration_, 2.0);

        std::string model_name;
        nh.param<std::string>("manager/model_name", model_name, "policy_model.pt");

        modelInf_latency_pub_ = nh.advertise<std_msgs::Float64>("/plan_manage/modelInf_latency", 10);
        planner_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/plan_manage/planner_cloud", 10);

        dep_odom_sub_ = nh.subscribe<nav_msgs::Odometry>("plan_manage/odom", 10, &npe2ePlannerManager::odomCallback, this);
        dep_cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("plan_manage/cloud", 10, &npe2ePlannerManager::cloudCallback, this);

        depthCloudCount_ = 0;
        depthCloudStack_.resize(depthCloudStackNum_);
        plannerCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

        has_odom_ = false;
        has_cloud_ = false;

        grid_map_.reset(new GridMap);
        grid_map_->initMap(nh);

        initNNModels(model_name);

        visualization_ = vis;
    }

    void npe2ePlannerManager::initNNModels(const std::string &model_name)
    {
        int warmup_count = 20;

        const std::string model_path = ros::package::getPath("plan_manage") + "/nnModels/" + model_name;

        ROS_INFO("Loading NN model: %s", model_path.c_str());

        try
        {
            model_ = torch::jit::load(model_path, device_);
            model_ = torch::jit::optimize_for_inference(model_);
            model_.eval();
            model_loaded_ = true;
            std::cout << "NN model successfully loaded on device: " << (device_.is_cuda() ? "CUDA" : "CPU") << std::endl;
        }
        catch (const c10::Error &e)
        {
            ROS_ERROR("Error loading model: %s", e.what());
            model_loaded_ = false;
            return;
        }

        try
        {
            torch::Tensor state_tensor = torch::randn({1, 7}, torch::dtype(torch::kFloat32)).to(device_);
            torch::Tensor cloud_tensor = torch::randn({1, sampleSize_, 3}, torch::dtype(torch::kFloat32)).to(device_);
            torch::Tensor vel_tensor = torch::randn({1, 1}, torch::dtype(torch::kFloat32)).to(device_);
            torch::Tensor acc_tensor = torch::randn({1, 3}, torch::dtype(torch::kFloat32)).to(device_);

            for (int i = 0; i < warmup_count; ++i)
            {
                {
                    torch::NoGradGuard no_grad;
                    std::vector<torch::jit::IValue> inputs = {state_tensor, cloud_tensor, vel_tensor, acc_tensor};
                    model_.forward(inputs);
                }
            }
            std::cout << "NN model warm-up completed." << std::endl;
        }
        catch (const c10::Error &e)
        {
            ROS_WARN("NN model warm-up failed: %s", e.what());
        }
    }

    void npe2ePlannerManager::odomCallback(const nav_msgs::OdometryConstPtr &odom)
    {
        odom_pos_(0) = odom->pose.pose.position.x;
        odom_pos_(1) = odom->pose.pose.position.y;
        odom_pos_(2) = odom->pose.pose.position.z;

        odom_vel_(0) = odom->twist.twist.linear.x;
        odom_vel_(1) = odom->twist.twist.linear.y;
        odom_vel_(2) = odom->twist.twist.linear.z;

        odom_q_.w() = odom->pose.pose.orientation.w;
        odom_q_.x() = odom->pose.pose.orientation.x;
        odom_q_.y() = odom->pose.pose.orientation.y;
        odom_q_.z() = odom->pose.pose.orientation.z;

        has_odom_ = true;
    }

    void npe2ePlannerManager::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr latestCloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*img, *latestCloud);

        depthCloudStack_[depthCloudCount_] = latestCloud;
        depthCloudCount_ = (depthCloudCount_ + 1) % depthCloudStackNum_;

        has_cloud_ = true;
    }

    bool npe2ePlannerManager::trajReplan(const Eigen::Vector3d &start_pt,
                                         const Eigen::Vector3d &start_vel,
                                         const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &local_target_pt,
                                         const Eigen::Matrix3d &RWV)
    {
        if (!model_loaded_)
        {
            ROS_WARN("Model not loaded, cannot perform inference!");
            return false;
        }

        Eigen::Matrix3d RVW = RWV.inverse();

        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>());
        const double horizontal_tan = std::tan(pcd_horizontal_deg_ * M_PI / 360.0);
        const double vertical_tan = std::tan(pcd_vertical_deg_ * M_PI / 360.0);

        if (has_cloud_)
        {
            for (const auto &cloud : depthCloudStack_)
            {
                if (!cloud)
                    continue;

                for (const auto &pt : cloud->points)
                {
                    const Eigen::Vector3d pt_vec(pt.x - start_pt.x(),
                                                 pt.y - start_pt.y(),
                                                 pt.z - start_pt.z());
                    const double distance = pt_vec.norm();
                    if (distance > pcd_range_limit_ || distance < 1e-6)
                        continue;

                    const Eigen::Vector3d pt_vel_frame = RVW * pt_vec;
                    if (pt_vel_frame.x() <= 0.0)
                        continue;
                    if (std::abs(pt_vel_frame.y()) > pt_vel_frame.x() * horizontal_tan)
                        continue;
                    if (std::abs(pt_vel_frame.z()) > pt_vel_frame.x() * vertical_tan)
                        continue;

                    filteredCloud->points.push_back(pt);
                }
            }
        }

        filteredCloud->width = filteredCloud->points.size();
        filteredCloud->height = 1;
        filteredCloud->is_dense = true;

        sensor_msgs::PointCloud2 out_msg;
        pcl::toROSMsg(*filteredCloud, out_msg);
        out_msg.header.frame_id = "world";
        out_msg.header.stamp = ros::Time::now();
        planner_cloud_pub_.publish(out_msg);

        plannerCloud_->clear();
        if (filteredCloud->points.size() > static_cast<size_t>(sampleSize_))
        {
            rs_.setInputCloud(filteredCloud);
            rs_.setSample(sampleSize_);
            rs_.filter(*plannerCloud_);
        }
        else
        {
            *plannerCloud_ = *filteredCloud;
        }

        Eigen::Vector3d dir_target = RVW * (local_target_pt - start_pt);
        Eigen::Vector3d dir_target_from_drone;
        if (dir_target.norm() > 1e-6)
        {
            dir_target_from_drone = dir_target / (dir_target.norm());
        }
        else
        {
            dir_target_from_drone = RWV.col(0);
        }

        Eigen::VectorXf state_features(7);
        state_features << start_vel.norm(),
            (RVW * start_acc).cast<float>(),
            dir_target_from_drone.cast<float>();
        torch::Tensor state_tensor = torch::from_blob(state_features.data(), {1, 7}, torch::kFloat32).to(device_);

        std::vector<float> cloud_data(sampleSize_ * 3, 100.0f);
        size_t num_pcd_points = std::min(plannerCloud_->points.size(), size_t(sampleSize_));
        for (size_t i = 0; i < num_pcd_points; ++i)
        {
            auto &point = plannerCloud_->points[i];

            Eigen::Vector3d point_worldFrame(point.x, point.y, point.z);
            Eigen::Vector3d point_velFrame = RVW * (point_worldFrame - start_pt);
            cloud_data[i * 3] = static_cast<float>(point_velFrame.x());
            cloud_data[i * 3 + 1] = static_cast<float>(point_velFrame.y());
            cloud_data[i * 3 + 2] = static_cast<float>(point_velFrame.z());
        }
        torch::Tensor cloud_tensor = torch::from_blob(cloud_data.data(), {1, sampleSize_, 3}, torch::kFloat32).to(device_);

        float vel_magnitude = static_cast<float>(start_vel.norm());
        Eigen::Vector3d acc_vel_frame = RVW * start_acc;
        float acc_data[3] = {
            static_cast<float>(acc_vel_frame.x()),
            static_cast<float>(acc_vel_frame.y()),
            static_cast<float>(acc_vel_frame.z())};
        torch::Tensor vel_tensor = torch::from_blob(&vel_magnitude, {1, 1}, torch::kFloat32).to(device_);
        torch::Tensor acc_tensor = torch::from_blob(acc_data, {1, 3}, torch::kFloat32).to(device_);

        if (device_.is_cuda())
        {
            torch::cuda::synchronize();
        }

        ros::Time t1 = ros::Time::now();
        torch::jit::IValue output;
        {
            torch::NoGradGuard no_grad;
            std::vector<torch::jit::IValue> inputs = {state_tensor, cloud_tensor, vel_tensor, acc_tensor};
            output = model_.forward(inputs);
        }

        if (device_.is_cuda())
        {
            torch::cuda::synchronize();
        }

        ros::Time t2 = ros::Time::now();

        auto output_tuple = output.toTuple();
        auto coeffs_non_fixed_out = output_tuple->elements()[0].toTensor().to(torch::kCPU).squeeze(0).contiguous();

        Eigen::MatrixXd coeffs = Eigen::MatrixXd::Zero(3, 6);
        coeffs.leftCols(3) = Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(
                                 coeffs_non_fixed_out.data_ptr<float>(), 3, 3)
                                 .cast<double>();
        coeffs.col(3) = Eigen::Vector3d(
            static_cast<double>(acc_data[0]) / 2.0,
            static_cast<double>(acc_data[1]) / 2.0,
            static_cast<double>(acc_data[2]) / 2.0);
        coeffs.col(4) = Eigen::Vector3d(static_cast<double>(vel_magnitude), 0.0, 0.0);
        local_data_.bestPrim_ = PrimitiveTrajectory(fixed_traj_duration_, coeffs);
        local_data_.start_time_ = ros::Time::now();
        local_data_.start_pos_ = start_pt;

        std::cout << "Model inference time: \033[44m" << (t2 - t1).toSec() * 1000 << "\033[0m (ms)" << std::endl;

        std_msgs::Float64 modelInf_time;
        modelInf_time.data = (t2 - t1).toSec() * 1000;
        modelInf_latency_pub_.publish(modelInf_time);

        std::vector<Eigen::Vector3d> point_set;
        double T_alloc = local_data_.bestPrim_.getDuration();
        double step_t = std::min(0.1, T_alloc / 10);
        for (double t = 0; t <= T_alloc; t += step_t)
        {
            Eigen::Vector3d pt = local_data_.start_pos_ + RWV * local_data_.bestPrim_.getPos(t);
            point_set.push_back(pt);
        }
        visualization_->displayInitPathList(point_set, 0.1, 0);

        return true;
    }

    Eigen::Matrix3d npe2ePlannerManager::computeRWV(const Eigen::Vector3d &vel, const Eigen::Quaterniond &odom_q)
    {
        Eigen::Vector3d xV = vel.norm() < 0.2
                                 ? odom_q.toRotationMatrix().col(0)
                                 : vel.normalized();
        xV.normalize();

        Eigen::Vector3d yV = xV.cross(Eigen::Vector3d(0.0, 0.0, -1.0));
        if (yV.norm() < 1e-6)
            yV = xV.cross(Eigen::Vector3d::UnitX());
        yV.normalize();
        Eigen::Vector3d zV = xV.cross(yV);

        Eigen::Matrix3d RWV;
        RWV.col(0) = xV;
        RWV.col(1) = yV;
        RWV.col(2) = zV;

        return RWV;
    }

}
