#include "data_set_gen/qp_primitive_planner.hpp"

namespace dataSet_gen
{

    QPPrimitivePlanner::QPPrimitivePlanner(ros::NodeHandle &nh, ros::NodeHandle &nh_priv, PrimitiveLibrary &primLib)
        : nh_(nh), nh_priv_(nh_priv), vis_(nh), primLib_(primLib)
    {
        nh_priv_.param("manager/depthCloudStackNum", depthCloudStackNum_, 1);
        nh_priv_.param("manager/boxX", boxX_, 6.0);
        nh_priv_.param("manager/boxY", boxY_, 6.0);
        nh_priv_.param("manager/boxZ", boxZ_, 6.0);
        nh_priv_.param("manager/voxelSize", voxelSize_, 0.1);
        nh_priv_.param("manager/sampleSize", sampleSize_, 1024);
        nh_priv_.param("manager/map_size_x", x_size_, 30.0);
        nh_priv_.param("manager/map_size_y", y_size_, 20.0);
        nh_priv_.param("manager/map_size_z", z_size_, 10.0);

        voxelNumX_ = static_cast<int>(boxX_ / voxelSize_);
        voxelNumY_ = static_cast<int>(boxY_ / voxelSize_);
        voxelNumZ_ = static_cast<int>(boxZ_ / voxelSize_);
        voxelNumAll_ = voxelNumX_ * voxelNumY_ * voxelNumZ_;
        voxelX_ = boxX_;
        voxelY_ = boxY_ / 2.0;
        voxelZ_ = boxZ_ / 2.0;

        depthCloudStack_.resize(depthCloudStackNum_);
        depthCloudCount_ = 0;
        has_cloud_ = false;
    }

    bool QPPrimitivePlanner::initPlanModules(const Eigen::Vector3d &pos_worldFrame,
                                             const Eigen::Vector3d &vel_worldFrame,
                                             const Eigen::Quaterniond &orientation,
                                             const sensor_msgs::PointCloud2 &cloud_msg,
                                             const Eigen::Vector3d &target_pos)
    {

        pos_worldFrame_ = pos_worldFrame;
        vel_worldFrame_ = vel_worldFrame;

        RWV_ = computeRWV(vel_worldFrame_, orientation);

        cloudInput(cloud_msg);
        if (trajReplan(pos_worldFrame_, target_pos, RWV_))
        {
            return true;
        }

        return false;
    }

    void QPPrimitivePlanner::cloudInput(const sensor_msgs::PointCloud2 &cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr latestCloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(cloud_msg, *latestCloud);

        if (latestCloud->points.empty())
        {

            has_cloud_ = true;

            return;
        }

        if (!has_odom_)
        {

            return;
        }

        depthCloudStack_[depthCloudCount_] = latestCloud;
        depthCloudCount_ = (depthCloudCount_ + 1) % depthCloudStackNum_;

        static int cloud_num = 0;
        cloud_num++;
        if (cloud_num >= depthCloudStackNum_)
        {
            cloud_num = 0;
            has_cloud_ = true;
        }
    }

    bool QPPrimitivePlanner::trajReplan(const Eigen::Vector3d &start_pt,
                                        const Eigen::Vector3d &target_pt,
                                        const Eigen::Matrix3d &RWV)
    {
        ros::Time t_0 = ros::Time::now();

        bool success_select = selectBestPrimitive(start_pt, target_pt, RWV);
        ros::Time t_1 = ros::Time::now();

        if (!success_select)
        {

            return false;
        }

        local_data_.start_time_ = ros::Time::now();
        local_data_.start_pos_ = start_pt;

        return true;
    }

    bool QPPrimitivePlanner::selectBestPrimitive(const Eigen::Vector3d &start_Pos,
                                                 const Eigen::Vector3d &targetPos,
                                                 const Eigen::Matrix3d &RWV)
    {
        const double w_pos = 10.0;
        const double w_len = 1.0;
        const int sampleRes = 10;

        Eigen::Matrix3d RVW = RWV.inverse();
        Eigen::Vector3d targetPos_VelFrame = RVW * (targetPos - start_Pos);
        double startToTarget = targetPos_VelFrame.norm();

        const auto &primitives = primLib_.getLibrary();
        std::vector<PrimitiveTrajectory *> primitives_ptrs;
        primitives_ptrs.reserve(primitives.size());

        for (auto &prim : primitives)
        {
            primitives_ptrs.push_back(const_cast<PrimitiveTrajectory *>(&prim));
        }

        if (!labelObsCollisionPrims(start_Pos, RWV, primitives_ptrs))
        {

            return false;
        }

        std::vector<PrimitiveTrajectory *> candidates_safe;
        for (auto *prim : primitives_ptrs)
        {
            if (clearPrimList_.count(prim->getIndex()) == 0)
            {
                candidates_safe.push_back(prim);
            }
        }

        if (candidates_safe.empty())
        {

            return false;
        }

        local_data_.bestPrim_ = nullptr;
        double minCost = std::numeric_limits<double>::infinity();

        for (auto *prim : candidates_safe)
        {
            Eigen::Vector3d primEndpoint = prim->getPos(prim->duration);
            double primLineLength = prim->getPrimitiveLineLen();
            double trajLength = prim->getPrimitiveTrajLen(sampleRes);
            double cost = 0.0;

            if (startToTarget > primLineLength)
            {
                double endpointErr = (primEndpoint - targetPos_VelFrame).norm();
                cost = w_pos * endpointErr + w_len * trajLength;
            }
            else
            {
                double minDist = std::numeric_limits<double>::infinity();
                for (int i = 0; i <= sampleRes; ++i)
                {
                    double t = prim->duration * i / sampleRes;
                    Eigen::Vector3d pt = prim->getPos(t);
                    double dist = (pt - targetPos_VelFrame).norm();
                    minDist = std::min(minDist, dist);
                }
                cost = w_pos * minDist + w_len * trajLength;
            }

            if (cost < minCost)
            {
                minCost = cost;
                local_data_.bestPrim_ = prim;
            }
        }

        if (local_data_.bestPrim_ == nullptr)
        {

            return false;
        }

        return true;
    }

    bool QPPrimitivePlanner::labelObsCollisionPrims(const Eigen::Vector3d &start_pt,
                                                    const Eigen::Matrix3d &RWV,
                                                    const std::vector<PrimitiveTrajectory *> &candidates)
    {
        ros::Time t0 = ros::Time::now();

        if (!has_cloud_)
        {

            return false;
        }

        clearPrimList_.clear();

        pcl::PointCloud<pcl::PointXYZ>::Ptr plannerCloudStack(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto &cloud : depthCloudStack_)
        {
            if (cloud)
            {
                *plannerCloudStack += *cloud;
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZ>());
        rs_.setInputCloud(plannerCloudStack);
        rs_.setSample(sampleSize_);
        rs_.filter(*plannerCloud);

        processed_sensedCloud_velFrame_.reset(new pcl::PointCloud<pcl::PointXYZ>());

        std::unordered_set<int> obstacle_voxel_ids;
        Eigen::Matrix3d RVW = RWV.inverse();
        for (const auto &pt : plannerCloud->points)
        {
            Eigen::Vector3d pos_w(pt.x - start_pt(0), pt.y - start_pt(1), pt.z - start_pt(2));
            Eigen::Vector3d pos_v = RVW * pos_w;

            if (pos_v(0) >= 1e-4 && pos_v(0) <= voxelX_ - 1e-4 &&
                pos_v(1) >= -voxelY_ + 1e-4 && pos_v(1) <= voxelY_ - 1e-4 &&
                pos_v(2) >= -voxelZ_ + 1e-4 && pos_v(2) <= voxelZ_ - 1e-4)
            {
                int voxelIDX = std::floor((voxelX_ - pos_v(0)) / voxelSize_);
                int voxelIDY = std::floor((voxelY_ - pos_v(1)) / voxelSize_);
                int voxelIDZ = std::floor((voxelZ_ - pos_v(2)) / voxelSize_);
                int voxelID = voxelNumY_ * voxelNumZ_ * voxelIDX + voxelNumZ_ * voxelIDY + voxelIDZ;
                obstacle_voxel_ids.insert(voxelID);
            }

            pcl::PointXYZ point;
            point.x = pos_v(0);
            point.y = pos_v(1);
            point.z = pos_v(2);
            processed_sensedCloud_velFrame_->points.push_back(point);
        }
        processed_sensedCloud_velFrame_->width = processed_sensedCloud_velFrame_->points.size();
        processed_sensedCloud_velFrame_->height = 1;
        processed_sensedCloud_velFrame_->is_dense = true;

        ros::Time t1 = ros::Time::now();

        for (const auto *prim : candidates)
        {
            for (int voxel_id : obstacle_voxel_ids)
            {
                if (prim->isVoxelInPrimitive(voxel_id))
                {
                    clearPrimList_.insert(prim->getIndex());
                    break;
                }
            }
        }

        ros::Time t2 = ros::Time::now();

        return true;
    }

    Eigen::Matrix3d QPPrimitivePlanner::computeRWV(const Eigen::Vector3d &vel_worldFrame,
                                                   const Eigen::Quaterniond &odom_q,
                                                   double xV_offset)
    {
        Eigen::Vector3d xV;
        Eigen::Vector3d odom_x_dir_ = odom_q.toRotationMatrix().col(0);

        if (vel_worldFrame.norm() < 0.2)
        {
            Eigen::AngleAxisd rotation_angle_axis(xV_offset, Eigen::Vector3d::UnitZ());
            odom_x_dir_ = rotation_angle_axis * odom_x_dir_;
            xV = odom_x_dir_.normalized();
        }
        else
        {
            xV = vel_worldFrame.normalized();
        }

        Eigen::Vector3d yV = xV.cross(Eigen::Vector3d(0, 0, -1)).normalized();
        Eigen::Vector3d zV = xV.cross(yV).normalized();

        Eigen::Matrix3d RWV;
        RWV.col(0) = xV;
        RWV.col(1) = yV;
        RWV.col(2) = zV;

        return RWV;
    }

    const Eigen::Matrix3d &QPPrimitivePlanner::getRWV() const
    {
        return RWV_;
    }

    const LocalData &QPPrimitivePlanner::getLocalData() const
    {
        return local_data_;
    }

    void QPPrimitivePlanner::publishTrajectory(ros::Publisher &traj_pub)
    {

        data_set_gen::PrimitiveTraj primitiveTraj;
        primitiveTraj.idx = local_data_.bestPrim_->getIndex();
        primitiveTraj.start_time = local_data_.start_time_;
        primitiveTraj.duration = local_data_.bestPrim_->getDuration();

        primitiveTraj.coeffs.clear();
        primitiveTraj.piece_sizes.clear();
        primitiveTraj.rot_mat.clear();

        const Eigen::MatrixXd &coff_mat = local_data_.bestPrim_->getCoeffs();

        primitiveTraj.piece_sizes.push_back(coff_mat.rows());
        primitiveTraj.piece_sizes.push_back(coff_mat.cols());

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> coeff_mat_row = coff_mat;
        primitiveTraj.coeffs.insert(primitiveTraj.coeffs.end(), coeff_mat_row.data(), coeff_mat_row.data() + coff_mat.size());

        primitiveTraj.rot_mat.resize(9);
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                primitiveTraj.rot_mat[i * 3 + j] = RWV_(i, j);
            }
        }

        primitiveTraj.start_pos.resize(3);
        primitiveTraj.start_pos[0] = local_data_.start_pos_(0);
        primitiveTraj.start_pos[1] = local_data_.start_pos_(1);
        primitiveTraj.start_pos[2] = local_data_.start_pos_(2);

        traj_pub.publish(primitiveTraj);
    }

}
