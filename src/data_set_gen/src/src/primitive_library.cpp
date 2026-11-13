#include "data_set_gen/primitive_library.hpp"
#include <stdexcept>

namespace dataSet_gen
{
    PrimitiveLibrary::PrimitiveLibrary() {}

    const std::vector<PrimitiveTrajectory> &PrimitiveLibrary::getLibrary() const
    {
        return primitive_lib_;
    }

    void PrimitiveLibrary::generateLibrary(ros::NodeHandle &nh_priv, double iniVel_magnitude, std::vector<double> iniAcc_velFrame)
    {
        ros::Time t_gen_start = ros::Time::now();

        primitive_lib_.clear();
        Trajopt_qp trajopt_qp(nh_priv);

        double r, thetaMin_pf, thetaMax_pf, psiMin_pf, psiMax_pf, step_theta_pf, step_psi_pf;
        double valueMin_v0, valueMax_v0, step_value_v0;
        double valueMin_a0, valueMax_a0, step_value_a0;
        double avgV;
        std::vector<double> endVel, endAcc;
        double box_x, box_y, box_z, drone_radius, voxelSize;

        nh_priv.getParam("r", r);
        nh_priv.getParam("thetaMin_pf", thetaMin_pf);
        nh_priv.getParam("thetaMax_pf", thetaMax_pf);
        nh_priv.getParam("psiMin_pf", psiMin_pf);
        nh_priv.getParam("psiMax_pf", psiMax_pf);
        nh_priv.getParam("step_theta_pf", step_theta_pf);
        nh_priv.getParam("step_psi_pf", step_psi_pf);

        nh_priv.getParam("valueMin_v0", valueMin_v0);
        nh_priv.getParam("valueMax_v0", valueMax_v0);
        nh_priv.getParam("step_value_v0", step_value_v0);

        nh_priv.getParam("valueMin_a0", valueMin_a0);
        nh_priv.getParam("valueMax_a0", valueMax_a0);
        nh_priv.getParam("step_value_a0", step_value_a0);

        nh_priv.getParam("avgV", avgV);
        if (!nh_priv.getParam("end_vel", endVel) || endVel.size() != 3)
            throw std::invalid_argument("end_vel must contain three numeric values");
        if (!nh_priv.getParam("end_acc", endAcc) || endAcc.size() != 3)
            throw std::invalid_argument("end_acc must contain three numeric values");

        nh_priv.getParam("box_x", box_x);
        nh_priv.getParam("box_y", box_y);
        nh_priv.getParam("box_z", box_z);
        nh_priv.getParam("drone_radius", drone_radius);
        nh_priv.getParam("voxelSize", voxelSize);

        std::vector<std::vector<Eigen::Vector3d>> pathSet;
        for (size_t i = 0; i <= std::floor((thetaMax_pf - thetaMin_pf) / step_theta_pf); i++)
        {
            for (size_t j = 0; j <= std::floor((psiMax_pf - psiMin_pf) / step_psi_pf); j++)
            {
                double theta = thetaMin_pf + i * step_theta_pf;
                double psi = psiMin_pf + j * step_psi_pf;
                pathSet.push_back(constructPath(r, theta, psi));
            }
        }

        std::vector<double> iniVel = {iniVel_magnitude, 0, 0};

        int voxelNumX = std::floor(box_x / voxelSize);
        int voxelNumY = std::floor(box_y / voxelSize);
        int voxelNumZ = std::floor(box_z / voxelSize);
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxelPoints(new pcl::PointCloud<pcl::PointXYZ>());

        for (int indX = 0; indX < voxelNumX; indX++)
        {
            double x = box_x - voxelSize * indX - 0.5 * voxelSize;
            for (int indY = 0; indY < voxelNumY; indY++)
            {
                double y = box_y / 2.0 - voxelSize * indY - 0.5 * voxelSize;
                for (int indZ = 0; indZ < voxelNumZ; indZ++)
                {
                    double z = box_z / 2.0 - voxelSize * indZ - 0.5 * voxelSize;
                    voxelPoints->points.emplace_back(x, y, z);
                }
            }
        }

        pcl::KdTreeFLANN<pcl::PointXYZ> voxel_kdtree;
        voxel_kdtree.setInputCloud(voxelPoints);
        double obs_inflate_radius = std::sqrt(3) * voxelSize / 2.0 + drone_radius;

        int counter_all = 0;
        int counter_success = 0;

        for (const auto &path : pathSet)
        {
            ros::Time t0 = ros::Time::now();
            counter_all++;

            trajopt_qp.successFlag = trajopt_qp.optCal(path, iniVel, iniAcc_velFrame, endVel, endAcc, avgV);

            if (trajopt_qp.successFlag)
            {

                double dur = trajopt_qp.getDurations()[0];
                Eigen::MatrixXd coeffMat = trajopt_qp.getCoeffs(0);

                std::unordered_set<int> voxel_ids;
                int num_samples = static_cast<int>(dur * 100);
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;

                for (int i = 0; i <= num_samples; i++)
                {
                    double t = (dur * i) / num_samples;
                    Eigen::Vector3d pos = trajopt_qp.getPos(t);
                    pcl::PointXYZ search_point(pos(0), pos(1), pos(2));

                    if (voxel_kdtree.radiusSearch(search_point, obs_inflate_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
                    {
                        for (size_t idx = 0; idx < pointIdxRadiusSearch.size(); ++idx)
                        {
                            voxel_ids.insert(pointIdxRadiusSearch[idx]);
                        }
                    }
                }

                PrimitiveTrajectory prim(counter_success, dur, coeffMat, voxel_ids);
                primitive_lib_.push_back(prim);
                counter_success++;
            }

            ros::Time t1 = ros::Time::now();
        }

        ros::Time t_gen_end = ros::Time::now();
    }

    void PrimitiveLibrary::polarToCartesian(std::vector<double> &Vector)
    {
        double value = Vector[0];
        double theta = Vector[1] * M_PI / 180.0;
        double psi = Vector[2] * M_PI / 180.0;

        double x = value * cos(theta) * cos(psi);
        double y = value * cos(theta) * sin(psi);
        double z = value * sin(theta);

        Vector[0] = x;
        Vector[1] = y;
        Vector[2] = z;
    }

    std::vector<Eigen::Vector3d> PrimitiveLibrary::constructPath(double r, double theta, double psi)
    {
        std::vector<Eigen::Vector3d> twoPointPath;
        std::vector<double> pathPoint0 = {0, 0, 0};
        std::vector<double> pathPoint1 = {r, theta, psi};

        polarToCartesian(pathPoint0);
        polarToCartesian(pathPoint1);

        twoPointPath.emplace_back(pathPoint0[0], pathPoint0[1], pathPoint0[2]);
        twoPointPath.emplace_back(pathPoint1[0], pathPoint1[1], pathPoint1[2]);

        return twoPointPath;
    }
}
