#ifndef PRIMITIVE_LIBRARY_HPP
#define PRIMITIVE_LIBRARY_HPP

#include "trajopt_qp.hpp"

#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>

namespace dataSet_gen
{

    class PrimitiveTrajectory
    {
    public:
        int index;
        double duration;
        Eigen::MatrixXd coeffs;
        std::unordered_set<int> voxel_ids;

        PrimitiveTrajectory() : index(-1), duration(0.0) {}
        PrimitiveTrajectory(int idx, double dur, const Eigen::MatrixXd &mat, const std::unordered_set<int> &v_ids = {})
            : index(idx), duration(dur), coeffs(mat), voxel_ids(v_ids) {}

        void clear()
        {
            index = -1;
            duration = 0.0;
            coeffs.setZero();
            voxel_ids.clear();
        }

        int getIndex() const { return index; }

        double getDuration() const { return duration; }

        Eigen::MatrixXd getCoeffs() const { return coeffs; }

        const std::unordered_set<int> &getVoxelIds() const { return voxel_ids; }

        bool isVoxelInPrimitive(int voxel_id) const
        {
            return voxel_ids.count(voxel_id) > 0;
        }

        int getOptOrder() const { return coeffs.cols() / 2; }

        int getDegree() const { return coeffs.cols() - 1; }

        Eigen::Vector3d getPos(double t) const
        {
            Eigen::Vector3d pos = Eigen::Vector3d::Zero();
            double tn = 1.0;
            int D = getDegree();
            for (int i = D; i >= 0; i--)
            {
                pos += tn * coeffs.col(i);
                tn *= t;
            }
            return pos;
        }

        Eigen::Vector3d getVel(double t) const
        {
            Eigen::Vector3d vel = Eigen::Vector3d::Zero();
            double tn = 1.0;
            int n = 1;
            int D = getDegree();
            for (int i = D - 1; i >= 0; i--)
            {
                vel += n * tn * coeffs.col(i);
                tn *= t;
                n++;
            }
            return vel;
        }

        Eigen::Vector3d getAcc(double t) const
        {
            Eigen::Vector3d acc = Eigen::Vector3d::Zero();
            double tn = 1.0;
            int m = 1, n = 2;
            int D = getDegree();
            for (int i = D - 2; i >= 0; i--)
            {
                acc += m * n * tn * coeffs.col(i);
                tn *= t;
                m++;
                n++;
            }
            return acc;
        }

        Eigen::Vector3d getJer(double t) const
        {
            Eigen::Vector3d jer = Eigen::Vector3d::Zero();
            double tn = 1.0;
            int l = 1, m = 2, n = 3;
            int D = getDegree();
            for (int i = D - 3; i >= 0; i--)
            {
                jer += l * m * n * tn * coeffs.col(i);
                tn *= t;
                l++;
                m++;
                n++;
            }
            return jer;
        }

        double getPrimitiveTrajLen(int samples = 10) const
        {
            double length = 0.0;
            Eigen::Vector3d prev = getPos(0.0);
            for (int i = 1; i <= samples; i++)
            {
                double t = duration * i / samples;
                Eigen::Vector3d curr = getPos(t);
                length += (curr - prev).norm();
                prev = curr;
            }
            return length;
        }

        double getPrimitiveLineLen() const
        {
            return getPos(duration).norm();
        }
    };

    class PrimitiveLibrary
    {
    public:
        PrimitiveLibrary();

        const std::vector<PrimitiveTrajectory> &getLibrary() const;

        void generateLibrary(ros::NodeHandle &nh_priv, double iniVel_magnitude, std::vector<double> iniAcc);

    private:
        std::vector<PrimitiveTrajectory> primitive_lib_;

        void polarToCartesian(std::vector<double> &Vector);

        std::vector<Eigen::Vector3d> constructPath(double r, double theta, double psi);
    };

}

#endif