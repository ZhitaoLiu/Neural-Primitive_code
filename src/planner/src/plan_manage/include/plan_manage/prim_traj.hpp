#ifndef PRIM_TRAJ_HPP
#define PRIM_TRAJ_HPP

#include <Eigen/Eigen>

namespace npe2e_planner
{
    class PrimitiveTrajectory
    {
    public:
        float duration;
        Eigen::MatrixXd coeffs;

        PrimitiveTrajectory() : duration(0.0) {}
        PrimitiveTrajectory(float dur, const Eigen::MatrixXd &mat)
            : duration(dur), coeffs(mat) {}

        void clear()
        {
            duration = 0.0;
            coeffs.setZero();
        }

        double getDuration() const { return duration; }

        Eigen::MatrixXd getCoeffs() const { return coeffs; }

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

    };

}

#endif