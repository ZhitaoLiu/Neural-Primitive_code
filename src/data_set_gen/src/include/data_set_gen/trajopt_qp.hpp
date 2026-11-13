#ifndef TRAJOPT_QP_HPP
#define TRAJOPT_QP_HPP

#include "qp_solver.hpp"
#include "trajectory.hpp"

#include "ros/ros.h"
#include "Eigen/Eigen"

class Trajopt_qp
{
private:
    Qp_solver qp_solver;

    Trajectory<5> jerk_traj;
    Trajectory<7> snap_traj;

public:
    int optOrder;
    bool successFlag = false;
    double trajStamp;

public:
    Trajopt_qp(ros::NodeHandle &nh_priv)
        : qp_solver(QPConfig(nh_priv))
    {
        optOrder = nh_priv.param("OptOrder", 3);
        qp_solver.setOrder(optOrder);

    }

    bool optCal(std::vector<Eigen::Vector3d> path, std::vector<double> iniVel, std::vector<double> iniAcc, std::vector<double> endVel, std::vector<double> endAcc, double avgSpeed)
    {

        auto t1 = std::chrono::high_resolution_clock::now();

        if (path.size() > 1)
        {
            Eigen::MatrixXd iniPVA(3, 3), finPVA(3, 3);

            Eigen::Map<Eigen::Vector3d> startVel(iniVel.data());
            Eigen::Map<Eigen::Vector3d> startAcc(iniAcc.data());
            Eigen::Map<Eigen::Vector3d> terminalVel(endVel.data());
            Eigen::Map<Eigen::Vector3d> terminalAcc(endAcc.data());

            iniPVA.col(0) = path.front();
            iniPVA.col(1) = startVel;
            iniPVA.col(2) = startAcc;

            finPVA.col(0) = path.back();
            finPVA.col(1) = terminalVel;
            finPVA.col(2) = terminalAcc;

            Eigen::VectorXd times(path.size() - 1);
            times = timeAllocate(path, avgSpeed);

            Eigen::VectorXd polyCoffs;
            if (!qp_solver.solve(iniPVA, finPVA, path, times, polyCoffs))
            {
                return false;
            }

            size_t idx;
            if (optOrder == 3)
            {
                jerk_traj.clear();
                jerk_traj.reserve(times.size());
                Eigen::Matrix<double, 3, 6> coffMat;
                for (size_t i = 0; i < times.size(); i++)
                {
                    for (size_t j = 0; j < 3; j++)
                    {
                        idx = i * 3 * 6 + j * 6;
                        coffMat.row(j) = polyCoffs.segment(idx, 6);
                    }
                    jerk_traj.emplace_back(times(i), coffMat);
                }
            }
            else
            {
                snap_traj.clear();
                snap_traj.reserve(times.size());
                Eigen::Matrix<double, 3, 8> coffMat;
                for (size_t i = 0; i < times.size(); i++)
                {
                    for (size_t j = 0; j < 3; j++)
                    {
                        idx = i * 3 * 8 + j * 8;
                        coffMat.row(j) = polyCoffs.segment(idx, 8);
                    }
                    snap_traj.emplace_back(times(i), coffMat);
                }
            }

            auto t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration_ms = t2 - t1;

            trajStamp = ros::Time::now().toSec();

            return true;
        }
        else
        {
            std::cout << "\033[31m [trajOptQP - path input]: path points should be at least two! \033[0m" << std::endl;
            return false;
        }
    }

    void getTraj(Trajectory<5> &traj)
    {
        traj = jerk_traj;
    }

    void getTraj(Trajectory<7> &traj)
    {
        traj = snap_traj;
    }

    Eigen::Vector3d getPos(double t) const
    {
        return (optOrder == 4) ? snap_traj.getPos(t) : jerk_traj.getPos(t);
    }

    Eigen::Vector3d getVel(double t) const
    {
        return (optOrder == 4) ? snap_traj.getVel(t) : jerk_traj.getVel(t);
    }

    Eigen::Vector3d getAcc(double t) const
    {
        return (optOrder == 4) ? snap_traj.getAcc(t) : jerk_traj.getAcc(t);
    }

    Eigen::Vector3d getJer(double t) const
    {
        return (optOrder == 4) ? snap_traj.getJer(t) : jerk_traj.getJer(t);
    }

    size_t getPieceNum() const
    {
        return (optOrder == 4) ? snap_traj.getPieceNum() : jerk_traj.getPieceNum();
    }

    Eigen::VectorXd getDurations() const
    {
        return (optOrder == 4) ? snap_traj.getDurations() : jerk_traj.getDurations();
    }

    double getTotalDuration() const
    {
        return (optOrder == 4) ? snap_traj.getTotalDuration() : jerk_traj.getTotalDuration();
    }

    Eigen::MatrixXd getCoeffs(size_t i) const
    {
        if (i >= getPieceNum())
        {
            throw std::out_of_range("[trajOptQP]: Trajectory segment index out of bounds");
        }
        if (optOrder == 4)
        {
            return Eigen::MatrixXd(snap_traj[i].getCoeffMat());
        }
        else
        {
            return Eigen::MatrixXd(jerk_traj[i].getCoeffMat());
        }
    }

    void setTrajectory(double duration, const Eigen::MatrixXd &coeffMat)
    {
        if (optOrder == 3)
        {
            if (coeffMat.rows() != 3 || coeffMat.cols() != 6)
            {
                throw std::invalid_argument("[Trajopt_qp::setTrajectory] Expected 3x6 matrix for optOrder 3");
            }
            jerk_traj.clear();
            jerk_traj.emplace_back(duration, coeffMat);
        }
        else if (optOrder == 4)
        {
            if (coeffMat.rows() != 3 || coeffMat.cols() != 8)
            {
                throw std::invalid_argument("[Trajopt_qp::setTrajectory] Expected 3x8 matrix for optOrder 4");
            }
            snap_traj.clear();
            snap_traj.emplace_back(duration, coeffMat);
        }
        else
        {
            throw std::invalid_argument("[Trajopt_qp::setTrajectory] Invalid optOrder, must be 3 or 4");
        }
        successFlag = true;
    }

private:
    Eigen::VectorXd timeAllocate(const std::vector<Eigen::Vector3d> &path, double avg_speed)
    {
        Eigen::VectorXd times(path.size() - 1);
        double safe_avgSpeed = (avg_speed <= 0) ? 1e-06 : avg_speed;
        for (size_t i = 0; i < times.size(); i++)
        {
            double dist = (path[i + 1] - path[i]).norm();
            times[i] = dist / safe_avgSpeed;
        }
        return times;
    }
};

#endif