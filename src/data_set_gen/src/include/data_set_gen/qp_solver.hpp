
#ifndef QP_SOLVER_HPP
#define QP_SOLVER_HPP

#include "OsqpEigen/OsqpEigen.h"

#include "ros/ros.h"

struct QPConfig
{
    double MaxVelBox, MaxAccBox;
    int ConstRes;

    double Tolerance;

    QPConfig(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("MaxVelBox", MaxVelBox);
        nh_priv.getParam("MaxAccBox", MaxAccBox);
        nh_priv.getParam("ConstRes", ConstRes);

        nh_priv.getParam("Tolerance", Tolerance);
    }
};

class Qp_solver
{
private:
    QPConfig config;
    Eigen::Vector2d dyn_limits;
    size_t dim_ = 3;
    size_t state_dim_ = 3;
    size_t res_;
    size_t order_;
    size_t d_;

    size_t seg_;
    Eigen::MatrixXd zero_A_;
    Eigen::MatrixXd A_, Q_, G_;
    Eigen::VectorXd h_, b_;

    double obj_cost_ = -1;

public:
    Qp_solver(const QPConfig &conf)
        : config(conf)
    {
        res_ = config.ConstRes;
        dyn_limits << config.MaxVelBox, config.MaxAccBox;
    }

    inline void setOrder(int &order)
    {
        order_ = order;
        d_ = 2 * order_;

        zero_A_.resize(order_, d_);
        zero_A_.setZero();

        if (order_ == 4)
        {
            zero_A_(0, 7) = 1.0;
            zero_A_(1, 6) = 1.0;
            zero_A_(2, 5) = 2.0;
            zero_A_(3, 4) = 6.0;
        }
        else
        {
            zero_A_(0, 5) = 1.0;
            zero_A_(1, 4) = 1.0;
            zero_A_(2, 3) = 2.0;
        }
        return;
    }

    inline double getObjCost()
    {
        return obj_cost_;
    }

    template <typename T>
    inline Eigen::MatrixXd get_t_state(const T &t)
    {
        T t_2 = t * t;
        T t_3 = t * t_2;
        T t_4 = t_2 * t_2;
        T t_5 = t_2 * t_3;

        Eigen::MatrixXd conti_A(order_, d_);

        if (order_ == 4)
        {
            T t_6 = t_3 * t_3;
            T t_7 = t_4 * t_3;

            conti_A << t_7, t_6, t_5, t_4, t_3, t_2, t, 1,
                7 * t_6, 6 * t_5, 5 * t_4, 4 * t_3, 3 * t_2, 2 * t, 1, 0,
                42 * t_5, 30 * t_4, 20 * t_3, 12 * t_2, 6 * t, 2, 0, 0,
                210 * t_4, 120 * t_3, 60 * t_2, 24 * t, 6, 0, 0, 0;
        }
        else
        {
            conti_A << t_5, t_4, t_3, t_2, t, 1,
                5 * t_4, 4 * t_3, 3 * t_2, 2 * t, 1, 0,
                20 * t_3, 12 * t_2, 6 * t, 2, 0, 0;
        }
        return conti_A;
    }

    template <typename T>
    inline bool solve(const Eigen::MatrixXd &iniPVA,
                      const Eigen::MatrixXd &finPVA,
                      const std::vector<Eigen::Vector3d> &path,
                      const T &times,
                      Eigen::VectorXd &qp_solution)
    {
        seg_ = times.size();

        size_t var_num = seg_ * dim_ * d_;

        size_t eq_num = (2 * state_dim_ + (seg_ - 1) * order_) * dim_ + 2 * (seg_ - 1) * dim_;

        A_.resize(eq_num, var_num);
        A_.setZero();
        b_.resize(eq_num);
        b_.setZero();

        size_t row = 0;
        size_t s_num = (seg_ - 1) * dim_ * d_;
        size_t idx, col_idx;

        for (size_t i = 0; i < dim_; i++)
        {
            idx = i * d_;

            A_.block(row, idx, state_dim_, d_) = zero_A_.block(0, 0, state_dim_, d_);
            b_.segment(row, state_dim_) = (iniPVA.row(i)).transpose();
            row += state_dim_;

            A_.block(row, s_num + idx, state_dim_, d_) = get_t_state(times(seg_ - 1)).block(0, 0, state_dim_, d_);
            b_.segment(row, state_dim_) = (finPVA.row(i)).transpose();
            row += state_dim_;
        }

        for (size_t i = 0; i < seg_ - 1; i++)
        {
            idx = i * dim_ * d_;
            for (size_t j = 0; j < dim_; j++)
            {
                col_idx = idx + j * d_;
                size_t next_col_idx = col_idx + dim_ * d_;

                A_.block(row, col_idx, order_, d_) = get_t_state(times(i));
                A_.block(row, next_col_idx, order_, d_) = -zero_A_;

                row += order_;
            }
        }

        for (size_t i = 0; i < seg_ - 1; i++)
        {
            idx = i * dim_ * d_;
            for (size_t j = 0; j < dim_; j++)
            {
                col_idx = idx + j * d_;
                size_t next_col_idx = col_idx + dim_ * d_;

                A_.block(row, col_idx, 1, d_) = get_t_state(times(i)).row(0);
                A_.block(row + 1, next_col_idx, 1, d_) = zero_A_.row(0);

                b_[row] = path[i + 1][j];
                b_[row + 1] = path[i + 1][j];

                row += 2;
            }
        }

        Q_.resize(var_num, var_num);
        Q_.setZero();
        Eigen::MatrixXd Cost_Q;
        float t, t_2, t_3, t_4, t_5, t_6, t_7;
        float m_11, m_12, m_13, m_14, m_22, m_23, m_24, m_33, m_34;

        for (size_t i = 0; i < seg_; i++)
        {
            idx = i * dim_ * d_;

            t = times(i);
            t_2 = t * t;
            t_3 = t * t_2;
            t_4 = t_2 * t_2;
            t_5 = t_2 * t_3;

            if (order_ == 4)
            {
                t_6 = t_3 * t_3;
                t_7 = t_4 * t_3;

                m_11 = 100800 * t_7;
                m_12 = 50400 * t_6;
                m_13 = 20160 * t_5;
                m_14 = 5040 * t_4;

                m_22 = 25920 * t_5;
                m_23 = 10800 * t_4;
                m_24 = 2880 * t_3;

                m_33 = 4800 * t_3;
                m_34 = 1400 * t_2;

                Cost_Q.resize(order_, order_);

                Cost_Q << m_11, m_12, m_13, m_14,
                    m_12, m_22, m_23, m_24,
                    m_13, m_23, m_33, m_34,
                    m_14, m_24, m_34, 576 * t;
            }
            else
            {

                m_11 = 720 * t_5;
                m_12 = 360 * t_4;
                m_13 = 120 * t_3;

                m_22 = 192 * t_3;
                m_23 = 72 * t_2;

                Cost_Q.resize(order_, order_);

                Cost_Q << m_11, m_12, m_13,
                    m_12, m_22, m_23,
                    m_13, m_23, 36 * t;
            }

            for (size_t j = 0; j < dim_; j++)
            {
                col_idx = idx + j * d_;
                Q_.block(col_idx, col_idx, order_, order_) = Cost_Q;
            }
        }

        size_t ineq_num = seg_ * res_ * 4 * dim_;
        G_.resize(ineq_num, var_num);
        G_.setZero();
        h_.resize(ineq_num);
        h_.setZero();
        Eigen::MatrixXd tempG(order_, d_);
        float step_time, cur_time;

        row = 0;
        for (size_t i = 0; i < seg_; i++)
        {
            step_time = times(i) / float(res_);
            idx = i * dim_ * d_;

            for (size_t j = 0; j < res_; j++)
            {
                cur_time = j * step_time;
                tempG = (j == 0) ? zero_A_ : get_t_state(cur_time);

                for (size_t k = 0; k < dim_; k++)
                {
                    col_idx = idx + k * d_;
                    G_.block(row, col_idx, 2, d_) = tempG.middleRows(1, 2);
                    h_.segment(row, 2) = dyn_limits;
                    row += 2;

                    G_.block(row, col_idx, 2, d_) = -tempG.middleRows(1, 2);
                    h_.segment(row, 2) = dyn_limits;
                    row += 2;
                }
            }
        }

        OsqpEigen::Solver solver;

        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        solver.settings()->setAbsoluteTolerance(config.Tolerance);
        solver.settings()->setRelativeTolerance(config.Tolerance);

        solver.settings()->setWarmStart(false);

        int total_const_num = A_.rows() + G_.rows();

        Eigen::VectorXd p(var_num);
        p.setZero();

        Eigen::SparseMatrix<double> hessianMatrix = Q_.sparseView();

        Eigen::MatrixXd linearMatrix(total_const_num, var_num);
        linearMatrix << A_,
            G_;

        Eigen::SparseMatrix<double> linearSparseMatrix = linearMatrix.sparseView();

        Eigen::VectorXd lowerBound(total_const_num), upperBound(total_const_num);
        upperBound << b_,
            h_;

        Eigen::VectorXd inf_vec = -INFINITY * Eigen::VectorXd::Ones(h_.size());
        lowerBound << b_,
            inf_vec;

        solver.data()->setNumberOfVariables(var_num);
        solver.data()->setNumberOfConstraints(total_const_num);
        solver.data()->setHessianMatrix(hessianMatrix);
        solver.data()->setGradient(p);
        solver.data()->setLinearConstraintsMatrix(linearSparseMatrix);
        solver.data()->setLowerBound(lowerBound);
        solver.data()->setUpperBound(upperBound);
        solver.initSolver();

        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        {

            return false;
        }

        float result = solver.getObjValue();
        if (result > 5000 || result < -0.01)
        {

            return false;
        }

        auto status = solver.getStatus();
        if ((status != OsqpEigen::Status::Solved))
        {

            return false;
        }

        qp_solution = solver.getSolution();

        obj_cost_ = solver.getObjValue();
        return true;
    }
};

#endif