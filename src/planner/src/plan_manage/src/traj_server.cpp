#include "plan_manage/prim_traj.hpp"
#include "plan_manage/PrimitiveTraj.h"
#include "quadrotor_msgs/PositionCommand.h"

#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include "ros/ros.h"
#include <algorithm>

ros::Publisher pos_cmd_pub;

quadrotor_msgs::PositionCommand cmd;

bool receive_prim_ = false;
ros::Time start_time_;
npe2e_planner::PrimitiveTrajectory prim_;
Eigen::Matrix3d RWV_;
Eigen::Vector3d start_pos_;

bool stop_flag_ = false;
Eigen::Vector3d stop_pos;
bool have_odom_ = false;

double last_yaw_, last_yaw_dot_;
double time_forward_;

Eigen::Vector3d odom_pos_;
Eigen::Quaterniond odom_q_;

double aligned_yaw_;
bool need_yaw_alignment_ = false;

double max_acc_clamp_;

void primitiveTrajCallback(plan_manage::PrimitiveTrajConstPtr msg)
{
    prim_.clear();

    start_time_ = msg->start_time;
    prim_.duration = msg->duration;
    start_pos_ << msg->start_pos[0], msg->start_pos[1], msg->start_pos[2];

    if (prim_.duration == 0)
    {
        return;
    }

    size_t rows = msg->piece_sizes[0];
    size_t cols = msg->piece_sizes[1];

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> coeff_mat(rows, cols);
    std::memcpy(coeff_mat.data(), msg->coeffs.data(), rows * cols * sizeof(double));

    prim_.coeffs = coeff_mat;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            RWV_(i, j) = msg->rot_mat[i * 3 + j];
        }
    }

    receive_prim_ = true;
}

void stopCommandCallback(const std_msgs::Float64MultiArrayPtr msg)
{
    stop_pos << msg->data[0], msg->data[1], msg->data[2];
    stop_flag_ = true;
    receive_prim_ = false;
}

std::pair<double, double> calculate_yaw_for_init_alignment(double des_yaw, double dt)
{
    constexpr double YAW_DOT_MAX_PER_SEC = 0.5 * M_PI;
    constexpr double YAW_DOT_DOT_MAX_PER_SEC = 3 * M_PI;
    std::pair<double, double> yaw_yawdot(0, 0);

    double yaw_temp = des_yaw;
    Eigen::Matrix3d M = odom_q_.matrix();
    if (std::abs(atan2(M(1, 0), M(0, 0)) - yaw_temp) < 0.1)
    {
        need_yaw_alignment_ = false;
        return std::make_pair(last_yaw_, 0);
    }

    double yawdot = 0;
    double d_yaw = yaw_temp - last_yaw_;
    if (d_yaw >= M_PI)
    {
        d_yaw -= 2 * M_PI;
    }
    if (d_yaw <= -M_PI)
    {
        d_yaw += 2 * M_PI;
    }

    const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
    const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
    double d_yaw_max;
    if (fabs(last_yaw_dot_ + dt * YDDM) <= fabs(YDM))
    {

        d_yaw_max = last_yaw_dot_ * dt + 0.5 * YDDM * dt * dt;
    }
    else
    {

        double t1 = (YDM - last_yaw_dot_) / YDDM;
        d_yaw_max = ((dt - t1) + dt) * (YDM - last_yaw_dot_) / 2.0;
    }

    if (fabs(d_yaw) > fabs(d_yaw_max))
    {
        d_yaw = d_yaw_max;
    }
    yawdot = d_yaw / dt;

    double yaw = last_yaw_ + d_yaw;
    if (yaw > M_PI)
        yaw -= 2 * M_PI;
    if (yaw < -M_PI)
        yaw += 2 * M_PI;
    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    last_yaw_ = yaw_yawdot.first;
    last_yaw_dot_ = yaw_yawdot.second;

    yaw_yawdot.second = yaw_temp;

    return yaw_yawdot;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
    constexpr double PI = 3.1415926;
    constexpr double YAW_DOT_MAX_PER_SEC = PI;

    std::pair<double, double> yaw_yawdot(0, 0);
    double yaw = 0;
    double yawdot = 0;

    double t_safe = std::min(t_cur + time_forward_, prim_.getDuration());
    Eigen::Vector3d dir = (start_pos_ + RWV_ * prim_.getPos(t_safe)) - pos;

    double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
    double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
    if (yaw_temp - last_yaw_ > PI)
    {
        if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
        {
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI)
                yaw += 2 * PI;

            yawdot = -YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }
    else if (yaw_temp - last_yaw_ < -PI)
    {
        if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
        {
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI)
                yaw -= 2 * PI;

            yawdot = YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }
    else
    {
        if (yaw_temp - last_yaw_ < -max_yaw_change)
        {
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI)
                yaw += 2 * PI;

            yawdot = -YAW_DOT_MAX_PER_SEC;
        }
        else if (yaw_temp - last_yaw_ > max_yaw_change)
        {
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI)
                yaw -= 2 * PI;

            yawdot = YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }

    if (fabs(yaw - last_yaw_) <= max_yaw_change)
    {
        yaw = 0.5 * last_yaw_ + 0.5 * yaw;
    }
    yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;

    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    return yaw_yawdot;
}

void publish_cmd(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double y, double yd)
{

    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

    cmd.position.x = p(0);
    cmd.position.y = p(1);
    cmd.position.z = p(2);
    cmd.velocity.x = v(0);
    cmd.velocity.y = v(1);
    cmd.velocity.z = v(2);
    cmd.acceleration.x = a(0);
    cmd.acceleration.y = a(1);
    cmd.acceleration.z = a(2);
    cmd.yaw = y;
    cmd.yaw_dot = yd;
    pos_cmd_pub.publish(cmd);
}

void cmdCallback(const ros::TimerEvent &)
{

    if (!have_odom_)
    {
        ROS_WARN_THROTTLE(1.0, "[traj_server] waiting for odom");
        return;
    }

    if (need_yaw_alignment_)
    {

        std::pair<double, double> yaw_yawdot(0, 0);

        yaw_yawdot = calculate_yaw_for_init_alignment(aligned_yaw_, 0.01);

        publish_cmd(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), yaw_yawdot.first, yaw_yawdot.second);
        last_yaw_ = yaw_yawdot.first;
        return;
    }

    if (receive_prim_ && prim_.getDuration() >= 0)
    {
        ros::Time time_now = ros::Time::now();
        double t_cur = (time_now - start_time_).toSec();

        Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero());
        std::pair<double, double> yaw_yawdot(0, 0);

        static ros::Time time_last = ros::Time::now();
        if (t_cur < prim_.getDuration() && t_cur >= 0.0)
        {
            pos = start_pos_ + RWV_ * prim_.getPos(t_cur);
            vel = RWV_ * prim_.getVel(t_cur);
            acc = RWV_ * prim_.getAcc(t_cur);

            yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
        }
        else if (t_cur >= prim_.getDuration())
        {

            pos = start_pos_ + RWV_ * prim_.getPos(prim_.getDuration());
            vel.setZero();
            acc.setZero();

            yaw_yawdot.first = last_yaw_;
            yaw_yawdot.second = 0;
        }
        else
        {
            std::cout << "[Traj server]: invalid time." << std::endl;
        }
        time_last = time_now;

        acc(0) = std::clamp(acc(0), -max_acc_clamp_, max_acc_clamp_);
        acc(1) = std::clamp(acc(1), -max_acc_clamp_, max_acc_clamp_);
        acc(2) = std::clamp(acc(2), -max_acc_clamp_, max_acc_clamp_);

        cmd.header.stamp = time_now;
        cmd.header.frame_id = "world";
        cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

        cmd.position.x = pos(0);
        cmd.position.y = pos(1);
        cmd.position.z = pos(2);

        cmd.velocity.x = vel(0);
        cmd.velocity.y = vel(1);
        cmd.velocity.z = vel(2);

        cmd.acceleration.x = acc(0);
        cmd.acceleration.y = acc(1);
        cmd.acceleration.z = acc(2);

        cmd.yaw = yaw_yawdot.first;
        cmd.yaw_dot = yaw_yawdot.second;

        last_yaw_ = cmd.yaw;

        pos_cmd_pub.publish(cmd);
    }
    else if (stop_flag_)
    {
        ros::Time time_now = ros::Time::now();

        Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero());
        std::pair<double, double> yaw_yawdot(0, 0);
        pos = stop_pos;
        vel.setZero();
        acc.setZero();

        yaw_yawdot.first = last_yaw_;
        yaw_yawdot.second = 0;

        cmd.header.stamp = time_now;
        cmd.header.frame_id = "world";
        cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

        cmd.position.x = pos(0);
        cmd.position.y = pos(1);
        cmd.position.z = pos(2);

        cmd.velocity.x = vel(0);
        cmd.velocity.y = vel(1);
        cmd.velocity.z = vel(2);

        cmd.acceleration.x = acc(0);
        cmd.acceleration.y = acc(1);
        cmd.acceleration.z = acc(2);

        cmd.yaw = yaw_yawdot.first;
        cmd.yaw_dot = yaw_yawdot.second;

        last_yaw_ = cmd.yaw;

        pos_cmd_pub.publish(cmd);
    }
    else
    {
        return;
    }
}

void odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_q_.w() = msg->pose.pose.orientation.w;
    odom_q_.x() = msg->pose.pose.orientation.x;
    odom_q_.y() = msg->pose.pose.orientation.y;
    odom_q_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
}

void yawAlignmentCallback(const std_msgs::Float64ConstPtr &msg)
{
    aligned_yaw_ = msg->data;
    need_yaw_alignment_ = true;
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "traj_server");
    ros::NodeHandle node;
    ros::NodeHandle nh_priv("~");

    ros::Subscriber primitiveTraj_pub = node.subscribe("/planning/primitiveTraj", 10, primitiveTrajCallback);
    ros::Subscriber stop_sub = node.subscribe("/planning/stop_command", 1, stopCommandCallback);

    ros::Subscriber odom_sub = node.subscribe("/odom_world", 10, &odometryCallback);
    ros::Subscriber yaw_sub = node.subscribe("/npe2e_replan_fsm/aligned_yaw", 10, &yawAlignmentCallback);

    pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

    ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

    for (int i = 0; i < 3; ++i)
    {
        cmd.kx[i] = 0.0;
        cmd.kv[i] = 0.0;
    }

    nh_priv.param("traj_server/time_forward", time_forward_, 0.1);
    nh_priv.param("traj_server/max_acc", max_acc_clamp_, 10.0);
    last_yaw_ = 0.0;
    last_yaw_dot_ = 0.0;

    ros::Duration(1.0).sleep();

    ROS_WARN("[Traj server]: ready.");

    ros::spin();

    return 0;
}
