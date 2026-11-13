#include "plan_manage/replan_fsm.h"

namespace npe2e_planner
{
    void npe2eReplanFSM::init(ros::NodeHandle &nh)
    {
        exec_state_ = FSM_EXEC_STATE::INIT;
        have_target_ = false;
        have_odom_ = false;
        trigger_ = false;

        if_yaw_aligned_ = false;
        turnHead_flag_ = false;
        yaw_aligned_stable_start_time_ = ros::Time::now();

        nh.param("fsm/flight_type", target_type_, -1);
        nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
        nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
        nh.param("fsm/target_z", target_z_, 1.0);

        if (!nh.getParam("/waypoints/x", waypoints_x_) ||
            !nh.getParam("/waypoints/y", waypoints_y_) ||
            !nh.getParam("/waypoints/z", waypoints_z_))
        {
            ROS_ERROR("[FSM]: Error: Failed to load waypoints from launch file!");
            return;
        }
        num_waypoints_ = waypoints_x_.size();
        if (num_waypoints_ != waypoints_y_.size() || num_waypoints_ != waypoints_z_.size() || num_waypoints_ == 0)
        {
            ROS_ERROR_STREAM("[FSM]: Error: Inconsistent or empty waypoints! x: " << waypoints_x_.size()
                                                                                  << ", y: " << waypoints_y_.size() << ", z: " << waypoints_z_.size());
            return;
        }

        visualization_.reset(new PlanningVisualization(nh));
        planner_manager_.reset(new npe2ePlannerManager);
        planner_manager_->initPlanModules(nh, visualization_);

        exec_timer_ = nh.createTimer(ros::Duration(0.01), &npe2eReplanFSM::execFSMCallback, this);

        odom_sub_ = nh.subscribe("/odom_world", 1, &npe2eReplanFSM::odometryCallback, this);
        imu_sub_ = nh.subscribe("/quadrotor_simulator_so3/imu", 1, &npe2eReplanFSM::imuCallback, this);

        PrimitiveTraj_pub_ = nh.advertise<plan_manage::PrimitiveTraj>("/planning/primitiveTraj", 10);
        stop_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/planning/stop_command", 10);

        imu_acc_pub_ = nh.advertise<std_msgs::Float64>("/npe2e_replan_fsm/imu_acc_pub", 10);
        odom_speed_pub_ = nh.advertise<std_msgs::Float64>("/npe2e_replan_fsm/odom_speed_pub", 10);

        yaw_aligned_pub_ = nh.advertise<std_msgs::Float64>("/npe2e_replan_fsm/aligned_yaw", 10);

        if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
        {
            waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &npe2eReplanFSM::waypointCallback, this);
        }
        else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
        {
            while (ros::ok() && !have_odom_)
            {
                ros::spinOnce();
            }

            planbyGivenWps();
        }
        else
        {
            std::cout << "Wrong target_type_ value! target_type_=" << target_type_ << std::endl;
        }
    }

    void npe2eReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
    {
        if (msg->poses[0].pose.position.z < -0.1)
        {
            return;
        }
        std::cout << "Triggered!" << std::endl;
        trigger_ = true;
        have_target_ = true;
        from_collision_emergency_ = false;

        end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, target_z_;
        resetIntermediateGuide();

        if (exec_state_ == WAIT_TARGET)
        {

            changeFSMExecState(WAIT_TARGET, "TRIG");
        }
        else if (exec_state_ == EXEC_TRAJ)
        {
            changeFSMExecState(REPLAN_TRAJ, "TRIG");
        }

        visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
    }

    void npe2eReplanFSM::planbyGivenWps()
    {

        std::cout << "Triggered!" << std::endl;
        trigger_ = true;
        have_target_ = true;
        from_collision_emergency_ = false;

        path_global_.clear();
        path_global_.push_back(odom_pos_);
        for (size_t i = 0; i < num_waypoints_; ++i)
        {
            Eigen::Vector3d wp(waypoints_x_[i], waypoints_y_[i], waypoints_z_[i]);
            path_global_.push_back(wp);
        }

        waypoint_num_ = path_global_.size();
        goal_id_ = 1;
        end_pt_ = path_global_[goal_id_];
        resetIntermediateGuide();

        for (size_t i = 0; i < (size_t)path_global_.size(); i++)
        {
            visualization_->displayGoalPoint(path_global_[i], Eigen::Vector4d(1, 0, 0, 1), 0.3, i);
            ros::Duration(0.01).sleep();
        }

        changeFSMExecState(WAIT_TARGET, "TRIG");
    }

    void npe2eReplanFSM::execFSMCallback(const ros::TimerEvent &)
    {
        static int fsm_num = 0;
        fsm_num++;
        if (fsm_num == 100)
        {
            printFSMExecState();
            if (!have_odom_)
                std::cout << "no odom." << std::endl;
            if (!trigger_)
                std::cout << "wait for goal." << std::endl;
            fsm_num = 0;
        }

        switch (exec_state_)
        {
        case INIT:
        {
            if (!have_odom_)
            {
                return;
            }
            if (!trigger_)
            {
                return;
            }
            changeFSMExecState(WAIT_TARGET, "FSM");
            break;
        }
        case WAIT_TARGET:
        {
            if (!have_target_)
            {
                return;
            }
            else if (!if_yaw_aligned_)
            {
                yawAlignmentPub();
                return;
            }
            else
            {

                if (!turnHead_flag_)
                {
                    yaw_aligned_stable_start_time_ = ros::Time::now();
                    turnHead_flag_ = true;
                }

                if (((ros::Time::now() - yaw_aligned_stable_start_time_).toSec() >= 1.0) && odom_vel_.norm() < 0.1)
                {
                    changeFSMExecState(GEN_NEW_TRAJ, "FSM");
                    turnHead_flag_ = false;
                    yaw_aligned_stable_start_time_ = ros::Time();
                }
            }
            break;
        }
        case GEN_NEW_TRAJ:
        {
            start_pt_ = odom_pos_;
            start_vel_ = odom_vel_;
            start_acc_.setZero();

            bool success = callTrajReplan();
            if (success)
            {
                changeFSMExecState(EXEC_TRAJ, "FSM");
                flag_escape_emergency_ = true;
            }
            else
            {

                if (exec_state_ != REPLAN_TRAJ && exec_state_ != EMERGENCY_STOP)
                    changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            }
            break;
        }
        case REPLAN_TRAJ:
        {

            start_pt_ = odom_pos_;
            start_vel_ = odom_vel_;
            start_acc_ = imu_acc_;

            bool success = callTrajReplan();
            if (success)
            {
                changeFSMExecState(EXEC_TRAJ, "FSM");
            }
            else
            {

                if (exec_state_ != EMERGENCY_STOP)
                    changeFSMExecState(REPLAN_TRAJ, "FSM");
            }

            break;
        }
        case EXEC_TRAJ:
        {

            LocalTrajData *info = &planner_manager_->local_data_;
            ros::Time time_now = ros::Time::now();
            double t_cur = (time_now - info->start_time_).toSec();
            double duration_ = info->bestPrim_.getDuration();
            t_cur = std::min(duration_, t_cur);
            Eigen::Vector3d pos = info->start_pos_ + RWV * info->bestPrim_.getPos(t_cur);

            if ((odom_pos_ - end_pt_).norm() < no_replan_thresh_)
            {
                if (goal_id_ == waypoint_num_ - 1)
                {
                    changeFSMExecState(APPROACH_GOAL, "FSM");
                }
                else
                {
                    goal_id_++;
                    end_pt_ = path_global_[goal_id_];
                    resetIntermediateGuide();
                    if (kEnableIntermediateGuide)
                        changeFSMExecState(REPLAN_TRAJ, "NEW GOAL");
                }
            }
            else if (kEnableIntermediateGuide && updateIntermediateGuide(odom_pos_, odom_vel_))
            {
                changeFSMExecState(REPLAN_TRAJ, "GUIDE");
            }
            else if ((info->start_pos_ - pos).norm() >= replan_thresh_)
            {
                changeFSMExecState(REPLAN_TRAJ, "FSM");
            }
            else if (t_cur >= duration_ * 2.0 / 3.0)
            {
                changeFSMExecState(REPLAN_TRAJ, "FSM");
            }

            break;
        }

        case APPROACH_GOAL:
        {
            callEmergencyStop(odom_pos_);

            if (odom_vel_.norm() < 0.1)
            {
                if_yaw_aligned_ = false;
                have_target_ = false;
                changeFSMExecState(WAIT_TARGET, "FSM");
            }
            return;
        }

        case EMERGENCY_STOP:
        {
            if (from_collision_emergency_)
            {

                callEmergencyStop(odom_pos_);
            }
            else
            {
                if (flag_escape_emergency_)
                {
                    callEmergencyStop(odom_pos_);
                }
                else
                {
                    if (odom_vel_.norm() < 0.1)
                    {
                        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
                    }
                }
                flag_escape_emergency_ = false;
            }
            break;
        }
        }
    }

    void npe2eReplanFSM::yawAlignmentPub()
    {
        Eigen::Vector2d dxy = end_pt_.block<2, 1>(0, 0) - odom_pos_.block<2, 1>(0, 0);
        Eigen::Matrix3d M = odom_q_.matrix();
        if (dxy.norm() > 0.1)
        {
            if (std::abs(atan2(dxy(1), dxy(0)) - atan2(M(1, 0), M(0, 0))) < 0.1)
            {
                if_yaw_aligned_ = true;
                return;
            }
            std_msgs::Float64 aligned_yaw_msg;
            aligned_yaw_msg.data = atan2(dxy(1), dxy(0));
            yaw_aligned_pub_.publish(aligned_yaw_msg);
        }
        else
        {
            if_yaw_aligned_ = true;
        }
    }

    void npe2eReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        odom_pos_(0) = msg->pose.pose.position.x;
        odom_pos_(1) = msg->pose.pose.position.y;
        odom_pos_(2) = msg->pose.pose.position.z;

        odom_vel_(0) = msg->twist.twist.linear.x;
        odom_vel_(1) = msg->twist.twist.linear.y;
        odom_vel_(2) = msg->twist.twist.linear.z;

        odom_q_.w() = msg->pose.pose.orientation.w;
        odom_q_.x() = msg->pose.pose.orientation.x;
        odom_q_.y() = msg->pose.pose.orientation.y;
        odom_q_.z() = msg->pose.pose.orientation.z;

        have_odom_ = true;

        std_msgs::Float64 speedMsg;
        speedMsg.data = odom_vel_.norm();
        odom_speed_pub_.publish(speedMsg);
    }

    void npe2eReplanFSM::imuCallback(const sensor_msgs::ImuConstPtr &msg)
    {
        imu_acc_(0) = msg->linear_acceleration.x;
        imu_acc_(1) = msg->linear_acceleration.y;
        imu_acc_(2) = msg->linear_acceleration.z;

        std_msgs::Float64 accMsg;
        accMsg.data = imu_acc_.norm();
        imu_acc_pub_.publish(accMsg);
    }

    void npe2eReplanFSM::resetIntermediateGuide()
    {
        guide_created_ = false;
        guide_active_ = false;
    }

    bool npe2eReplanFSM::updateIntermediateGuide(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity)
    {
        const Eigen::Vector2d horizontal_velocity = velocity.head<2>();
        const Eigen::Vector2d horizontal_goal = (end_pt_ - position).head<2>();
        const double speed = horizontal_velocity.norm();
        const double goal_distance = horizontal_goal.norm();
        if (speed < kMinGuideSpeed || goal_distance < 1e-6)
            return false;

        const Eigen::Vector2d direction = horizontal_velocity / speed;
        const double angle_cosine = direction.dot(horizontal_goal / goal_distance);
        if (guide_active_)
        {
            if (angle_cosine <= kGuideAngleCosine)
                return false;

            guide_active_ = false;
            return true;
        }

        if (guide_created_ || angle_cosine >= kGuideAngleCosine)
            return false;

        const double cross = direction.x() * horizontal_goal.y() - direction.y() * horizontal_goal.x();
        const double turn_side = cross >= 0.0 ? 1.0 : -1.0;
        const Eigen::Vector2d guide_direction(-turn_side * direction.y(), turn_side * direction.x());
        guide_pt_ = position;
        guide_pt_.head<2>() += guide_direction * (end_pt_ - position).norm();
        guide_created_ = true;
        guide_active_ = true;
        return true;
    }

    bool npe2eReplanFSM::callTrajReplan()
    {
        std::cout << "\033[32m------------ start new replan ------------\033[0m" << std::endl;
        if (kEnableIntermediateGuide)
            updateIntermediateGuide(start_pt_, start_vel_);
        RWV = planner_manager_->computeRWV(start_vel_, odom_q_);
        const Eigen::Vector3d &planning_target = kEnableIntermediateGuide && guide_active_ ? guide_pt_ : end_pt_;
        bool plan_success = planner_manager_->trajReplan(start_pt_, start_vel_, start_acc_, planning_target, RWV);

        if (plan_success)
        {
            if (!isTrajCollisionFree())
            {
                if (collision_replan_count_ < 100)
                {
                    if (exec_state_ != EMERGENCY_STOP)
                        changeFSMExecState(REPLAN_TRAJ, "COLLISION CHECK");
                    return false;
                }
            }
            collision_replan_count_ = 0;
            std::cout << "Receding plan succeed!" << std::endl;
            publishTrajectory();
        }

        return plan_success;
    }

    bool npe2eReplanFSM::isTrajCollisionFree()
    {
        auto info = &planner_manager_->local_data_;
        auto map = planner_manager_->grid_map_;

        constexpr double time_step = 0.05;
        double dur = info->bestPrim_.getDuration();
        double t_check = dur * 2.0 / 3.0;

        for (double t = 0.0; t <= t_check; t += time_step)
        {
            Eigen::Vector3d pos = info->start_pos_ + RWV * info->bestPrim_.getPos(t);
            if (map->getInflateOccupancy(pos))
            {
                collision_replan_count_++;
                return false;
            }
        }
        return true;
    }

    void npe2eReplanFSM::callEmergencyStop(const Eigen::Vector3d &stop_pos)
    {

        std_msgs::Float64MultiArray stop_msg;
        for (int i = 0; i < 3; i++)
        {
            stop_msg.data.push_back(stop_pos(i));
        }
        stop_pub_.publish(stop_msg);
    }

    void npe2eReplanFSM::printFSMExecState()
    {
        static std::string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

        std::cout << "[FSM]: state: " + state_str[int(exec_state_)] << std::endl;
    }

    void npe2eReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call)
    {

        static std::string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "APPROACH_GOAL"};
        int pre_s = int(exec_state_);
        exec_state_ = new_state;
        std::cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << std::endl;
    }

    void npe2eReplanFSM::publishTrajectory()
    {
        auto info = &planner_manager_->local_data_;

        plan_manage::PrimitiveTraj primitiveTraj;
        primitiveTraj.start_time = info->start_time_;
        primitiveTraj.duration = info->bestPrim_.getDuration();

        const Eigen::MatrixXd &coff_mat = info->bestPrim_.getCoeffs();

        primitiveTraj.piece_sizes.push_back(coff_mat.rows());
        primitiveTraj.piece_sizes.push_back(coff_mat.cols());

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> coeff_mat_row = coff_mat;
        primitiveTraj.coeffs.insert(primitiveTraj.coeffs.end(), coeff_mat_row.data(), coeff_mat_row.data() + coff_mat.size());

        primitiveTraj.rot_mat.resize(9);
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                primitiveTraj.rot_mat[i * 3 + j] = RWV(i, j);
            }
        }

        primitiveTraj.start_pos.resize(3);
        primitiveTraj.start_pos[0] = info->start_pos_(0);
        primitiveTraj.start_pos[1] = info->start_pos_(1);
        primitiveTraj.start_pos[2] = info->start_pos_(2);

        PrimitiveTraj_pub_.publish(primitiveTraj);
        std::cout << "Primitive trajectory published!" << std::endl;
    }

}
