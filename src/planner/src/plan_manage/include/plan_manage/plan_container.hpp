#ifndef _PLAN_CONTAINER_HPP_
#define _PLAN_CONTAINER_HPP_

#include "plan_manage/prim_traj.hpp"

#include "Eigen/Eigen"
#include "ros/ros.h"

namespace npe2e_planner
{
    struct LocalTrajData
    {

        ros::Time start_time_;
        Eigen::Vector3d start_pos_;

        PrimitiveTrajectory bestPrim_;
    };
}

#endif