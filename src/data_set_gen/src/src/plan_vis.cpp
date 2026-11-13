#include "data_set_gen/plan_vis.h"

namespace dataSet_gen
{
    PlanVisualizer::PlanVisualizer(ros::NodeHandle &nh)
        : nh_(nh),
          has_drone_pose_(false),
          has_planner_traj_(false),
          has_target_point_(false),
          has_lib_(false),
          has_cloud_(false)
    {
        drone_pub_ = nh_.advertise<visualization_msgs::Marker>("/plan_vis/robot", 10, true);
        plannedTraj_pub_ = nh_.advertise<visualization_msgs::Marker>("/plan_vis/plannedTraj", 2);
        target_point_pub_ = nh_.advertise<visualization_msgs::Marker>("/plan_vis/target_point", 1);
        lib_pub_ = nh_.advertise<visualization_msgs::Marker>("/plan_vis/primitive_library", 1);
        pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/cloud", 100);

        pub_timer_ = nh_.createTimer(ros::Duration(0.1), &PlanVisualizer::pubTimerCallback, this);
    }

    void PlanVisualizer::visualDronePose(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation)
    {
        position_drone_ = position;
        orientation_drone_ = orientation;

        has_drone_pose_ = true;
    }

    void PlanVisualizer::visualPlannedTraj(PrimitiveTrajectory *bestPrim_, const Eigen::Vector3d &start_pos, const Eigen::Matrix3d &RWV)
    {
        if (!bestPrim_)
        {
            return;
        }
        pts_plannedTraj_.clear();

        double T_alloc = bestPrim_->getDuration();
        double step_t = std::min(0.1, T_alloc / 10);
        for (double t = 0; t <= T_alloc; t += step_t)
        {
            if (t > T_alloc)
            {
                t = T_alloc;
            }
            Eigen::Vector3d pt = start_pos + RWV * bestPrim_->getPos(t);
            pts_plannedTraj_.push_back(pt);
        }

        has_planner_traj_ = true;
    }

    void PlanVisualizer::visualTargetPoint(const Eigen::Vector3d &target_pos)
    {
        target_pos_ = target_pos;

        has_target_point_ = true;
    }

    void PlanVisualizer::visualLib(const PrimitiveLibrary &primLib, const Eigen::Vector3d &start_pos, const Eigen::Matrix3d &RWV)
    {
        pts_lib_.clear();

        const auto &primitives = primLib.getLibrary();
        for (size_t i = 0; i < primitives.size(); ++i)
        {
            const auto &prim = primitives[i];
            std::vector<Eigen::Vector3d> point_set;
            double T_alloc = prim.getDuration();
            double step_t = std::min(0.1, T_alloc / 10);
            for (double t = 0; t <= T_alloc; t += step_t)
            {
                if (t > T_alloc)
                {
                    t = T_alloc;
                }
                Eigen::Vector3d pt = start_pos + RWV * prim.getPos(t);
                point_set.push_back(pt);
            }
            pts_lib_.push_back(point_set);
        }

        has_lib_ = true;
    }

    void PlanVisualizer::visualSensedCloud(const sensor_msgs::PointCloud2 &output_cloud)
    {

        output_cloud_ = output_cloud;

        has_cloud_ = true;
    }

    void PlanVisualizer::publishDrone()
    {

        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = "world";
        clear_marker.header.stamp = ros::Time::now();
        clear_marker.ns = "drone";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        drone_pub_.publish(clear_marker);

        visualization_msgs::Marker meshROS;
        meshROS.header.frame_id = "world";
        meshROS.header.stamp = ros::Time::now();
        meshROS.ns = "drone";
        meshROS.id = 0;
        meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
        meshROS.action = visualization_msgs::Marker::ADD;

        meshROS.pose.position.x = position_drone_.x();
        meshROS.pose.position.y = position_drone_.y();
        meshROS.pose.position.z = position_drone_.z();

        meshROS.pose.orientation.w = orientation_drone_.w();
        meshROS.pose.orientation.x = orientation_drone_.x();
        meshROS.pose.orientation.y = orientation_drone_.y();
        meshROS.pose.orientation.z = orientation_drone_.z();

        meshROS.scale.x = 0.5;
        meshROS.scale.y = 0.5;
        meshROS.scale.z = 0.5;

        meshROS.color.r = 0.0;
        meshROS.color.g = 0.0;
        meshROS.color.b = 0.0;
        meshROS.color.a = 1.0;

        meshROS.mesh_resource = std::string("package://data_set_gen/meshes/fake_drone.dae");

        drone_pub_.publish(meshROS);
    }

    void PlanVisualizer::publishPlannedTraj()
    {
        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = "world";
        clear_marker.header.stamp = ros::Time::now();
        clear_marker.ns = "plannedTraj";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        plannedTraj_pub_.publish(clear_marker);

        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "world";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = 0;
        line_strip.id = 0 + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 0;
        sphere.color.g = line_strip.color.g = 0;
        sphere.color.b = line_strip.color.b = 1;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = 0.2;
        sphere.scale.y = 0.2;
        sphere.scale.z = 0.2;
        line_strip.scale.x = 0.2 / 2;
        geometry_msgs::Point pt;
        for (int i = 0; i < int(pts_plannedTraj_.size()); i++)
        {
            pt.x = pts_plannedTraj_[i](0);
            pt.y = pts_plannedTraj_[i](1);
            pt.z = pts_plannedTraj_[i](2);
            sphere.points.push_back(pt);
            line_strip.points.push_back(pt);
        }
        plannedTraj_pub_.publish(sphere);
        plannedTraj_pub_.publish(line_strip);
    }

    void PlanVisualizer::publishTargetPoint()
    {
        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = "world";
        clear_marker.header.stamp = ros::Time::now();
        clear_marker.ns = "target_point";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        target_point_pub_.publish(clear_marker);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "target_point";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = target_pos_.x();
        marker.pose.position.y = target_pos_.y();
        marker.pose.position.z = target_pos_.z();

        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;

        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;

        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 1;

        target_point_pub_.publish(marker);
    }

    void PlanVisualizer::publishLib()
    {
        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = "world";
        clear_marker.header.stamp = ros::Time::now();
        clear_marker.ns = "primitive_library";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        lib_pub_.publish(clear_marker);

        visualization_msgs::Marker line_list;
        line_list.header.frame_id = "world";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "primitive_library";
        line_list.id = 0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.action = visualization_msgs::Marker::ADD;

        line_list.pose.orientation.w = 1.0;
        line_list.color.r = 0;
        line_list.color.g = 1;
        line_list.color.b = 0;
        line_list.color.a = 0.6;
        line_list.scale.x = 0.05;

        for (const auto &point_set : pts_lib_)
        {
            if (point_set.size() < 2)
            {
                continue;
            }
            for (size_t i = 0; i < point_set.size() - 1; ++i)
            {
                geometry_msgs::Point p1, p2;
                p1.x = point_set[i](0);
                p1.y = point_set[i](1);
                p1.z = point_set[i](2);
                p2.x = point_set[i + 1](0);
                p2.y = point_set[i + 1](1);
                p2.z = point_set[i + 1](2);
                line_list.points.push_back(p1);
                line_list.points.push_back(p2);
            }
        }

        lib_pub_.publish(line_list);
    }

    void PlanVisualizer::publishSensedCloud()
    {
        output_cloud_.header.stamp = ros::Time::now();
        pub_cloud_.publish(output_cloud_);
    }

    void PlanVisualizer::pubTimerCallback(const ros::TimerEvent &)
    {
        ros::Time stamp = ros::Time::now();

        if (has_drone_pose_)
        {
            publishDrone();
        }
        if (has_planner_traj_)
        {
            publishPlannedTraj();
        }
        if (has_target_point_)
        {
            publishTargetPoint();
        }
        if (has_lib_)
        {
            publishLib();
        }
        if (has_cloud_)
        {
            publishSensedCloud();
        }
    }
}